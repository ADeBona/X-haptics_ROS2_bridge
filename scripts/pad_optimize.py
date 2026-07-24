#!/usr/bin/env python3
"""
Hardware-in-the-loop Bayesian optimisation of reverse-PFM deflation bounds.

Searches (T_max, T_min) to minimise the mean squared error between the
measured pressure descent and an ideal linear ramp of duration T_ref
(typically the measured inflation rise time, giving a symmetric profile).

Surrogate: Gaussian process, Matern 5/2 kernel.
Acquisition: Expected Improvement.

Usage:
    python3 pad_optimize.py --port /dev/ttyACM0 \
        --start 50 --tref 2500 --trials 25 \
        --out ../pads/pad_v2_55x30_shore10
"""
import argparse
import csv
import json
import os
import time

import numpy as np
import serial

P_END = 2.0                 # descent terminates here (must match firmware)
GRID_DT = 0.02               # resample step, seconds


# ---------------------------------------------------------------- GP surrogate

def matern52(A, B, ls, var):
    """Matern 5/2 kernel on already-normalised inputs."""
    d = np.sqrt(np.maximum(
        ((A[:, None, :] - B[None, :, :]) / ls) ** 2, 1e-12).sum(-1))
    s5 = np.sqrt(5.0) * d
    return var * (1.0 + s5 + 5.0 / 3.0 * d ** 2) * np.exp(-s5)


class GP:
    def __init__(self, ls=0.3, var=1.0, noise=1e-4):
        self.ls, self.var, self.noise = ls, var, noise

    def fit(self, X, y):
        self.X = X
        self.y_mean = y.mean()
        self.y_std = y.std() + 1e-9
        yn = (y - self.y_mean) / self.y_std
        K = matern52(X, X, self.ls, self.var) + self.noise * np.eye(len(X))
        self.L = np.linalg.cholesky(K)
        self.alpha = np.linalg.solve(self.L.T, np.linalg.solve(self.L, yn))

    def predict(self, Xs):
        Ks = matern52(self.X, Xs, self.ls, self.var)
        mu = Ks.T @ self.alpha
        v = np.linalg.solve(self.L, Ks)
        var = np.maximum(self.var - (v ** 2).sum(0), 1e-12)
        return mu * self.y_std + self.y_mean, np.sqrt(var) * self.y_std


def expected_improvement(mu, sd, best, xi=0.01):
    from math import erf, sqrt
    imp = best - mu - xi
    z = imp / sd
    Phi = 0.5 * (1.0 + np.array([erf(v / sqrt(2.0)) for v in z]))
    phi = np.exp(-0.5 * z ** 2) / np.sqrt(2.0 * np.pi)
    return imp * Phi + sd * phi


# ---------------------------------------------------------------- objective

def median3(x):
    if len(x) < 3:
        return x
    out = x.copy()
    out[1:-1] = np.median(np.stack([x[:-2], x[1:-1], x[2:]]), axis=0)
    return out


def score(trace, p_start_nominal, t_ref_s):
    """
    MSE against an ideal linear descent, anchored on the MEASURED start
    pressure rather than the nominal target, so trial-to-trial variation in
    the starting point does not masquerade as a parameter effect.
    """
    if len(trace) < 10:
        return 1e6, 0.0, 0.0
    t = np.array([r[0] for r in trace], float)
    p = median3(np.array([r[2] for r in trace], float))
    t = (t - t[0]) / 1000.0

    p0 = float(np.median(p[:5]))          # actual start
    t_actual = float(t[-1])
    t_grid = np.arange(0.0, max(t_actual, t_ref_s) + GRID_DT, GRID_DT)

    meas = np.interp(t_grid, t, p, left=p[0], right=p[-1])
    ideal = np.clip(p0 - (p0 - P_END) * t_grid / t_ref_s, P_END, None)

    return float(np.mean((meas - ideal) ** 2)), t_actual, p0


# ---------------------------------------------------------------- hardware

class Rig:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, baud, 0.1
        self.ser.dtr = False
        self.ser.open()
        self.ser.dtr = False
        time.sleep(0.1)
        self.ser.dtr = True
        time.sleep(3.0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def trial(self, p_start, t_max, t_min, timeout=180.0):
        self.ser.reset_input_buffer()
        self.ser.write(f'F,{p_start:.2f},{int(t_max)},{int(t_min)}\n'.encode())
        self.ser.flush()

        trace, recording, result = [], False, None
        t0 = time.time()
        while time.time() - t0 < timeout:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                continue
            if line.startswith('EVT,deflate start'):
                recording, trace = True, []
            elif line.startswith('D,') and recording:
                try:
                    _, t, tgt, act, _mode = line.split(',')
                    trace.append((int(t), float(tgt), float(act)))
                except ValueError:
                    pass
            elif line.startswith('RESULT,DEFLATE'):
                result = line
                break
            elif line.startswith('EVT,') and 'aborted' in line:
                break
        self.ser.write(b'Z\n')
        self.ser.flush()
        return trace, result

    def close(self):
        try:
            self.ser.write(b'Z\n')
            self.ser.flush()
            time.sleep(0.5)
        finally:
            self.ser.close()


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--start', type=float, required=True,
                    help='descent start pressure, kPa')
    ap.add_argument('--tref', type=float, required=True,
                    help='reference duration, ms (use the inflation rise time)')
    ap.add_argument('--trials', type=int, default=25)
    ap.add_argument('--init', type=int, default=6, help='random seed trials')
    ap.add_argument('--out', default='../pads/pad_current')
    args = ap.parse_args()

    outdir = os.path.abspath(args.out)
    os.makedirs(outdir, exist_ok=True)
    t_ref_s = args.tref / 1000.0

    # search space
    lo = np.array([40.0, 10.0])       # T_max, T_min lower bounds
    hi = np.array([250.0, 120.0])     # upper bounds

    def norm(X):
        return (X - lo) / (hi - lo)

    rng = np.random.default_rng(0)
    X_raw, y = [], []
    history = []

    rig = Rig(args.port)
    print(f'Reference duration {t_ref_s:.2f} s, start {args.start:.1f} kPa, '
          f'{args.trials} trials\n')

    try:
        for i in range(args.trials):
            if i < args.init:
                cand = lo + rng.random(2) * (hi - lo)
                if cand[1] >= cand[0]:
                    cand[1] = cand[0] * 0.5
                origin = 'random'
            else:
                gp = GP()
                gp.fit(norm(np.array(X_raw)), np.array(y))
                pool = lo + rng.random((4000, 2)) * (hi - lo)
                pool = pool[pool[:, 1] < pool[:, 0]]        # enforce T_min<T_max
                mu, sd = gp.predict(norm(pool))
                ei = expected_improvement(mu, sd, min(y))
                cand = pool[int(np.argmax(ei))]
                origin = 'BO'

            t_max, t_min = float(cand[0]), float(cand[1])
            print(f'[{i+1:2d}/{args.trials}] {origin:6s} '
                  f'T_max={t_max:6.1f}  T_min={t_min:6.1f} ... ', end='', flush=True)

            trace, result = rig.trial(args.start, t_max, t_min)
            mse, dur, p0 = score(trace, args.start, t_ref_s)
            print(f'MSE={mse:8.3f}  dur={dur:5.2f}s  p0={p0:5.1f}  n={len(trace)}')

            X_raw.append([t_max, t_min])
            y.append(mse)
            history.append({'trial': i + 1, 'origin': origin,
                            'T_max': t_max, 'T_min': t_min,
                            'mse': mse, 'duration_s': dur, 'p_start_meas': p0,
                            'samples': len(trace)})

            # keep the best trace for plotting
            if mse == min(y):
                best_trace = trace

            time.sleep(2.0)     # let the pad settle between trials

    finally:
        rig.close()

    best = int(np.argmin(y))
    T_max_opt, T_min_opt = X_raw[best]
    print(f'\n=== OPTIMUM ===')
    print(f'  T_max = {T_max_opt:.0f} ms')
    print(f'  T_min = {T_min_opt:.0f} ms')
    print(f'  MSE   = {y[best]:.3f}')

    with open(os.path.join(outdir, 'optimisation_history.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(history[0].keys()))
        w.writeheader()
        w.writerows(history)

    with open(os.path.join(outdir, 'optimum.json'), 'w') as f:
        json.dump({'T_max_ms': round(T_max_opt),
                   'T_min_ms': round(T_min_opt),
                   'mse': y[best],
                   'p_start_kPa': args.start,
                   'p_end_kPa': P_END,
                   't_ref_ms': args.tref,
                   'trials': args.trials}, f, indent=2)

    if 'best_trace' in dir() and best_trace:
        t0 = best_trace[0][0]
        with open(os.path.join(outdir, 'deflation_best.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time_s', 'target_kPa', 'actual_kPa'])
            for t, tgt, act in best_trace:
                w.writerow([f'{(t-t0)/1000:.3f}', f'{tgt:.2f}', f'{act:.2f}'])

    print(f'\nWrote optimum.json, optimisation_history.csv, deflation_best.csv '
          f'to {outdir}')


if __name__ == '__main__':
    main()