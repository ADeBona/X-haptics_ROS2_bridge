#!/usr/bin/env python3
"""Plot inflation and optimised deflation on one axis, showing symmetry."""
import argparse, csv, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load(path):
    t, a = [], []
    with open(path) as f:
        for row in csv.reader(f):
            if not row or row[0].startswith('#') or row[0] == 'time_s':
                continue
            t.append(float(row[0])); a.append(float(row[2]))
    return np.array(t), np.array(a)


def trim_to_motion(t, p, thresh=1.0):
    """Drop the leading flat section before pressure starts changing."""
    d = np.abs(np.diff(p))
    idx = np.argmax(d > thresh) if np.any(d > thresh) else 0
    return t[idx:] - t[idx], p[idx:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    ap.add_argument('--gap', type=float, default=1.0,
                    help='hold duration drawn between the two phases (s)')
    args = ap.parse_args()
    d = os.path.abspath(args.dir)

    ti, pi = trim_to_motion(*load(os.path.join(d, 'inflation.csv')))
    td, pd = load(os.path.join(d, 'deflation_best.csv'))
    td = td - td[0]

    t_off = ti[-1] + args.gap
    t_hold = np.array([ti[-1], t_off])
    p_hold = np.array([pi[-1], pd[0]])

    plt.figure(figsize=(9.5, 5.5))
    plt.plot(ti, pi, 'b-', lw=2, label='Inflation (pump, bang-bang)')
    plt.plot(t_hold, p_hold, color='0.6', lw=2, ls='-', label='Hold')
    plt.plot(td + t_off, pd, 'g-', lw=2, label='Deflation (reverse-PFM)')

    # ideal symmetric reference
    plt.plot([t_off, t_off + ti[-1]], [pd[0], 10.0],
             'r--', lw=1.5, alpha=0.8, label='Ideal linear descent')

    plt.xlabel('Time (s)'); plt.ylabel('Pressure (kPa)')
    plt.title('Pad v2 (55×30×5 mm, Shore 10): inflation / deflation symmetry')
    plt.grid(True, ls=':', alpha=0.6); plt.legend(loc='best')
    plt.tight_layout()
    out = os.path.join(d, 'symmetry.png')
    plt.savefig(out, dpi=150)
    print(f'-> {out}')
    print(f'   inflation {ti[-1]:.2f} s   deflation {td[-1]:.2f} s')


if __name__ == '__main__':
    main()