# Finding axis_vertical, axis_lateral, lever_sign, tighten_sign

| Parameter       | What it is                                                    |
| --------------- | -------------------------------------------------------------- |
| `axis_vertical`  | which of the sensor's 3 torque channels is "twist about the bolt" |
| `axis_lateral`   | which of the sensor's 3 force channels is "sideways push"       |
| `lever_sign`     | +1 or -1, whichever makes the lever-arm correction cancel, not add to, the false twist |
| `tighten_sign`   | +1 or -1, whichever makes **tightening** read out as positive `tau_screw` / `u` (see §5) |

Index order for both is `0 = x, 1 = y, 2 = z` — same order the wrench message
uses.

***

## 0. Setup

1. Start the bridge in REAL mode with the params file (GUIDE.md §3).
2. Open the CSV log in a second terminal on the **Pi** (not in the container —
   it's host-visible now) and watch it scroll:

   ```bash
   tail -f ~/kinova_haptic_ws/logs/kinova_haptic_bridge_log.csv
   ```

   Columns, in order: `t, fx, fy, fz, tx, ty, tz, tau_raw, tau_screw, tau_perp_mag, u, pA, pB`.
3. Put the arm in the actual task pose (parallel to the table, over the bolt), but don't touch anything yet.

### Tare *before* doing anything else below

There are **two separate zero-points** in this system and it's easy to
confuse them:

- **Arduino startup tare** — happens automatically ~2 s after the serial
  port opens (firmware zeroes the pressure ADC). You don't do anything for
  this one; the bridge already waits for it.
- **ROS-level screw-torque tare** — zeroes `tau_screw`, and it is **not
  automatic**. `TareEstimator.offset` starts at `0.0` on node launch and
  stays exactly `0.0` until you explicitly publish to `~/tare`. Starting the
  bridge does *not* tare it.

Skipping the second one is the single most common cause of "the pad is
already inflated before I've touched anything" / "the numbers don't scale
the way I expect." Here's why: `tau_screw` is not the raw torque reading —
it already has the lever-arm correction baked in:

```
tau_screw = tau_raw + lever_sign * lever_L * f_lateral
```

At rest, `tau_raw` (the `tx`/`ty`/`tz` column you picked as `axis_vertical`)
is genuinely near zero — but `f_lateral` usually isn't (the driver's own
resting weight/contact load shows up there), and the lever arm multiplies
it. E.g. with `f_lateral ≈ -4 N`, `lever_L = 0.15`, `lever_sign = -1`:
`tau_screw ≈ 0 + (-1)(0.15)(-4.0) ≈ 0.6 Nm` — well above `tau_deadband`
(0.05), so it sails straight through into `u` and out as pressure, even
though nothing is being tightened. Watching `tx` and thinking "that's ~0, so
I'm zeroed" checks the wrong column.

Changing `tau_max` does **not** fix this — it only rescales
`u = (|tau_tared| - tau_deadband) / (tau_max - tau_deadband)`, so a bigger
`tau_max` just makes the *same* untared 0.6 Nm baseline map to a smaller,
still-nonzero, pressure. It looks like progress but the offset is still
there.

**So, before step 1 below**, with the arm at rest and nothing touching the
driver:

```bash
ros2 topic pub /kinova_haptic_bridge/tare std_msgs/msg/Empty "{}" --once
```

Watch the bridge log for `Tare captured: offset=... Nm` — it averages the
next `tare_samples` (20, ≈1 s at 20 Hz) samples of `tau_screw` and freezes
that as the offset. It will **not** drift after that; if you move the arm to
a different pose, or bump the driver, re-tare rather than chasing it with
`tau_deadband`. Reset it back to zero at any point with:

```bash
ros2 topic pub /kinova_haptic_bridge/reset_tare std_msgs/msg/Empty "{}" --once
```

With a fresh, good tare, `tau_screw` in the CSV should sit close to `0` at
rest — *that's* the check to use, not `tx`/`ty`/`tz` alone.

***

## 1. Find `axis_vertical`

**Action:** by hand, hold the nut driver/socket and twist it as if screwing —
a pure rotation about the vertical bolt axis, no sideways push.

**Watch:** `tx`, `ty`, `tz`. Exactly one of them should swing noticeably while
you twist; the other two should stay close to their resting values.

**Set** `axis_vertical` to the index of the one that moved (`tx`→0, `ty`→1,
`tz`→2).

If two of them move together, you're probably also pushing sideways by
accident — brace your hand and try again with a cleaner pure twist.

***

## 2. Find `axis_lateral`

**Action:** now do the opposite motion — push the driver sideways (in the
horizontal plane, the direction a slip or a wobble would push it), with
**no twisting at all**.

**Watch:** `fx`, `fy`, `fz`. One of them should swing with the push; that's
your candidate `axis_lateral`. (`fz` is usually vertical/gravity-ish and is
rarely the right answer here, but don't assume — check the number.)

**Also watch:** `tau_raw` (the column, not `tx/ty/tz` directly — it's already
picked out by whatever you set `axis_vertical` to in step 1). A pure sideways
push with no real twist should still make `tau_raw` move — that's the false
twist this whole feature exists to cancel. If `tau_raw` doesn't move at all
during a sideways push, you likely have `axis_vertical` wrong; go back to
step 1.

**Set** `axis_lateral` to the force index that moved.

***

## 3. Find `lever_sign`

Now that `axis_vertical` and `axis_lateral` are set, repeat the **pure
sideways push, no twist** from step 2, and compare two columns side by side:
`tau_raw` (uncorrected) vs `tau_screw` (corrected).

- If `tau_screw` moves **less** than `tau_raw` during the push — good, the
  correction is cancelling the false twist. Keep `lever_sign` as it is.
- If `tau_screw` moves **more** than `tau_raw` — the correction is adding to
  the false twist instead of cancelling it. Flip `lever_sign` (1 → -1 or
  -1 → 1) and repeat the push to confirm `tau_screw` now stays flatter than
  `tau_raw`.

`lever_L` matters here too — if it's very wrong (e.g. 0.20 m when the real
arm is 0.05 m), the cancellation will be partial even with the right sign.
Measure `lever_L` on the actual driver/socket (sensor origin to nut centre)
and set that before judging the sign.

Changing `lever_sign` changes `tau_screw`'s resting value, which changes
what the tare offset should be. **Re-tare after settling `lever_sign`**,
before moving on to step 4.

***

## 4. Sanity checks before you trust it

1. **At rest, no touching, freshly tared:** `tau_raw` and `tau_screw` should
   both sit close to `0` and stay steady. If `tau_screw` doesn't settle near
   zero here, the tare offset is stale or was captured while the arm was
   loaded — re-tare with the arm genuinely at rest (§0) before blaming the
   axes.
2. **Pure twist (step 1's motion again):** `tau_screw` should move about the
   same as `tau_raw` did — the correction shouldn't be doing much here, since
   there's no lateral force this time.
3. **Pure sideways push (step 2/3's motion):** `tau_screw` should stay much
   flatter than `tau_raw`.
4. **`tau_perp_mag`** (the two torque channels you *didn't* pick) should stay
   small in all of the above. If it's consistently large, you likely have
   the wrong `axis_vertical` even if steps 1–3 looked plausible individually.

If all four hold, set `tau_max` from the peak *tared* `tau_screw` seen during
a real screwing pass, and `tau_deadband` just above the at-rest noise floor
you just observed in check 1.

***

## 5. Confirming `tighten_sign`

None of steps 1–4 tell you whether **tightening** (the direction you
actually want to render) comes out as positive or negative `tau_screw`.
`lever_sign` only controls the sideways-push correction term — it is not a
"flip the whole result" switch. That's what `tighten_sign` is for: it's
applied last, after tare, purely to pick which physical twist direction
counts as "tightening":

```python
tau_tared = tighten_sign * tare.apply(tau_screw)
```

The pipeline wants **tightening → positive `u` → group_A (`pA`) inflates**,
since `pA` is the one wired channel. With a fresh tare in place, do a real
screwing pass by hand and watch `u`/`pA` in the CSV:

- If `u` (and `pA`) rises while tightening — `tighten_sign = 1` is correct,
  nothing else to do.
- If `u` goes negative instead while tightening (`pB` would be the one
  rising, but nothing is wired to it) — set `tighten_sign = -1` and repeat
  the pass to confirm `u`/`pA` now rises on tightening.

**Don't use `lever_sign` for this** — that will break the sideways-push
cancellation you tuned in step 3. `tighten_sign` exists specifically so you
never have to.

***

## Quick recap: order of operations

1. Setup + **tare** (§0) — always first, and again any time the arm moves or
   `lever_sign` changes.
2. `axis_vertical` (§1).
3. `axis_lateral` (§2).
4. `lever_sign` (§3), then **re-tare**.
5. Sanity checks (§4).
6. `tighten_sign` (§5).
7. Set `tau_max` / `tau_deadband` from the tared, sign-correct signal.
