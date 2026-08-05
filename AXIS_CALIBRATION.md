# Finding axis_vertical, axis_lateral, lever_sign 


| Parameter       | What it is                                                    |
| --------------- | -------------------------------------------------------------- |
| `axis_vertical`  | which of the sensor's 3 torque channels is "twist about the bolt" |
| `axis_lateral`   | which of the sensor's 3 force channels is "sideways push"       |
| `lever_sign`     | +1 or -1, whichever makes the lever-arm correction cancel, not add to, the false twist |

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

***

## 4. Sanity checks before you trust it

1. **At rest, no touching:** `tau_raw` and `tau_screw` should be small and
   steady (not exactly zero until you've tared — see GUIDE.md §6, step 1).
2. **Pure twist (step 1's motion again):** `tau_screw` should move about the
   same as `tau_raw` did — the correction shouldn't be doing much here, since
   there's no lateral force this time.
3. **Pure sideways push (step 2/3's motion):** `tau_screw` should stay much
   flatter than `tau_raw`.
4. **`tau_perp_mag`** (the two torque channels you *didn't* pick) should stay
   small in all of the above. If it's consistently large, you likely have
   the wrong `axis_vertical` even if steps 1–3 looked plausible individually.

If all four hold, tare (GUIDE.md §6 step 1) and set `tau_max` from a real
screwing pass.

***

## One open question we have not settled yet

None of the above tells you whether **tightening** (the direction you
actually want to render) comes out as positive or negative `tau_screw`.
`lever_sign` only controls the sideways-push correction term — it is not a
"flip the whole result" switch, and there's currently no parameter that
inverts `tau_raw`'s own sign.

Right now the pipeline assumes **tightening → positive `u` → group_A
(`pA`) inflates**, since that's the one wired channel. Watch the CSV during
a real screwing pass:

- If `u` (and `pA`) goes up while tightening — matches expectations, nothing
  else to do.
- If it goes negative instead (`pB` would be the one rising, but nothing is
  wired to it) — the sensor's raw sign convention has tightening reading
  negative on the chosen axis, and none of the current parameters can flip
  just that. We'd need to add one more knob for it (or physically swap which
  motion you call "tightening", if that's an option). **Don't hack around it
  by misusing `lever_sign`** — that will break the sideways-push cancellation
  you just tuned in step 3. Flag it and we'll add the right parameter rather
  than fudge an existing one.

This is exactly what tomorrow's rosbag session should settle, alongside the
actual axis/sign values.
