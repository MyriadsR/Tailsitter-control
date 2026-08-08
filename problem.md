# Future-trajectory CNN residual control: retained design rationale

## Problem

The analytical controller already uses reference acceleration and jerk, so a
future-trajectory network has limited room to improve a nominal, model-matched
simulation. A learned translational acceleration residual also cannot directly
solve modes dominated by angular moment or actuator allocation.

The important structural issue was feedforward inconsistency. Adding a residual
to the position-loop acceleration changed desired attitude, while `omega_ref`
still came from the unmodified trajectory acceleration and jerk. Fast transitions
could therefore receive mutually inconsistent attitude and angular-rate commands.

PPO additionally has to balance trajectories with different durations and error
scales. Optimizing a single absolute reward allowed long or high-error modes to
dominate, while an average improvement could hide a serious regression in one
mode.

## Retained solution

The selected policy uses:

- an `11 x 12` future-reference sequence encoded by two `Conv1d` layers;
- a separate 16-dimensional current-state MLP branch;
- a bounded three-axis body-frame acceleration residual at 50 Hz;
- the original flatness/INDI control chain at 500 Hz;
- a paired zero-residual baseline and bounded relative RMS reward;
- inverse-duration trajectory sampling and downside multiplier `2.0`;
- worst-case-aware fixed-suite checkpoint selection.

The residual is transformed to inertial coordinates, filtered at 10 Hz, and
differentiated to obtain residual jerk. The same filtered terms modify the
position-loop command and the acceleration/jerk used for attitude and angular-rate
feedforward:

```text
a_ref_star = a_ref + delta_a_i
j_ref_star = j_ref + delta_j_i
```

Because the available action is unsuitable for every flight regime, deployment
sets the residual scale to zero for `transition-to-hover` and `differential-turn`.
Circular knife-edge remains disabled by the safety gate.

## Result

The selected 50k checkpoint improves Lemniscate by `0.57%`, coordinated circle by
`8.54%`, knife-edge transition by `1.10%`, and transition-to-forward by `10.01%`.
The other evaluated modes return exactly to the analytical baseline through gates.
Across the six deployment cases, mean position-RMS improvement is `3.370379%`,
worst improvement is `0.00%`, and no retained case saturates a rotor or flap.

## Remaining limitations

- The result is based on one PPO seed and a nominal simulation model.
- Mode gates avoid known regressions rather than teaching one policy to solve all
  flight regimes.
- Differential turn would need a separately bounded angular-rate or moment residual,
  not a reopened translational acceleration residual.
- Any change to residual limits, gates, plant parameters, or observation layout
  requires a new multi-mode A/B evaluation before deployment.

The exact model, metadata, logs, plots, and per-case NPZ data retained by the
project are listed in `results/PreviewResidual/README.md`.
