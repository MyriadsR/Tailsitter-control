# Future-trajectory CNN consistent residual controller

## Control integration

The policy receives a 16-dimensional current-state vector and a `11 x 12`
future-reference sequence. Two temporal `Conv1d` layers encode the preview, an
MLP encodes the current state, and PPO's actor produces a normalized three-axis
body-frame acceleration residual. The deployment limit is `0.5 m/s^2` per axis.

Adding a residual only to the position-loop acceleration changes desired attitude
without changing the angular-rate feedforward derived from trajectory acceleration
and jerk. The retained implementation removes that inconsistency:

```text
delta_a_i_raw = R_b_to_i * delta_a_b
delta_a_i     = first_order_low_pass(delta_a_i_raw, 10 Hz)
delta_j_i     = d(delta_a_i) / dt
a_ref_star    = a_ref + delta_a_i
j_ref_star    = j_ref + delta_j_i
```

The same filtered `delta_a_i` is added to the position-loop command. The modified
`a_ref_star` and `j_ref_star` are also used to construct reference force, attitude,
and `omega_ref`, so attitude and angular-rate feedforward see the same learned
correction. NPZ logs expose the commanded body residual, applied inertial residual,
and residual jerk.

## PPO objective and sampling

Training advances a zero-residual baseline with the same trajectory and initial
condition as the policy environment. Position and velocity improvements use
bounded symmetric normalization:

```text
r_p = (||e_p,b||^2 - ||e_p,pi||^2)
      / (||e_p,b||^2 + ||e_p,pi||^2 + 0.01)
r_v = (||e_v,b||^2 - ||e_v,pi||^2)
      / (||e_v,b||^2 + ||e_v,pi||^2 + 0.25)
r   = r_p + 0.02*r_v
      - 0.002*||action||^2
      - 0.001*||delta_action||^2
```

`r_p` and `r_v` are clipped to `[-1, 1]`, and negative improvement receives a
`2.0` multiplier. Trajectories are sampled inversely to duration so the six modes
contribute approximately equal transition counts. Fixed-suite checkpoint scoring is:

```text
selection_score = mean_improvement
                  + 0.5 * min(worst_improvement, 0)
```

## Training and selection

- PPO seed `7`, learning rate `3e-4`
- `100,000` requested transitions (`100,352` rollout-aligned)
- policy/control rates `50/500 Hz`
- 12 preview points over `0.6 s`
- training/deployment residual limit `0.5 m/s^2`
- validation every `25,000` transitions

The ungated fixed-suite history was:

| Transitions | Mean improvement | Worst improvement | Selection score |
| ---: | ---: | ---: | ---: |
| 0 | +0.01% | -0.10% | -0.04 |
| 25,000 | +1.22% | -4.33% | -0.95 |
| 50,000 | +2.12% | -5.85% | -0.81 |
| 75,000 | +1.51% | -14.17% | -5.58 |
| 100,000 | +3.78% | -9.02% | -0.73 |

The 50k checkpoint was selected because all four modes suited to translational
acceleration residuals improved there. Explicit zero-residual gates then return
`transition-to-hover` and `differential-turn` to the analytical controller.
Circular knife-edge retains its independent safety gate.

## Deployment result

| Trajectory | Baseline RMS | Policy RMS | Change |
| --- | ---: | ---: | ---: |
| Lemniscate | 0.182 m | 0.181 m | +0.57% |
| Coordinated circle | 0.125 m | 0.114 m | +8.54% |
| Knife-edge transition | 0.520 m | 0.514 m | +1.10% |
| Transition to forward | 0.151 m | 0.136 m | +10.01% |
| Transition to hover | 0.142 m | 0.142 m | 0.00% (mode-gated) |
| Differential turn | 0.755 m | 0.755 m | 0.00% (mode-gated) |
| Circular knife edge | 2.941 m | 2.941 m | 0.00% (safety-gated) |

Across the six deployment cases, mean improvement is `+3.370379%`, worst
improvement is `0.00%`, and selection score is `+3.370379`. No evaluated case
has rotor or flap saturation. A post-cleanup rerun reproduced every stored
`rms_improvement_percent` value to an absolute tolerance of `1e-12`.

The retained artifact is
`saved_models/preview_residual_consistent_gated.{zip,json}`. Its limits and gates
must remain paired with the model; changing either requires a new complete A/B
evaluation. This result is a simulation result and is not a real-flight safety
qualification.

