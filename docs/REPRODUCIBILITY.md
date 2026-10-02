# Reproducibility and known issues

The source `.mlx` and `.slx` files are unchanged historical submissions. No MATLAB installation was available during repository preparation. The instructions below describe how to inspect and attempt the original workflows; successful execution has not been verified.

## Environment

- MATLAB, Simulink, Control System Toolbox and Symbolic Math Toolbox are needed for the core code (`syms`, `jacobian`, `jordan`, `ss`, `ctrb`, `obsv`, `place`, `lqr`, `lyap`).
- Live Script metadata records R2021b for Phase 1 and R2022b for Phases 2 and 3. Use R2022b or a compatible newer release as a starting point; earlier-release support is untested.
- The final model contains Simulation 3D UAV Vehicle and Simulation 3D Scene Configuration blocks. The UAV block belongs to UAV Toolbox, and MathWorks documents an additional Simulink 3D Animation requirement for simulation. See [official block documentation](https://www.mathworks.com/help/uav/ref/simulation3duavvehicle.html). License availability and graphical execution remain unverified.

## Open a phase

1. Open MATLAB and change the Current Folder to the selected phase's `matlab/` folder.
2. Start from a clean workspace. Run the original Live Script in order, inspecting each section's output. Use the plain `.m` export to review code on GitHub; its code-cell text matches the original and therefore retains historical issues.
3. Open the corresponding model only after its required variables have been initialized.
4. Inspect manual switches, reference/disturbance signals, observer selection, saturation branches and the plant matrix used by the selected scenario.
5. Run the intended scenario and save figures and numeric results separately. State the active configuration and solver when reporting reproduced results.

| Phase | Live Script | Simulink model | Saved stop time |
| --- | --- | --- | ---: |
| 1 | `Phase1.mlx` | `Phase1_simulink.slx` | 1 s |
| 2 | `Main.mlx` | `Main.slx` | 30 s |
| 3 | `Main.mlx` | `Main.slx` | 30 s |

The two `Main.slx` files are different models. Open them one at a time; both resolve to the MATLAB model name `Main`. The Live Scripts mostly initialize variables and generate analytical plots; they do not automatically execute all manual Simulink scenarios.

MATLAB Command Window examples, from the relevant phase's `matlab/` folder:

```matlab
% Phase 1
open('Phase1.mlx')
% Run and review Live Script sections first, then:
open_system('Phase1_simulink.slx')
```

```matlab
% Phase 2 OR Phase 3, in that phase's own folder
open('Main.mlx')
% Run and review Live Script sections first, then:
open_system('Main.slx')
```

## Issues identified during source review

### Stabilizability and detectability messages

The original code selects eigenvalues with a scalar Boolean expression such as `eigenvalues(rank(Ctrb)==n)`. This selects an element, rather than identifying uncontrollable modes. It then checks whether that value has negative real part. For this fully controllable hover model with zero open-loop eigenvalues, that logic produces an incorrect negative stabilizability result. The corresponding observability/detectability logic has the same problem.

Exact independent rank checks show all 12 states are controllable and observable. Consequently, the nominal model is stabilizable and detectable. The contradictory statements in the report and original console text are retained as source history, not accepted as verified conclusions.

### Phase 1 staircase decomposition

Phase 1 interprets `k(1)` and `k(2)` from `ctrbf` as controllable and observable dimensions. MathWorks defines `k` as the number of controllable states extracted at each staircase iteration; `sum(k)` gives the total controllable order. It is not an observability-dimension vector. The script also overwrites the matrices returned by `ctrbf` using a different transformation convention. The subsequent partitions and `ss(...)` construction therefore need review and can prevent the script from finishing. See [official ctrbf documentation](https://www.mathworks.com/help/control/ref/ctrbf.html).

For the actual hover model, all states are controllable and observable, so its minimal order remains 12. No physical order reduction is required.

### Report/code tuning differences

- Phase 1's narrative describes some initial conditions as ones; the Live Script sets `Condition_1=0.2*ones(12,1)` and uses a different explicit `x0` vector for the analytical response.
- Phase 1's report parameter table and the later scripts use different inertias and rotor coefficients. Do not substitute one phase's parameter set into another without recording that change.
- Phase 3 report PDF page 50 uses integral-weight entry `3500`; the active Live Script uses `3000` at the corresponding position.
- Phase 3 report PDF page 55 shows a different reduced-order `F` and `L_RO` tuning from the active Live Script. Treat report plots and code as potentially different revisions.
- The final report says the integral-augmented pair is not controllable. Exact rational checks of the matrices in the supplied script give rank 16 of 16. A large raw controllability matrix can give misleading numerical rank results; document the tolerance and use a suitable rank/decomposition method when reproducing this check.

### Final model scenario status

The Phase 3 model includes an explicitly **commented-out Kalman Filter block** configured as discrete time but referencing the continuous hover matrices. Its presence is not evidence of a completed or validated hybrid Kalman/LQG experiment. Do not activate it without reviewing discretization, covariance dimensions, sampling, connections and estimator configuration.

Two Rate Transition blocks and a Zero-Order Hold are present; one Rate Transition is commented out. These blocks do not establish that the entire observer/controller has been discretized. The saved model includes multiple alternate and commented branches, including motor, PID and visualization-related work.

Four saved noise blocks use sample times of `0.01`, `0.1`, `0.1`, and `0.1` seconds, with different noise powers. The assignment asks for a 100 Hz noise scenario; do not describe every saved noise branch as uniformly configured to 100 Hz.

The report evaluates `1.2*A`; reproducing it requires confirming the selected plant branch uses the perturbed matrix while the gains remain nominal. The report's qualitative comparisons are scenario-specific; no numeric MAE table or independently reproduced flight experiment is provided by this repository.

## Changes made while organizing the repository

The original binary files were copied without modification. Added materials are English documentation, source-code exports, extracted report figures, inventories and an independent algebra-check script. No correction has been silently applied to the original code or models.
