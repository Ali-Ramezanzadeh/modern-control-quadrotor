# Quadrotor Modeling and Modern Control

A three-phase team course project on modeling, feedback control, reference tracking, and state estimation for a quadrotor using MATLAB and Simulink.

The project develops a nonlinear model, linearizes it around hover, and studies the resulting 12-state system. Subsequent phases implement pole-placement and LQR feedback, static and integral reference tracking, and full-order and reduced-order observers. The final report compares performance with actuator constraints, measurement noise, and a 20% change in the plant state matrix.

**Course:** Modern Control, Department of Electrical Engineering, Amirkabir University of Technology (Tehran Polytechnic). **Instructor:** Dr. Hajar Atrianfar. This repository preserves the submitted project files and adds English documentation for readers of the original Persian reports.

## Explore the project

| Phase | Main work | Start here |
| --- | --- | --- |
| 1 | Nonlinear modeling, hover linearization, Jordan form, transfer functions, stability, controllability and observability | [Phase 1](phases/phase-01/README.md) |
| 2 | Pole placement, LQR, linear/nonlinear comparisons, reference tracking and disturbance scenarios | [Phase 2](phases/phase-02/README.md) |
| 3 | Integral LQR, actuator constraints, full/reduced observers, MAE, noise and model mismatch | [Phase 3](phases/phase-03/README.md) |

Each phase includes its original MATLAB Live Script (`.mlx`), Simulink model (`.slx`), Persian report and assignment, an English assignment translation, a condensed English technical companion to the report, and extracted report figures. Plain `.m` exports make the original code readable on GitHub.

## Example result from the submitted report

![Reported linear/nonlinear response and reference tracking under integral LQR](phases/phase-03/reports/figures/p051-img02.png)

Original Phase 3 report, PDF page 51. This figure is a historical submitted result; simulations have not been rerun for this repository. See the [English Phase 3 companion](phases/phase-03/reports/report.en.md) for context and source differences.

## Open and run

Use MATLAB with Simulink, Control System Toolbox and Symbolic Math Toolbox. The later Live Scripts record MATLAB R2022b metadata; Phase 1 records R2021b. These are file metadata, not a tested compatibility guarantee.

Open one phase at a time, run its Live Script sections to populate the MATLAB workspace, then open its Simulink model and select the intended scenario. The models use workspace variables and manual switches. Read [reproducibility and known issues](docs/REPRODUCIBILITY.md) before running the historical code. The final model also contains a 3D UAV visualization branch with additional toolbox requirements.

## Model and evidence

The state vector is `[x, x_dot, y, y_dot, z, z_dot, phi, theta, psi, p, q, r]`. The inputs are total thrust and roll, pitch, and yaw torques. The measured outputs used for the observer design are `[x, y, z, psi]`.

- [Model equations, parameters and control structure](docs/MODEL.md)
- [Verification performed for this repository](docs/VERIFICATION.md)
- [Team credits and source provenance](CREDITS.md)
- [References](docs/REFERENCES.md)
- [Original presentation link](presentation/README.md)

Original files have been preserved byte for byte and checked with SHA-256. Independent algebra checks confirm full controllability and observability of the reconstructed hover model. MATLAB/Simulink execution and the report plots have not been independently reproduced. Specific numerical and report/code inconsistencies are documented in the reproducibility guide.

## Authorship

This is a team academic project. The original report authors are credited in [CREDITS.md](CREDITS.md). The repository does not assign individual ownership of the modeling, code or results.

No license has been added to the original team materials. The repository documents their source; redistribution permissions are not established by this draft.
