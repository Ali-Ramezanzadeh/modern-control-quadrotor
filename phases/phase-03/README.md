# Phase 3 — Observers and practical constraints

The final phase extends integral LQR reference tracking with actuator constraints and full-order/reduced-order state observers. The report compares nominal tracking, mean absolute estimation error, noise sensitivity and a 20% plant-matrix change.

## Files

| File | Purpose |
| --- | --- |
| [Main.mlx](matlab/Main.mlx) | Original MATLAB Live Script |
| [Main.slx](matlab/Main.slx) | Original final Simulink model |
| [source_export.m](matlab/source_export.m) | Readable export of original code cells; untested |
| [report.fa.pdf](reports/report.fa.pdf) | Complete original Persian final report, 66 pages; also reviews Phases 1 and 2 |
| [report.en.md](reports/report.en.md) | Condensed English technical companion |
| [FIGURES.md](reports/FIGURES.md) | 89 extracted report images with page references |
| [assignment.fa.pdf](assignment/assignment.fa.pdf) | Original course assignment, 4 pages |
| [assignment.en.md](assignment/assignment.en.md) | English translation, including optional tasks |

## Scope and evidence

The code defines a 12-state full-order observer and an eight-state reduced-order observer using measurements `[x,y,z,psi]`. The final model includes noise, saturation, first-order motor branches, MAE subsystems and a 3D UAV visualization branch.

A Kalman Filter block is present but explicitly commented out. Rate Transition and Zero-Order Hold blocks are also present. These are documented model contents, not a claim that the optional hybrid Kalman/LQG or complete discrete-controller tasks were finished and validated.

The report and active Live Script use different integral weights and reduced-observer matrices. Read [the reproducibility notes](../../docs/REPRODUCIBILITY.md) when comparing figures to saved code. Reported observer advantages are specific to the submitted scenarios.

Open this phase's `Main.mlx`, initialize its workspace, and then open this phase's `Main.slx`. The saved stop time is 30 seconds. Core dependencies and extra requirements for the visualization branch are in the [run guide](../../docs/REPRODUCIBILITY.md). MATLAB/Simulink execution has not been verified during repository preparation.

[Back to the project](../../README.md)
