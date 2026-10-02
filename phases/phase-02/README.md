# Phase 2 — Feedback and reference tracking

This phase builds on the hover model to design pole-placement and LQR controllers, compare linear/nonlinear closed-loop behavior, and implement static and integral reference compensation. Reference generators and disturbance scenarios support the comparisons.

## Files

| File | Purpose |
| --- | --- |
| [Main.mlx](matlab/Main.mlx) | Original MATLAB Live Script |
| [Main.slx](matlab/Main.slx) | Original Phase 2 Simulink model |
| [source_export.m](matlab/source_export.m) | Readable export of original code cells; untested |
| [report.fa.pdf](reports/report.fa.pdf) | Complete original Persian report, 27 pages |
| [report.en.md](reports/report.en.md) | Condensed English technical companion |
| [FIGURES.md](reports/FIGURES.md) | 9 extracted report images with page references |
| [assignment.fa.pdf](assignment/assignment.fa.pdf) | Original course assignment, 2 pages |
| [assignment.en.md](assignment/assignment.en.md) | English translation of the substantive instructions |

## Main comparisons

The report discusses overshoot and settling time, tests a dominant pole pair at `-0.5 ± 0.5j`, and compares pole-placement with LQR tracking. The Live Script augments the model with four output-error integrators. See [the model guide](../../docs/MODEL.md) for state and reference-channel conventions.

The inertia parameters differ from Phase 1. The report's nonlinear attraction-region observations are simulation observations, not a certified region of attraction.

Open this phase's `Main.mlx`, initialize its workspace, and then open this phase's `Main.slx`. The saved stop time is 30 seconds. Do not open Phase 3's identically named `Main` model simultaneously. See [reproducibility notes](../../docs/REPRODUCIBILITY.md).

[Back to the project](../../README.md)
