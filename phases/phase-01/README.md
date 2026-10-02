# Phase 1 — Modeling and system analysis

This phase develops a nonlinear quadrotor model and a 12-state linearization around hover. It studies Jordan form, the MIMO transfer matrix, open-loop stability, time response, controllability and observability, and compares linear/nonlinear Simulink implementations.

## Files

| File | Purpose |
| --- | --- |
| [Phase1.mlx](matlab/Phase1.mlx) | Original MATLAB Live Script |
| [Phase1_simulink.slx](matlab/Phase1_simulink.slx) | Original Simulink model |
| [source_export.m](matlab/source_export.m) | Readable export of original code cells; untested |
| [report.fa.pdf](reports/report.fa.pdf) | Complete original Persian report, 46 pages |
| [report.en.md](reports/report.en.md) | Condensed English technical companion |
| [FIGURES.md](reports/FIGURES.md) | 36 extracted report images with page references |
| [assignment.fa.pdf](assignment/assignment.fa.pdf) | Original course assignment, 7 pages |
| [assignment.en.md](assignment/assignment.en.md) | English translation of the substantive instructions |

## Main findings

The hover model has four integrator chains and all poles at zero. Independent algebra checks confirm controllability and observability ranks of 12. Open-loop response growth motivates feedback control in the later phases.

The original Phase 1 decomposition code and stabilizability/detectability messages need review; see [known issues](../../docs/REPRODUCIBILITY.md). Those issues are preserved in both the original Live Script and the readable export.

Open `Phase1.mlx` in MATLAB, run and inspect sections in order, then open `Phase1_simulink.slx`. The saved model stop time is 1 second. Simulations have not been rerun for this repository.

[Back to the project](../../README.md)
