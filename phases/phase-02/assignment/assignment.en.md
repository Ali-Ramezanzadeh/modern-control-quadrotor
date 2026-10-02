# Phase 2 assignment — English translation

**Source:** [original Persian assignment](assignment.fa.pdf), 2 PDF pages. Modern Control, first semester of academic year 1403–1404; Dr. Atrianfar. The stated deadline is **1403/09/20** in the Solar Hijri calendar.

This is an English translation of the substantive historical course instructions. It describes required tasks rather than claiming every task has been verified in the submitted project.

## A. Introduction and B. Submission

The project applies Modern Control concepts in phases. The first two phases take place during the term, and the final phase emphasizes practical design and implementation. Submit code, simulations and a report in the provided format through the course platform.

The source repeats the requirements for a `Main.m` entry file that runs all outputs in order, white plot backgrounds, labeled axes with units and legends, MATLAB **Edit → Copy Figure** rather than screenshots, captions in the report, and one compressed group submission using `MC_Project1_GroupNo` as the printed naming pattern. The same bonus opportunities are listed: an original idea; Simscape and graphical/physical implementation; additional relevant paper sections; and an IEEE-style article for novel work.

## C. Phase 2 questions

1. **Stability review.** Examine system stability by eigenvalues and Lyapunov methods, and determine BIBO stability.

2. **Minimal realization.** Find the irreducible/minimal subsystem of the linear model.

3. **Specify performance and design feedback.** Define a meaningful performance criterion, such as settling time and overshoot, physical state constraints, actuator constraints, or robustness to disturbances and uncertainty. Select desired poles from that criterion, then design a state-feedback controller that stabilizes the plant and meets the defined objectives. Compare criteria and design results with the chosen paper where possible.

4. **Closed-loop simulations.** Simulate both the linear and nonlinear plants with the state-feedback controller designed in Question 3. Analyze controller performance.

5. **Nonlinear instability.** If the nonlinear closed-loop model is unstable, investigate the cause. The source suggests studying the **domain of attraction**.

6. **Reference tracking.** Define appropriate step and sinusoidal reference signals, using the selected paper if useful. Design static and dynamic precompensators for the linear model. Simulate and analyze each method's reference-tracking performance in the presence of a constant disturbance.

7. **Nonlinear reference tracking.** Apply the designed dynamic precompensator to the nonlinear model and examine its performance, provided the experiment does not lead to instability.

The assignment closes with a success wish signed by Shah-Rajabian.
