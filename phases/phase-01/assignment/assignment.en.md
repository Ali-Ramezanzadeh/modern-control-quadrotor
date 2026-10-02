# Phase 1 assignment — English translation

**Source:** [original Persian assignment](assignment.fa.pdf), 7 PDF pages. Modern Control, first semester of academic year 1403–1404; Dr. Atrianfar. The stated deadline is **1403/08/10** in the Solar Hijri calendar.

This document translates the substantive course instructions. Illustrations and the original layout remain in the source PDF. These are historical assignment requirements, not instructions to publish or modify this repository.

## A. Introduction

The project aims to apply Modern Control course concepts through several phases completed throughout the semester. The first two phases are submitted during the term. The third phase emphasizes practical controller design and implementation, followed by a final report and presentation at the end of the semester.

Each group first selects a physical system from reputable research papers, then answers the questions for each phase and prepares a complete report. More than one paper may be needed, but one principal paper should initially provide the nonlinear model and its numerical parameters.

## B. Submission instructions

At each deadline, submit the code and simulation files together with a report in the supplied format through the course platform.

1. Include a MATLAB entry file named `Main.m` whose execution displays all required outputs in order and runs the simulations and plots.
2. Use white plot backgrounds. Label axes, including variable units, and identify signals in legends.
3. Insert figures using MATLAB's **Edit → Copy Figure** command instead of taking screenshots. Give each figure an appropriate caption in the report.
4. Complete the corresponding report section after each phase. One group member submits a compressed archive for the whole group, using the naming pattern `MC_Project1_GroupNo`.

The following may receive bonus credit: an original idea in the selected topic; Simscape modeling and comparison with the state-space results; graphical simulation in Simulink, Virtual Reality, ROS or Webots, or a physical implementation; additional relevant work from the chosen paper; and an IEEE conference-style write-up when the project has a novel idea.

## C. Suggested systems

The source illustrates eleven possible systems:

| System | Description / example control objective |
| --- | --- |
| Ball and plate | Keep a ball at a desired position on a plate with adjustable inclination; mechanisms may include a Stewart/hexapod platform and camera-based ball sensing. |
| Automobile suspension | Reduce road-induced vibration of the sprung mass while maintaining tire contact; ride comfort and handling involve competing requirements. |
| Quadrotor | Use four rotors to produce thrust and torques; possible tasks include attitude control, takeoff/landing and trajectory tracking. |
| Two-degree-of-freedom helicopter | A laboratory plant with two motors controlling pitch and yaw, such as a Quanser platform. |
| Satellite | Attitude control, orbit changes or trajectory control. |
| Cubli | A cube with three reaction wheels and inertial sensors; balance on an edge or corner in the presence of disturbances. |
| Wheeled mobile robot | Control wheel speed/torque to track a path, addressing slip, uncertainty and obstacle avoidance. |
| Parallel manipulator | Accurate, fast motion with coupled joints; the source mentions a drawing robot as an example. |
| Quadruped robot | Balance, walking, running, jumping and path tracking on difficult terrain; the source illustrates MIT Cheetah 3. |
| Gymnastic robot | Model coordinated link motion to reproduce a desired gymnastics maneuver, on a bar, rings or the ground. |
| Soft robot | Control flexible links/actuators, often pneumatically driven, for tasks such as delicate interaction or rehabilitation. |

The submitted team project chooses the quadrotor.

## D. Phase 1 questions

1. **Introduce the system and select a reference paper.** Explain the chosen physical system, how it works, its sensors and actuators, using papers and other research sources. Select a paper for the model and obtain the instructor's approval before proceeding. The paper must contain the nonlinear equations and all numerical parameters needed for simulation. Prefer a state-space-based controller so that results can be compared. Suggested search terms include the system name with **State Feedback**, **LQR** and **Optimal Control**.

2. **Model and linearize.** Obtain the nonlinear model from the paper and introduce its parameters. Linearize around a suitable equilibrium using MATLAB and obtain a state-space model. Determine the equilibrium from physical reasoning, solving the equations with `fsolve`, or the selected paper.

3. **Jordan form.** Obtain the block-diagonal Jordan form for the model.

4. **Transfer matrix.** Obtain the system's transfer function or transfer-function matrix and plot its pole-zero map.

5. **Stability.** Examine internal stability using eigenvalues and a Lyapunov approach, stating the type of stability. Is the system BIBO stable?

6. **Time response.** Find the state-transition matrix. In MATLAB, obtain state and output responses to an arbitrary initial condition and a unit-step input.

7. **Initial-condition selection.** Choose initial conditions so that a particular frequency component in the output is not excited.

8. **Linear/nonlinear simulation.** Implement both the nonlinear paper model and the linear state-space model in Simulink. Compare their responses for (a) zero input and a physically appropriate nonzero initial condition, and (b) a step input of appropriate magnitude and zero initial condition.

   If the equilibrium state `Xe` or input `Ue` is nonzero, the linear block uses deviation variables: `delta U=U-Ue` and `delta Y=Y-Ye`. Add `Ye` back to the linear block's output when comparing physical outputs. Plot both results together with clear legends using `subplot` or `hold`, and explain the comparison. If the open-loop system is unstable, simulation is not required for this question, but both Simulink implementations are still required.

9. **Validity of the linear approximation.** With suitable initial conditions, vary the input deviation to move away from equilibrium and compare the linear/nonlinear responses. The source suggests `delta U=0.01`, `0.05`, `0.1`, and further values as examples. Identify when the responses differ substantially. Apply the appropriate physical input `U=Ue+delta U` to the nonlinear model.

10. **Controllability and observability.** Examine the linearized model. If it is not minimal, identify controllable and observable poles. Is it stabilizable and detectable?

11. **Kalman decomposition.** Obtain the controllable/observable, controllable/unobservable, uncontrollable/observable and uncontrollable/unobservable subsystems.

The assignment closes with a success wish signed by Shah-Rajabian. This is reproduced as course-source context.
