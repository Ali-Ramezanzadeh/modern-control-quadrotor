# Phase 1 — English technical report companion

**Original title:** Modeling, stability analysis, controllability and observability of a quadrotor system. **Source:** [complete Persian report](report.fa.pdf), 46 PDF pages. See [team credits](../../../CREDITS.md).

This is a **condensed technical translation and explanation**, organized by the original chapters. Repeated general discussion and screenshot-based matrices are consolidated. It is not a sentence-by-sentence translation of all 46 pages. All original report figures are available in the [figure gallery](FIGURES.md). PDF page references count the cover and contents. Editorial notes distinguish source claims from independent checks and saved-code details.

## Chapter 1 — Introduction and system overview

*Source: PDF pages 8–11.*

The project studies a quadrotor using Modern Control concepts. The first phase develops a mathematical model and investigates stability, controllability and observability. The attached MATLAB code is organized by assignment question.

A quadrotor combines a lightweight, strong frame with four motor/propeller assemblies arranged around its center. The report describes symmetric construction and opposite rotor directions as mechanisms for distributing forces and balancing reaction torques. Changing motor speeds creates thrust and rotational motion.

The system overview discusses accelerometers for linear acceleration, gyroscopes for angular rates, magnetometers for heading, pressure/altitude sensors for vertical position and GPS for geographic position. Electronic motor controls act on commands from the flight controller. These are background descriptions of quadrotor components, not evidence of an instrumented physical vehicle built for this project.

The report introduces PID as a controller with proportional, integral and derivative actions, and LQR as a state-space approach that balances state deviation and control effort using weighting matrices. The cited literature motivates the choice of a quadrotor. Actual state-feedback design belongs to the later phases.

## Chapter 2 — Modeling and simulation

### 2.1 Nonlinear model and hover equilibrium

*Source: PDF pages 13–17.*

The model uses twelve states: three positions and translational velocities, three Euler angles, and three body angular rates. The four inputs are total thrust and roll, pitch and yaw torques.

Translational acceleration follows the orientation of the total thrust vector and gravity. Rotational acceleration depends on inertia, coupled body rates and applied torque. Euler-angle derivatives are calculated from the body rates. The complete equations and phase-specific parameters are written in [the English model guide](../../../docs/MODEL.md).

The selected hover equilibrium sets all states to zero, total thrust to `m*g`, and torques to zero. Symbolic Jacobians with respect to the state and input vectors produce the linearized matrices `A_eq` and `B_eq`.

**Code clarification:** the analytical `C` measures only `[x,y,z,psi]`. The report's statement that all states are measured does not describe this matrix. The separate `C_simulink=eye(12)` is used to display every state.

The report parameter table gives `m=1 kg`, `g=9.81 m/s²`, `Ixx=Iyy=0.0035 kg·m²`, `Izz=0.005 kg·m²`, `J=0.0001`, and a hover rotor speed of `500 rad/s`. Additional rotor coefficients and the arm-length table entry are preserved in the model guide with a note about their difference from the code and later phases.

### 2.2 Jordan form

*Source: PDF page 18.*

The script computes a Jordan decomposition of `A_eq`. The Jordan form expresses repeated eigenvalues through blocks and makes the internal integrator structure visible.

**Technical clarification:** the columns returned in the transformation matrix can include generalized eigenvectors; they need not all be ordinary eigenvectors. For the reconstructed hover model, the chains have lengths 4, 4, 2 and 2, each at eigenvalue zero. Jordan form alone does not determine controllability or observability.

### 2.3 Transfer matrix and pole-zero analysis

*Source: PDF pages 19–21.*

The report forms the state-space model, evaluates `C*(s*I-A_eq)^(-1)*B_eq`, converts it to transfer-function form, and draws the pole-zero map. It reports all poles at zero and no finite transmission zeros.

The transfer matrix routes pitch torque to horizontal `x` position, roll torque to horizontal `y` position with a negative sign, total thrust to `z`, and yaw torque to `psi`. The first two paths have four integrators; the last two have two.

**Interpretation:** absence of finite zeros is not a stability guarantee. Open-loop stability depends on the pole/Jordan structure, which here permits growing responses.

### 2.4 Simulink implementation and linearization validity

*Source: PDF pages 22–27.*

The report implements the linear system in two ways: a State-Space block and a network of matrix gains/integrators. A separate nonlinear plant implements the physical equations. It also describes angle wrapping and compares output trajectories under different inputs and initial conditions.

![Original Phase 1 Simulink diagram](figures/p022-img01.png)

Original report, PDF page 22.

The comparisons include zero input with nonzero initial conditions, a `0.2` step with zero initial conditions, and separate perturbations `delta U1=5`, `delta U2=0.05`, `delta U3=0.05` and `delta U4=0.05`. The report presents output plots rather than a numeric validity-region table.

The narrative notes that with zero attitude and pure vertical-thrust variation, the vertical acceleration expression matches the hover linear form. This agreement for that special condition does not make the complete nonlinear model globally linear. As attitudes or other inputs move away from hover, the linear/nonlinear responses can diverge.

**Source difference:** the narrative describes `Condition_1` as a vector of ones, while the saved Live Script uses `0.2*ones(12,1)`. This must be resolved when reproducing the displayed plots.

## Chapter 3 — Stability, initial conditions and response

### 3.1 Stability

*Source: PDF pages 29–30.*

The script calculates eigenvalues and checks for negative real parts. It attempts a positive-definite Lyapunov solution only when this strict stability condition holds. The open-loop hover model fails that check because its eigenvalues are zero.

**Technical interpretation:** zero eigenvalues alone do not always imply internal instability. Here, however, nontrivial integrator chains produce polynomial growth, so the open-loop realization is internally unstable and not BIBO stable. A unit thrust or torque input can produce unbounded position or yaw output.

### 3.2 State transition and forced response

*Source: PDF pages 31–34.*

The report obtains the Laplace-domain transition matrix `(s*I-A_eq)^(-1)` and its inverse Laplace transform. It computes response from initial conditions plus unit steps:

$$X(s)=(sI-A)^{-1}X_0+(sI-A)^{-1}B\,U(s),\qquad Y(s)=CX(s).$$

The source shows symbolic matrices, output expressions and a MATLAB plot. The active Live Script defines a specific twelve-element initial-condition vector, rather than the all-ones vector mentioned in part of the narrative. See the original code export for its values.

### 3.3 Initial conditions and frequency excitation

*Source: PDF page 35.*

The report observes that the model has only the zero eigenvalue and selects a zero initial condition as a way to remove the free response.

**Scope note:** zero initial condition eliminates the initial-condition contribution. It does not eliminate response driven by a nonzero external input. With repeated zero eigenvalues, interpreting the dynamics only as ordinary sinusoidal frequency modes is also incomplete.

## Chapter 4 — Controllability, observability and decomposition

*Source: PDF pages 37–43.*

The code constructs controllability and observability matrices and compares their ranks with the twelve-state order. Both ranks are 12 for the supplied hover matrices. This means the whole state is reachable through the four control inputs and reconstructible from `[x,y,z,psi]` in the nominal linear model.

The next section attempts a Kalman decomposition and minimal realization. Its purpose is to separate controllable/observable and unreachable/unobservable components. For this fully controllable and observable model, all twelve states belong to the controllable/observable subsystem.

**Editorial correction:** the source's stabilizability/detectability tests use scalar Boolean indexing and give misleading negative messages. The nominal model is stabilizable and detectable. Phase 1 also misinterprets the `ctrbf` iteration vector as separate controllability/observability dimensions. Those partitions and their displayed reduced model should not be taken as a validated order reduction. [The reproducibility guide](../../../docs/REPRODUCIBILITY.md) explains the code issues without changing the original files.

## Chapter 5 — Conclusion and suggestions

*Source: PDF page 45; references on page 46.*

The report concludes that mathematical modeling and modern control analysis provide a basis for improving quadrotor behavior. It recommends further investigation of optimal and robust methods.

**Scope clarification:** the original conclusion also refers broadly to adaptive/robust control and Kalman estimation. Those remarks go beyond the demonstrated Phase 1 code, which focuses on modeling and open-loop analysis. They are not presented as implemented Phase 1 features in the repository overview.

The project references are collected in [REFERENCES.md](../../../docs/REFERENCES.md). Historical figures and all original pages are retained for comparison.
