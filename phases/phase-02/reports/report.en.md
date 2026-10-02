# Phase 2 — English technical report companion

**Original title:** Controller design to achieve stability objectives and reference tracking for a quadrotor system. **Source:** [complete Persian report](report.fa.pdf), 27 PDF pages. See [team credits](../../../CREDITS.md).

This is a **condensed technical translation and explanation**, organized by the original chapters. Repeated general discussion is consolidated; original diagrams and plots remain in the [figure gallery](FIGURES.md). It is not a sentence-by-sentence translation of all 27 pages. Page references below count the cover and contents. Editorial notes distinguish reported claims from code details and independently checked model properties.

## Chapter 1 — Introduction

*Source: PDF page 5.*

This phase designs controllers for the quadrotor modeled in Phase 1. The report considers state feedback, static/dynamic precompensation and LQR, with linear/nonlinear simulations and disturbance scenarios. Its objectives are to improve stability and reference-following behavior and study the response under changing conditions.

## Chapter 2 — Stability and minimal realization

*Source: PDF page 7.*

The open-loop hover model has all eigenvalues at zero. The report observes unbounded outputs for bounded inputs and identifies a lack of BIBO stability. In this model, that conclusion follows from the nontrivial integrator chains.

The code creates a state-space realization, calls `minreal`, and extracts matrices with `ssdata`. The purpose is to remove uncontrollable or unobservable components. Independent checks of this twelve-state plant show full controllability and observability, so a minimal realization still has twelve states. Calling the function does not imply that a smaller physical model was obtained.

**Parameter note:** Phase 2 uses `Ixx=Iyy=0.0085532` and `Izz=0.01476`, which differ from the Phase 1 setup. The complete state/input conventions and parameter comparison are in [MODEL.md](../../../docs/MODEL.md).

## Chapter 3 — State-feedback controller

### 3.1 Performance criteria and pole selection

*Source: PDF pages 9–10.*

The report discusses overshoot and settling time as useful performance criteria. It names approximately **4.6% overshoot** and **8 seconds settling time** as its desired performance values, with faster real poles accompanying a dominant complex pair.

The code specifies:

```matlab
Poles = [-0.5+0.5j, -9.3, -0.5-0.5j, -9, ...
         -9.3, -9.3, -9.7, -9.7, -9.7, -9.7, -9.9, -9.9];
Kfeedback = place(A_eq,B_eq,Poles);
A_feedback = A_eq-B_eq*Kfeedback;
```

These are design choices and reported targets, not independently measured all-channel closed-loop metrics. The overshoot/settling-time behavior of the complete MIMO nonlinear plant cannot be inferred from the dominant pair alone.

### 3.2 Simulink closed-loop comparisons

*Source: PDF pages 10–15.*

The Simulink workflow applies feedback to the linear and nonlinear plants and combines control inputs with reference/disturbance signals. The report describes a constant disturbance generator with a four-element `[2,2,2,2]` setting and discusses additive and multiplicative disturbances as background models:

$$y_{dist}(t)=y(t)+d(t),\qquad y_{dist}(t)=y(t)(1+\alpha(t)).$$

It also describes a trajectory generator built from constant and varying signals. A general sinusoidal reference is written as `Aref*sin(omega*t+phi)+Cref`. These expressions explain the report's signal categories; they do not establish that every cited disturbance type is active in each saved branch.

**Terminology correction:** the original discussion sometimes equates linear outputs with stable behavior and nonlinear outputs with unstable behavior. Linearity and stability are distinct properties. The relevant experiment compares plants and controller configurations, not intrinsically stable versus unstable categories of signals.

### 3.3 Nonlinear attraction-region observations

*Source: PDF page 16.*

The report describes local closed-loop simulation observations: roll and pitch initial angles around `0.1 rad`, position excursions of roughly `1–2`, and different sensitivity for yaw and body rates. It suggests that larger position changes require greater control effort and recommends examining state trajectories to understand attraction boundaries.

These are qualitative observations from tested conditions. They are not a formal region-of-attraction certificate or a general claim that yaw can take arbitrary values. Initial-state units, simultaneous perturbations, control limits and the selected branch must be specified in any reproduction.

## Chapter 4 — Static and dynamic reference compensation

### 4.1 Design

*Source: PDF pages 18–19; implementation also appears in the Live Script.*

Static precompensation adjusts the input/reference mapping using the closed-loop DC gain. The script computes `G_0=-C_final/A_feedback*B_eq`, uses a pseudoinverse, and chooses columns corresponding to `[x,y,z,psi]`. It repeats the calculation for LQR feedback.

Dynamic precompensation adds four integral states for reference errors. The augmented system has sixteen states, and the code uses LQR to obtain its feedback gain. A second weighting choice, `Qk`, supplies an alternative tuning in the Live Script.

The main non-augmented LQR weights in the saved code are:

```matlab
Q1 = diag([4000,240,4000,250,2500,400,250,750,1000,60,60,10]);
R1 = 3*diag([1,1,5,5]);
```

The primary augmented weights are:

```matlab
Q = diag([1,1,1,1,1,1,1,1,1,1,1,1,500,4500,4000,400]);
R = diag([1,5,1,1]);
```

The static method depends on nominal steady-state mappings. Integral action is intended to reduce constant tracking offsets, but its performance still depends on closed-loop stability, tuning and constraints.

### 4.2 Reported comparisons

*Source: PDF pages 20–24.*

The pole-placement experiment shows appreciable delay and unsatisfactory `x`/`y` trajectory tracking in the report's selected scenario. Subsequent figures show the LQR cases with static and dynamic compensation.

![Original reported pole-placement tracking comparison](figures/p020-img02.jpg)

Original report, PDF page 20.

The narrative describes relatively small offsets in steady/slowly changing conditions and larger transient oscillations when reference or disturbance signals change quickly. It expresses tracking error as `e(t)=y_ref(t)-y(t)` and also uses an absolute error for comparison. It states that the dynamic cases can show initial oscillation before approaching equilibrium.

**Reading the comparison:** the source mixes the phrases “static/dynamic conditions” and “static/dynamic precompensator.” They should not be treated as identical concepts. The plotted experiment, active reference and controller branch need to be identified before making a method ranking. This repository retains the original plots without inventing a numeric comparison table.

## Chapter 5 — Conclusions

*Source: PDF page 26; references on page 27.*

The report concludes that state-feedback and LQR approaches can improve the chosen simulated responses, while hover linearization provides a useful design model. It recommends robust/adaptive methods as possible future work for disturbances and more difficult environments.

Those recommendations are not evidence of completed robust, adaptive or MPC implementations in this phase. The archived files demonstrate pole-placement/LQR design and reference-compensation work. Further numerical validation requires MATLAB/Simulink execution with recorded scenario settings.

[References](../../../docs/REFERENCES.md) · [Reproducibility and known issues](../../../docs/REPRODUCIBILITY.md) · [All original figures](FIGURES.md)
