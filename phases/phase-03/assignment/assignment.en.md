# Phase 3 assignment — English translation

**Source:** [original Persian assignment](assignment.fa.pdf), 4 PDF pages. Modern Control, first semester of academic year 1403–1404; Dr. Atrianfar. The stated deadline is **1403/10/21** in the Solar Hijri calendar.

This translates the substantive course instructions and equations. The original PDF retains layout and notation. Optional requirements below are separate from the submitted work's verified scope.

## A. Introduction and B. Submission

The final phase covers optimal control, observer design and control using estimated states. It then examines actuator saturation, noise and uncertainty, which were absent from nominal simulations. Bonus tasks introduce discretization for implementation and Kalman estimation.

Submit code, simulations and a report through the course platform. The repeated instructions require a `Main.m` entry file that displays outputs in order, white plot backgrounds, axis labels with units and legends, MATLAB **Edit → Copy Figure** rather than screenshots, figure captions, and one group archive using the printed pattern `MC_Project1_GroupNo`. Bonus credit may be available for a novel idea, Simscape/graphical/physical implementation and an IEEE-style write-up of novel results.

## C. Required questions

1. **LQR tracking.** To reduce control cost while maintaining a suitable tracking response, design an LQR state-feedback controller with dynamic precompensation. Explain the choice of weighting matrices `Q` and `R`, analyze the observed behavior and simulate a complete reference-trajectory scenario.

2. **Actuators and saturation.** Include the actuator model from the paper if provided. Otherwise, use a first-order transfer function with a time constant suitable for the plant. Apply physical actuator/control constraints to the feedback output using Saturation blocks. Evaluate the previous controller with these dynamics and constraints; retune the weighting matrices if necessary to obtain acceptable response within the control-input limits.

3. **Full and reduced observers.** Design both full-order and reduced-order observers. Plot true and estimated states and compare estimation quality in closed loop with the Question 1 controller using component-wise mean absolute error:

   $$MAE_{estimation}(T)=\frac{1}{T}\int_0^T|x(t)-\hat x(t)|dt$$

   $$MAE_{tracking}(T)=\frac{1}{T}\int_0^T|x(t)-x_d(t)|dt$$

   `T` is the simulation duration. Absolute value acts component by component; state-estimation MAE is therefore a vector. Simulink integrators can calculate these quantities.

4. **Noise.** Add suitably scaled white noise to the output signals with a **Band-Limited White Noise** block at the specified 100 Hz rate. Study the effect on estimation and tracking, and report both MAEs for each observer.

5. **Model uncertainty.** Change the plant state matrix to `1.2*A`, representing a 20% change. Use the previously designed controllers and observers without redesigning their gains for the perturbed plant. Report estimation and tracking MAE and suggest ways to improve robustness.

## D. Bonus tasks

### 1. Sensor update rate

Use a Rate Transition block to set the output/sensor update rate. If the sensor sampling rate is unknown and the simulation step is `0.01` seconds, the source gives `0.02` seconds as an example sensor period.

### 2. Discrete controller and observer

For implementation on a board, replace continuous controller/observer blocks with discrete equivalents. Convert the relevant subsystem to a Reference Model, save it as a separate model and use Model Discretizer with ZOH and `Ts=0.01` seconds (100 Hz) as the example setting. Faster plant dynamics may require a shorter sample time. Set a fixed-step solver in both models, simulate, and adjust gains if needed so that the discrete controller and observer can control the continuous plant.

This is a requested implementation exercise, not a claim that hardware testing occurred.

### 3. Hybrid Kalman estimation and LQG

Large Luenberger gains can speed error convergence but amplify noisy sensor data. The assignment introduces Kalman filtering as a compromise between convergence speed and estimation-error variance. It mentions continuous, discrete and hybrid linear filters, and EKF/UKF for nonlinear plants. It requests a hybrid filter with continuous prediction and correction when a new discrete measurement arrives.

The model is:

$$\dot x(t)=Ax(t)+Bu(t)+\Gamma w(t),\qquad y_k=Cx_k+Du_k+v_k$$

Process noise `w(t)` and measurement noise `v_k` are independent, zero-mean Gaussian noises, with process intensity `Q` and measurement covariance `R`. The stated correlations are `E[w(t)w(tau)^T]=Q*delta(t-tau)`, `E[v_k*v_j^T]=R*delta_kj` and zero cross-correlation.

The given algorithm is:

1. **Initialize:** `xhat(0)=E[x(0)]` and `P(0)=E[(x(0)-xhat(0))*(x(0)-xhat(0))']`.
2. **Propagate covariance and estimate:**

   $$\dot P=AP+PA^T+\Gamma Q\Gamma^T,\qquad \dot{\hat x}=A\hat x+Bu$$

   Integrate to obtain the predicted covariance and estimate. Force covariance symmetry using `Pminus=(Pminus+Pminus')/2`.
3. **Calculate gain:**

   $$K_k=P_k^-C_k^T(C_kP_k^-C_k^T+R)^{-1}$$
4. **Correct when a measurement arrives:** the printed source uses

   $$\hat x_k^+=\hat x_k^-+K_k[y_k-C_kx_k^-]$$

   and the Joseph covariance update

   $$P_k^+=(I-K_kC_k)P_k^-(I-K_kC_k)^T+K_kRK_k^T.$$

**Notation note:** the source's innovation term prints `x_k^-` without a hat and omits the known `D*u` term. A consistent implementation uses the predicted estimate and includes known feedthrough when `D` is nonzero. The quadrotor scripts use `D=0`.

The source suggests a block implementation or a MATLAB Function, `persistent` variables for initialization and Runge–Kutta integration for prediction. Trigger measurement correction with a counter condition `mod(counter,SR)==0`, where `SR` is the measurement period divided by the simulation period.

If the noise-input distribution matrix is unknown, use `Gamma=I`. Generate process and measurement noise with separate Band-Limited White Noise blocks, using the simulation and sensor sample periods respectively and different seeds. Set their powers according to `Q` and `R`. The source notes that these parameters are usually estimated/tuned in real systems, whereas in the exercise the noise is generated deliberately. It states an effective hybrid measurement setting `SR*R`; this is part of the assignment's noise-generation convention and should be checked against the chosen block scaling in an implementation.

A larger `R` gives less confidence in measurements and more weight to the model prediction. A larger `Q` expresses greater model/process uncertainty and gives measurements more influence. Choose an initial covariance as a multiple of the identity; the source cautions against an excessively small initial covariance.

Finally, close the loop with Kalman state estimates and the LQR controller from Question 1 (an LQG configuration), plot true/estimated states and trajectory tracking, and compare MAEs with the Luenberger observer.

The assignment closes with a success wish signed by Shah-Rajabian. The presence of this optional task in the assignment does not establish its completion in the final submitted model.
