# Quadrotor model and controller structure

This document describes equations and values found in the supplied Live Scripts. Phase-specific parameter sets are kept separate.

## States, inputs and outputs

$$X=[x,\dot x,y,\dot y,z,\dot z,\phi,\theta,\psi,p,q,r]^T$$

Positions are in meters; translational velocities in meters per second; Euler angles in radians; and body angular rates in radians per second. Here `p, q, r` are body-axis angular rates, which generally differ from the Euler angle derivatives.

$$U=[U_1,U_2,U_3,U_4]^T$$

`U1` is total thrust in newtons. `U2`, `U3` and `U4` are roll, pitch and yaw torques in newton-meters. Rotor-speed allocation is discussed in the reports and modeled in branches of the final Simulink file; the symbolic plant equations take thrust and torques as their inputs.

The analytical output matrix `C` selects `[x,y,z,psi]`. `C_simulink = eye(12)` exposes all states for display. `C_final` selects `[x,y,z,phi,theta,psi]`, with rows `[1,2,3,6]` used for the four reference channels and observers. Direct feedthrough matrices are zero.

## Parameters in the code

| Parameter | Phase 1 | Phases 2 and 3 |
| --- | ---: | ---: |
| Mass `m` (kg) | 1 | 1 |
| Gravity `g` (m/s²) | 9.81 | 9.81 |
| `Ixx` (kg·m²) | 0.0035 | 0.0085532 |
| `Iyy` (kg·m²) | 0.0035 | 0.0085532 |
| `Izz` (kg·m²) | 0.005 | 0.01476 |
| Gyroscopic constant `J` (kg·m²) | 0.0001 | Not used in the symbolic equations |
| Aggregate rotor imbalance `Omega` | 0 | 0 |
| Hover rotor speed `Omega_h` (rad/s) | 500, declared | Not declared |
| Thrust coefficient `b` | Not declared in the Live Script | 7.66e-5 |
| Drag coefficient `d` | Not declared in the Live Script | 5.63e-6 |
| Arm length `l` (m) | Not declared in the Live Script | 0.22 |

The Phase 1 report separately gives `b=9.80e-6`, `d=1.60e-7` and `l=22.5` with a meter unit. That last length/unit pairing needs checking; this documentation does not silently reinterpret it as centimeters. The parameter sets above follow the native code rather than blending the report tables and later scripts.

## Nonlinear equations

The scripts implement the following translational accelerations:

$$\ddot x=\frac{U_1}{m}(\cos\phi\sin\theta\cos\psi+\sin\phi\sin\psi)$$
$$\ddot y=\frac{U_1}{m}(\cos\phi\sin\theta\sin\psi-\sin\phi\cos\psi)$$
$$\ddot z=-g+\frac{U_1}{m}\cos\phi\cos\theta$$

The rotational dynamics in Phases 2 and 3 are:

$$\dot p=\frac{I_{yy}-I_{zz}}{I_{xx}}qr+\frac{U_2}{I_{xx}},\qquad
\dot q=\frac{I_{zz}-I_{xx}}{I_{yy}}pr+\frac{U_3}{I_{yy}},\qquad
\dot r=\frac{I_{xx}-I_{yy}}{I_{zz}}pq+\frac{U_4}{I_{zz}}$$

Phase 1 includes gyroscopic terms `+(J/Ixx)*theta_dot*Omega` and `-(J/Iyy)*phi_dot*Omega` in the first two equations. Since `Omega=0`, these terms vanish in the supplied setup.

The Euler-angle kinematics are:

$$\dot\phi=p+q\sin\phi\tan\theta+r\cos\phi\tan\theta$$
$$\dot\theta=q\cos\phi-r\sin\phi$$
$$\dot\psi=q\sin\phi\sec\theta+r\cos\phi\sec\theta$$

This representation is singular when `cos(theta)=0`; the hover linearization is a local model.

## Hover linearization

The equilibrium is `Xe=zeros(12,1)`, `Ue=[m*g,0,0,0]'`. The scripts use symbolic Jacobians to obtain `A_eq = df/dX` and `B_eq = df/dU` at that point.

With `delta X=X-Xe` and `delta U=U-Ue`, the nonzero dynamics are:

```text
delta x_dot     = delta vx
delta vx_dot    = g * delta theta
delta y_dot     = delta vy
delta vy_dot    = -g * delta phi
delta z_dot     = delta vz
delta vz_dot    = delta U1 / m
delta phi_dot   = delta p
delta theta_dot = delta q
delta psi_dot   = delta r
delta p_dot     = delta U2 / Ixx
delta q_dot     = delta U3 / Iyy
delta r_dot     = delta U4 / Izz
```

For outputs `[x,y,z,psi]` and inputs `[U1,U2,U3,U4]`, the reconstructed transfer matrix is:

$$G(s)=\begin{bmatrix}
0&0&g/(I_{yy}s^4)&0\\
0&-g/(I_{xx}s^4)&0&0\\
1/(ms^2)&0&0&0\\
0&0&0&1/(I_{zz}s^2)
\end{bmatrix}$$

There are integrator chains of lengths 4, 4, 2 and 2. All eigenvalues are zero, and the nontrivial Jordan blocks yield growing free responses. The open-loop model is internally unstable and not BIBO stable. This conclusion uses the actual chain structure, not merely the fact that some poles are at zero.

Exact rational checks give controllability and observability ranks of 12 for both supplied inertia sets; this hover realization is minimal, stabilizable and detectable. See [verification](VERIFICATION.md) for the checks and [reproducibility](REPRODUCIBILITY.md) for inconsistent historical console messages.

## Feedback and reference tracking

Pole placement uses `delta U=-Kfeedback*delta X` and closed-loop matrix `A_eq-B_eq*Kfeedback`. LQR instead minimizes a continuous-time quadratic state/input cost using phase-specific `Q` and `R`.

Static reference compensation uses the closed-loop DC gain and its pseudoinverse. Integral reference tracking introduces four accumulated output-error states `eta`, where `eta_dot=reference-Cref*delta X`:

$$A_i=\begin{bmatrix}A&0\\-C_{ref}&0\end{bmatrix},\qquad
B_i=\begin{bmatrix}B\\0\end{bmatrix}$$

The augmented gain has dimensions 4×16 and is partitioned into state and integral components. The reconstructed augmented system is exactly controllable with rank 16.

## Observers and evaluation

Phase 3 defines a full-order observer with `L=place(A_eq',C_eq',observer_poles)'`. Its nominal estimation-error matrix is `A_eq-L*C_eq`.

The reduced-order design uses four measured outputs and eight dynamic observer states. It solves a Sylvester equation for `T_O` and reconstructs the complete state through `[C;T_O]` and matrices `Q_1` and `Q_2`. Invertibility and numerical conditioning of that transformation should be checked for the specific saved tuning.

The reports use component-wise mean absolute error:

$$MAE_{est,j}(T)=\frac{1}{T}\int_0^T|X_j(t)-\hat X_j(t)|\,dt$$

$$MAE_{track,j}(T)=\frac{1}{T}\int_0^T|X_j(t)-X_{d,j}(t)|\,dt$$

Noise and `Aplant=1.2*A_nominal` are evaluation scenarios. They do not by themselves establish a general robustness guarantee.
