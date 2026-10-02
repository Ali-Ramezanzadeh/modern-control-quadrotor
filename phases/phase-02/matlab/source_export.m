% Code/prose export of the original Main.mlx.
% Original code-cell text is unchanged. This export has not been run in MATLAB.
% See ../../../docs/REPRODUCIBILITY.md before running the historical scripts.

%% Define symbolic variables
syms phi theta psi phi_dot theta_dot psi_dot x_dot y_dot z_dot x y z real;
syms p q r p_dot q_dot r_dot real;
syms U1 U2 U3 U4 real;

% Define numerical values for parameters
m = 1;  % mass in kg (example value)
g = 9.81; % gravity in m/s^2
Ixx = 8.5532e-3;  % moment of inertia around x-axis (example value)
Iyy = 8.5532e-3;  % moment of inertia around y-axis (example value)
Izz = 1.476e-2;  % moment of inertia around z-axis (example value)
b = 7.66e-5; 
d = 5.63e-6;
l = 0.22;
Omega = 0;


% Define control inputs directly as U1, U2, U3, U4
% U1 = Total thrust
% U2 = Roll torque
% U3 = Pitch torque
% U4 = Yaw torque

% Define relationships between control inputs and motor speeds
% U1_eq = b * (Omega1^2 + Omega2^2 + Omega3^2 + Omega4^2);
% U2_eq = b * (-Omega2^2 + Omega4^2);
% U3_eq = b * (Omega1^2 + Omega3^2);
% U4_eq = d * (-Omega1^2 + Omega2^2 - Omega3^2 + Omega4^2);



% Define state vector X (including positions x, y, z and velocities xdot, ydot, zdot)
X = [x , x_dot , y , y_dot , z , z_dot , phi ,theta , psi , p , q ,r];
disp("State vector:")
disp(transpose(X))

U =[U1 , U2 , U3 , U4];
disp("Input vector")
disp(transpose(U))


% For translational motion
zdd = -g + (cos(phi)*cos(theta)*(1/m)*U1);  % Zdotdot = zdd
xdd = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*(1/m)*U1;  % xdotdot = xdd
ydd = (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*(1/m)*U1;  % ydotdot = ydd

% Define equations of motion (non-linear) using U1, U2, U3, U4
p_dot = q*r*(Iyy - Izz)/Ixx + (1/Ixx)*U2;  % phidotdot = phidd
q_dot = p*r*(Izz - Ixx)/Iyy + (1/Iyy)*U3;    % thetadotdot = thetadd
r_dot = p*q*(Ixx - Iyy)/Izz + (1/Izz)*U4;      % psidotdot = psidd

phi_dot = p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);
theta_dot = q*cos(phi)-r*sin(phi);
psi_dot = q*sin(phi)*sec(theta)+r*cos(phi)*sec(theta);
% State derivatives (use new variables for second derivatives)
f = [x_dot,xdd,y_dot,ydd,z_dot,zdd,phi_dot,theta_dot,psi_dot,p_dot,q_dot,r_dot];
disp("State equations:")
disp(transpose(f))
% Substitute the equilibrium points:
% At equilibrium: phi = theta = psi = phidot = thetadot = psidot = 0, and U1 = m*g (hover condition)

equilibrium_values = [0, 0, 0, 0, 0, 0, 0, 0, 0 , 0 , 0 , 0, m*g, 0, 0, 0]; % hover condition

% Linearize the system around the hover condition (equilibrium points)
A = jacobian(f, X);  % Derivative of f with respect to state variables X
A_eq = subs(A, [X,U] , equilibrium_values ); % Substitute equilibrium


B = jacobian(f, U);  % Derivative of f with respect to input variables U
B_eq = subs(B, [X, U], equilibrium_values); % Substitute equilibrium
B_eq = double(B_eq);

% Define output matrix (assuming full-state feedback)
C = [1 0 0 0 0 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 1 0 0 0];  % Output is the full state vector

C_simulink = eye(12);
D = zeros(4,4);
D_Simulink = zeros(12,4);

% Display the matrices
disp('Linearized State Matrix A around equilibrium:');
disp(A_eq);
A_eq = double(A_eq);
disp('Linearized Input Matrix B around equilibrium:');
disp(B_eq);

disp('Output Matrix C:');
disp(C);



%% Q3) Jordan Block Diagonal Form
disp("Block jordan of matrix A is :")
[V,J] = jordan(A_eq);
disp(J)

%% Q4) Transfer Function
syms s t 
phis = inv(s*eye(12)-A_eq); 
TF = C*phis*B_eq %#ok

sys=ss(A_eq,B_eq,C,D);
% Display the transfer function matrix
Tf=tf(sys);
figure
pzmap(sys) % all poles on zero

% Get the poles of the MIMO system
poles = pole(sys);

% Display the poles
disp('Poles of the system:');
disp(poles);

% Compute the transmission zeros
transmission_zeros = tzero(sys);

% Display the transmission zeros
disp('Transmission Zeros of the MIMO system:');
disp(transmission_zeros);
disp('This sys doesnt have any Zeros')

%%   Q5)  Internal stability

% Check eigenvalues of A
eigenvalues = eig(A_eq);
disp('Eigenvalues of A:');
disp(eigenvalues);

% Ensure the A matrix is stable
if any(real(eigenvalues) >= 0)
    disp('The system is not stable. Lyapunov stability check is not applicable.');
else
    % Define the Lyapunov equation: A'P + PA = -Q
    Q = eye(size(A_eq)); % Define a positive definite matrix Q
    try
        P = lyap(A_eq', Q); % Solve the Lyapunov equation
        disp('Lyapunov matrix P:');
        disp(P);

        % Check if P is positive definite
        if all(eig(P) > 0)
            disp('The Lyapunov matrix P is positive definite, confirming stability.');
        else
            disp('The Lyapunov matrix P is not positive definite.');
        end
    catch ME
        disp('Error solving Lyapunov equation:');
        disp(ME.message);
    end
end

%% Q6) State transition matrix

phi_L = inv(s*eye(length(A_eq))-A_eq);
disp(phi_L)
phi_t = ilaplace(phi_L);
disp(phi_t)
x0 = transpose([0.1 0.1 3 1 4 0.34 1 0.01 1 0.1 3 1]) ;
Uin_s = transpose([1/s ,1/s,1/s,1/s]);
X = phi_L*x0 + phi_L*B_eq*Uin_s;
Y = C*X ;
y_t = ilaplace(Y)
T = 0:0.01:1;
y_numeric = double(subs(y_t, t, T));

plot(T, y_numeric(1, :), 'DisplayName', 'x', 'LineWidth', 1.5);
hold on;
plot(T, y_numeric(2, :), 'DisplayName', 'y', 'LineWidth', 1.5);
plot(T, y_numeric(3, :), 'DisplayName', 'z', 'LineWidth', 1.5);
plot(T, y_numeric(4, :), 'DisplayName', 'psi', 'LineWidth', 1.5);
hold off;

xlabel('Time (t)');
ylabel('Output y(t)');
title('Output Response y(t) - States x, y, z, and psi');
legend('show');  
grid on;

%% Q8)Nonlinear simulink
Condition_1 = transpose([0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2]);
Condition_0 = transpose([0 0 0 0 0 0 0 0 0 0 0 0]);

x_0 = phi_t*Condition_1;
yout = C*x_0

%% Q9) Simulink
%% Q10) Controllability and Observability Matrix
n = size(A_eq, 1); % Number of state


Ctrb = ctrb(A_eq, B_eq);
Obsv = obsv(A_eq, C);


rank_Ctrb = rank(Ctrb);

rank_Obsv = rank(Obsv);


disp('Rank of Controllability Matrix:');
disp(rank_Ctrb);

disp('Rank of Observability Matrix:');
disp(rank_Obsv);

if rank_Ctrb == n
    disp('The system is controllable.');
else
    disp('The system is not controllable.');
end

if rank_Obsv == n
    disp('The system is observable.');
else
    disp('The system is not observable.');
end




% Compute eigenvalues and eigenvectors
[eigVec_C, eigVal_C] = eig(A_eq);
control_poles = diag(eigVal_C(rank(Ctrb) == n));

[eigVec_O, eigVal_O] = eig(A_eq');
observable_poles = diag(eigVal_O(rank(Obsv) == n));

% Display Results
disp('Controllable Poles:');
disp(control_poles);

disp('Observable Poles:');
disp(observable_poles);





eigenvalues = eig(A_eq);

% Display eigenvalues
disp('Eigenvalues of A:');
disp(eigenvalues);

% Check stabilizability
if all(real(eigenvalues(rank(Ctrb) == n)) < 0)
    disp('The system is stabilizable.');
else
    disp('The system is not stabilizable.');
end

% Check detectability
if all(real(eigenvalues(rank(Obsv) == n)) < 0)
    disp('The system is detectable.');
else
    disp('The system is not detectable.');
end


%% Q11) Kalman Decomposition
%% Kalman Decomposition

% Controllability 
ControllabilityMatrix = ctrb(A_eq, B_eq);
rank_controllability = rank(ControllabilityMatrix);
disp('Rank of Controllability Matrix:');
disp(rank_controllability);

%  Observability 
ObservabilityMatrix = obsv(A_eq, C);
rank_observability = rank(ObservabilityMatrix);
disp('Rank of Observability Matrix:');
disp(rank_observability);

% T Matrix
[Ac, Bc, Cc, T_c, k] = ctrbf(A_eq, B_eq, C);
disp('Transformation matrix for controllability decomposition (T_c):');
disp(T_c);

% DECOMPOSITION
A_cc = Ac(1:rank_controllability, 1:rank_controllability); % Controllable part of A
A_uu = Ac(rank_controllability+1:end, rank_controllability+1:end); % Uncontrollable part of A

B_cc = Bc(1:rank_controllability, :); % Input affecting controllable subspace
B_uc = Bc(rank_controllability+1:end, :); % Input affecting uncontrollable subspace

C_cc = Cc(:, 1:rank_controllability); % Output from controllable subspace
C_uc = Cc(:, rank_controllability+1:end); % Output from uncontrollable subspace

disp('Controllable A_cc:');
disp(A_cc);

disp('Uncontrollable A_uu:');
disp(A_uu);

disp('Controllable Input B_cc:');
disp(B_cc);

disp('Uncontrollable Input B_uc:');
disp(B_uc);

disp('Observable Output C_cc:');
disp(C_cc);

disp('Unobservable Output C_uc:');
disp(C_uc);

%% Phase 2 
% Create state-space system from linearized matrices
sys = ss(double(A_eq), double(B_eq), C, D);


% irreducible (minimal) subsystem
sys_reduced = minreal(sys);

disp('Reduced (Minimal) System:');
disp(sys_reduced);

% reduced matrices A, B, C, and D
[A_reduced, B_reduced, C_reduced, D_reduced] = ssdata(sys_reduced);

% Display
disp('Reduced State Matrix A:');
disp(A_reduced);
disp('Reduced Input Matrix B:');
disp(B_reduced);
disp('Reduced Output Matrix C:');
disp(C_reduced);
disp('Reduced Direct Transmission Matrix D:');
disp(D_reduced);

%% State Feedback
%Poles = [-0.5 + 1i*0.5,-2,-2.1,-2.1,-0.5-1i*0.5,-2.5,-2.5,-2.4,-2.3,-2.3,-2.03,-2.03];
Poles = [-0.5+0.5j ,-9.3 , -0.5-0.5j , -9 ,-9.3,-9.3,-9.7,-9.7,-9.7,-9.7,-9.9,-9.9];
Kfeedback = place(A_eq,B_eq,Poles)
A_feedback = A_eq - B_eq*Kfeedback

C_final = [1 0 0 0 0 0 0 0 0 0 0 0 ;
            0 0 1 0 0 0 0 0 0 0 0 0 ;
            0 0 0 0 1 0 0 0 0 0 0 0 ;
            0 0 0 0 0 0 1 0 0 0 0 0 
            0 0 0 0 0 0 0 1 0 0 0 0 
            0 0 0 0 0 0 0 0 1 0 0 0 ];

% Parameters and matrices
T_new = 0:0.001:10;  % Time vector
n = size(A_feedback, 1); % Number of states
I = eye(n);

% Compute state transition matrix numerically using expm
phi_t_numeric = arrayfun(@(t) expm(A_feedback * t), T_new, 'UniformOutput', false);

% Compute the state response X_new numerically
X_numeric = cellfun(@(phi) phi * Condition_1, phi_t_numeric, 'UniformOutput', false);

% Compute the output response y_new
y_numeric_new = cellfun(@(X) C_final * X, X_numeric, 'UniformOutput', false);

% Convert to matrix form for plotting
y_numeric_new = cell2mat(reshape(y_numeric_new, 1, 1, []));
y_numeric_new = reshape(y_numeric_new, size(C_final, 1), []);

% Plot
plot(T_new, y_numeric_new(1, :), 'DisplayName', 'x', 'LineWidth', 1.5); hold on;
plot(T_new, y_numeric_new(2, :), 'DisplayName', 'y', 'LineWidth', 1.5);
plot(T_new, y_numeric_new(3, :), 'DisplayName', 'z', 'LineWidth', 1.5);
plot(T_new, y_numeric_new(4, :), 'DisplayName', 'phi', 'LineWidth', 1.5);
plot(T_new, y_numeric_new(5, :), 'DisplayName', 'theta', 'LineWidth', 1.5);
plot(T_new, y_numeric_new(6, :), 'DisplayName', 'psi', 'LineWidth', 1.5);
hold off;

xlabel('Time (t)');
ylabel('Output y(t)');
title('Output Response y(t) - States x, y, z,phi,theta and psi');
legend('show');
grid on;

%% LQR

%Q1 = diag([0.1, 0.1, 0.1, 0.1, 1, 50, 50000, 50000, 0.1, 0, 0, 0]);
Q1 = diag([4000, 240, 4000, 250, 2500, 400, 250, 750, 1000, 60, 60, 10]);
R1 = 3*diag([1, 1, 5, 5]);

[K_LQR1, ~, ~] = lqr(A_eq,B_eq,Q1,R1)

A_feedback_LQR = A_eq - B_eq*K_LQR1

%% Static Pre-compensator 
G_0 = -C_final/(A_feedback) * B_eq ;
inv_G_0 = pinv(G_0);
inv_G0 = inv_G_0( : , [1 , 2 ,3 ,6]);
B_feedback = B_eq * inv_G0;
phi_s_feedback = inv(s*eye(12) - A_feedback);
%X_feedback = phi_s_feedback * x0 + phi_s_feedback*B_feedback*Uin_s;
%Y_feedback = C_final*X_feedback;


%% Static Pre-compensator With LQR 
G_0_LQR = -C_final/(A_feedback_LQR) * B_eq ;
inv_G_0_LQR = pinv(G_0_LQR);
inv_G0_LQR = inv_G_0_LQR( : , [1 , 2 ,3 ,6]);


%% Dynamic Pre-compensator
%% Dynamic Pre-compensator with LQR
Q = diag([1 1 1 1 1 1 1 1 1 1 1 1 500 4500 4000 400 ]);
% 4 ta akhari moheman ahari ham mohrm nis
R = diag([1, 5, 1, 1]);

Qk = diag([1 1 1 1 1 1 1 1 1 1 1 1 1 0.8 5000 20 ]);
Rk = diag([1, 5, 1, 1]);

Ai = [A_eq, zeros(12, 4); -C_final([1 2 3 6], 1:12), zeros(4, 4)];
Bi = [B_eq; zeros(4, 4)];
rank(ctrb(Ai,Bi))
[K_LQR,~,P_LQR] = lqr(Ai,Bi,Q,R)
[K_LQRk,~,P_LQRk] = lqr(Ai,Bi,Qk,Rk)

K_q_LQR = K_LQR(:,12+1:12+4)
K_q_LQRk = K_LQRk(:,12+1:12+4)

