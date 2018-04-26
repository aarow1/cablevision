addpath('matlab_tools');

%% State definitions

%payload position, z positive down, must be estimated
x0 = sym('x0', [3 1]);

%payload velocity, must be estimated
x0_dot = sym('xo_dot', [3 1]);

%payload rotation, world frame, must be estimated
R0 = sym('R0', [3 3]);

%payload angular velocity, body frame, must by estimated
Omega0 = sym('Omega0', [3 1]);
% R0_dot = R0 * hat(Omega0)

%payload mass, a priori
m0 = sym('m0', [1 1]);

%payload moment of inertia, a priori
J0 = sym('J0', [3 3]);

% number of robots
n = 3;

%length of cables, [m]
l1 = 2;
l2 = 2;
l3 = 2;

%attachment points, relative to COM [m]
a1 = (0/3)*pi;
a2 = (2/3)*pi;
a3 = (4/3)*pi;
rho1 = [cos(a1) sin(a1) 0]';
rho2 = [cos(a2) sin(a2) 0]';
rho3 = [cos(a3) sin(a3) 0]';
rho = [rho1, rho2, rho3];

%cable directions, must be unit vectors, must be estimated
q = sym('q', [3, n]);

%cable angular velocity, must be estimated
w1 = sym('w1', [3 1]);
w2 = sym('w2', [3 1]);
% q1_dot = hat(w1) * q1;

%control matrix
P = zeros(6, 3*n);
for ii = 1:n
    P(:,(3*(ii-1))+(1:3)) = [eye(3); hat(rho(:,ii))];
end

%gravity [m/s^2]
g = 9.81;

%% Payload Errors

%payload desired position
x0d = sym('x0d', [3 1]);
%payload position error
ex0 = x0 - x0d;

%payload desired velocity (position dot)
x0d_dot = sym('x0d_dot', [3 1]);
ex0_dot = x0_dot - x0d_dot;

%payload desired orientation
R0d = sym('R0d', [3 3]);
%payload orientation error
eR0 = (1/2) * vee(R0d'*R0 - R0'*R0d);

%payload desired angular velocity, body frame
Omega0d = sym('Omega0d', [3 1]);
%payload angular velocity error
eOmega0 = Omega0 - R0'*R0d*Omega0d;

%payload desired acceleration
x0d_ddot = sym('x0_ddot', [3 1]);

%payload desired angular acceleration
Omega0d_dot = sym('Omega0d_dot', [3 1]);

%estimated integral force
Deltax0 = sym('Deltax0', [3 1]);
Deltax = sym('Deltax', [3 n]);

%estimated integral moment
DeltaR0 = sym('DeltaR0', [3 1]);
DeltaR = sym('DeltaR', [3 n]);

%% Payload gains

%payload pos p gain
syms kx0;
%payload pos d gain (xdot)
syms kx0_dot;

%payload orientation p gain
syms kR0;
%payload angular velocity gain
syms kOmega0;

%% Payload control

%Parallel components of force integral
Fd_integral_sum = zeros(3, 1);
for ii = 1:n
    Fd_integral_sum = Fd_integral_sum + q(:,ii)*q(:,ii)'*Deltax(:,ii);
end

%Desired force
Fd = m0 * (-kx0*ex0 - kx0_dot*ex0_dot + x0d_ddot - g*[0 0 1]') ...
    - Deltax0 - Fd_integral_sum;

Md_integral_sum = zeros(3,1);
for ii = 1:n
    Md_integral_sum = Md_integral_sum + ...
        hat(rho(:,ii))*R0*q(:,ii)*q(:,ii)'*Deltax(:,ii);
end

%Desired Moment
Md = -kR0*eR0 - kOmega0*eOmega0 + hat(R0'*R0d*Omega0d) * J0*R0'*R0d*Omega0d + ...
    J0*R0'*R0d*Omega0d_dot - DeltaR0 - Md_integral_sum;

%% Individual components from control

%Diagonal matrix of R0
diag = kron(R0, eye(n));
psuedo_P =  P' * inv(P*P');

%desired virtual controls, 3nx1
mud = diag * psuedo_P * [R0'*Fd; Md];

%virtual control input
mu = sym('mu', [3,n]);
for ii = 1:n
    mu(:,ii) = q(:,ii)*q(:,ii)'*(mud(3*(ii-1)+(1:3)));
end









