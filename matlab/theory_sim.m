addpath('matlab_tools');

%% System definitions
% These are constants that define the system

%payload mass, a priori
m0 = sym('m0', [1 1]);

%payload moment of inertia, a priori
J0 = sym('J0', [3 3]);

% number of robots
n = 3;

%length of cables, [m]
l = sym('l', [n 1]);
state.l = [2 2 2]';

%attachment points, relative to COM [m]
rho = sym('rho', [3 n]);

a1 = (0/3)*pi;
a2 = (2/3)*pi;
a3 = (4/3)*pi;
rho_num = [ cos(a1) sin(a1) 0;...
            cos(a2) sin(a2) 0;...
            cos(a3) sin(a3) 0]';
state.rho = rho_num;

%control matrix
P = sym('P', [6 3*n]);
for ii = 1:n
    P(:,(3*(ii-1))+(1:3)) = [eye(3); hat(rho(:,ii))];
end
[stvar, stval] = subs_state(state);
P = subs(P, stvar, stval);

%gravity [m/s^2]
g = 9.81;

%% State definitions
% These are time varying states that must be estimated and controlled

%payload position, z positive down, must be estimated
x0 = sym('x0', [3 1]);

%payload velocity, must be estimated
x0_dot = sym('x0_dot', [3 1]);

%payload rotation, world frame, must be estimated
R0 = sym('R0', [3 3]);

%payload angular velocity, body frame, must by estimated
Omega0 = sym('Omega0', [3 1]);
% R0_dot = R0 * hat(Omega0)

%cable directions, must be unit vectors, must be estimated
q = sym('q', [3, n]);

%cable angular velocity, must be estimated
w = sym('w', [3, n]);
% w1 = sym('w1', [3 1]);
% w2 = sym('w2', [3 1]);
% q1_dot = hat(w1) * q1;

%% Payload Errors

%payload desired position
x0d = sym('x0d', [3 1]);
%payload position error
% syms ex0(x0, x0d);
ex0 = x0 - x0d;

%payload desired velocity (position dot)
x0d_dot = sym('x0d_dot', [3 1]);
%payload velocity error
% syms ex0_dot(x0_dot, x0d_dot);
ex0_dot = x0_dot - x0d_dot;

%payload desired orientation
R0d = sym('R0d', [3 3]);
%payload orientation error
% syms eR0(R0d, R0)
eR0 = (1/2) * vee(R0d'*R0 - R0'*R0d);

%payload desired angular velocity, body frame
Omega0d = sym('Omega0d', [3 1]);
%payload angular velocity error
% syms eOmega0(Omega0, R0, R0d, Omega0d)
eOmega0 = Omega0 - R0'*R0d*Omega0d;

%payload desired acceleration
x0d_ddot = sym('x0d_ddot', [3 1]);

%payload desired angular acceleration
Omega0d_dot = sym('Omega0d_dot', [3 1]);

%estimated integral force
Deltax0 = sym('Deltax0', [3 1]);
Deltax = sym('Deltax', [3 n]);

%estimated integral moment
DeltaR0 = sym('DeltaR0', [3 1]);
% DeltaR = sym('DeltaR', [3 n]);

%% Payload gains

%payload pos p gain
kx0 = sym('kx0', [1 1]);
%payload pos d gain (xdot)
kx0_dot = sym('kx0_dot', [1 1]);

%payload orientation p gain
kR0 = sym('kR0', [1 1]);
%payload angular velocity gain
kOmega0 = sym('kOmega0', [1 1]);

%% Payload control

%Parallel components of force integral
Fd_integral_sum = zeros(3, 1);
for ii = 1:n
    Fd_integral_sum = Fd_integral_sum + q(:,ii)*q(:,ii)'*Deltax(:,ii);
end

%Desired force
Fd = m0 * (-1*kx0*ex0 - kx0_dot*ex0_dot + x0d_ddot - g*[0 0 -1]'...
    ) - Deltax0 - Fd_integral_sum;
%     - g*[0 0 1]'    ... % Gravity compensation

Md_integral_sum = zeros(3,1);
for ii = 1:n
    Md_integral_sum = Md_integral_sum + ...
        hat(rho(:,ii))*R0*q(:,ii)*q(:,ii)'*Deltax(:,ii);
end

%Desired Moment
Md = -kR0*eR0 - kOmega0*eOmega0 + hat(R0'*R0d*Omega0d) * J0*R0'*R0d*Omega0d + ...
    J0*R0'*R0d*Omega0d_dot - DeltaR0 - Md_integral_sum;

%% Individual components from control
% disp('symbolic components from control');

%Diagonal matrix of R0
a = repmat({R0}, 1, n);
diag = blkdiag(a{:});
psuedo_P =  P' * inv(P*P');

%desired virtual controls, 3nx1
mud = diag * psuedo_P * [R0'*Fd; Md];

%desired cable directions, 3nx1
qd = sym('qd', [3, n]);

%virtual control input
mu = sym('mu', [3,n]);
for ii = 1:n
%     fprintf('getting virtual control inputs %i/%i \n', ii, n);

    mu(:,ii) = q(:,ii)*q(:,ii)'*(mud(3*(ii-1)+(1:3)));
    qd(:,ii) = -(mud(3*(ii-1)+(1:3))) / norm((mud(3*(ii-1)+(1:3))));
end
