
% Start with empty state
if (~exist('state'))
    state.var = [];
    state.val = [];
end

% Populate state
state.x0        = [2 0 1]';
state.x0_dot    = [0 0 0]';
state.R0        = rot_axis_angle([1 0 0], 1*pi/10);
state.Omega0    = [0 0 0]';
state.q         = [ 0 0 -1;...
                    0 0 -1;...
                    0 0 -1]';

% Populate desired states
state.x0d       = [1 0 1]';
state.x0d_dot   = [0 0 0]';
state.x0d_ddot  = [0 0 0]';
state.R0d       = eye(3);
state.Omega0d   = [0 0 0]';
state.Omega0d_dot = [0 0 0]';
% state.H0        = [state.R0, state.x0;...
%                     0 0 0 1];

% Populate gains
state.kx0 = 1;
state.kx0_dot = 0;
state.kR0 = 5;
state.kOmega0 = 1;

% Populare integral terms
state.Deltax0 = zeros(3,1);
state.Deltax = zeros(3,n);
state.DeltaR0 = zeros(3,1);

% Mass properties
state.m0 = 1;
state.J0 = eye(3);
                
H0 = [R0, x0; [0 0 0 1]];
[stvar, stval] = subs_state(state);
H0 = subs(H0, stvar, stval);

%% Plotting

figure(1);
% clf;
axis equal;
axis([-5 5 -5 5 0 5]);
grid on;
hold on;

% Show object
obj_r = 1.5;
obj_th = 0:.1:2*pi;
clear('obj')
obj = [ obj_r*cos(obj_th);...
        obj_r*sin(obj_th);...
        zeros(size(obj_th)); ones(size(obj_th))]; %homogenous coordinates
obj = eval(H0*(obj));

new_plot=0;
if(~exist('h_obj'))
    h_obj = fill3(0,0,0, 'r', 'facealpha', 0.5);
    new_plot = 1;
end
h_obj.set('xdata', obj(1,:),...
    'ydata', obj(2,:),...
    'zdata', obj(3,:));

%% Quad and attachment points
q_eval = subs(q, stvar, stval);
for qq = 1:n
    attach(:,qq) = H0*[state.rho(:,qq); 1];
    state.x(:,qq) = state.x0 + state.R0*state.rho(:,qq) - state.l(qq)*q_eval(:,qq);
    if(new_plot)
        h_x(qq) = plot3(0,0,0, 'b.-', 'markersize', 50);
    end
    h_x(qq).set('xdata', [state.x(1, qq) attach(1, qq)],...
        'ydata', [state.x(2, qq) attach(2, qq)],...
        'zdata', [state.x(3, qq) attach(3, qq)]);
end

%% Desired Force and moments
% Fd_eval = subs(Fd, stvar, stval);
% quiver3(state.x0(1), state.x0(2), state.x0(3), ...
%     Fd_eval(1), Fd_eval(2), Fd_eval(3), ...
%     'g', 'maxheadsize', 100, 'linewidth', 5, 'autoscalefactor', 2);
% 
% Md_eval = subs(Md, stvar, stval);
% quiver3(state.x0(1), state.x0(2), state.x0(3), ...
%     Md_eval(1), Md_eval(2), Md_eval(3), ...
%     'r', 'maxheadsize', 100, 'linewidth', 5, 'autoscalefactor', 2);

%% Desired virtual controls
qd_eval = subs(qd, stvar, stval);
for qq = 1:n
    if(new_plot)
        h_qd(qq) = plot3(0,0,0,   'g.-', 'markersize', 50);
    end
    h_qd(qq).set('xdata', attach(1, qq) - [0 state.l(qq)*qd_eval(1,qq)],...
        'ydata', attach(2, qq) - [0 state.l(qq)*qd_eval(2,qq)],...
        'zdata', attach(3, qq) - [0 state.l(qq)*qd_eval(3,qq)]);
      
end

