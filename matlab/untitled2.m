x0 = sym('x0', [3 1]);
x0d = sym('x0d', [3 1]);
R = sym('R', [3 3]);

% syms ex0(x0, x0d)
ex0 = x0d-x0;
ex1 = R*(ex0);

state.x0 = [1 2 3]';
state.x0d = [10 20 30]';
state.R = round(rot_axis_angle([1 0 0], pi),2);

[state_var, state_val] = subs_state(state);
subs(ex1, state_var, state_val)
subs(ex0, state_var, state_val)
vpa(subs(R, state_var, state_val),2)




%%
% st.var = [];
% st.val = [];
% 
% st = addToState(st, x0, [1 2 3]');
% st = addToState(st, x0d, [0 0 1]');
% st = addToState(st, R, eye(3));
% 
% subs(ex1, st.var, st.val)

%%

function [state] = addToState(state, var, val)
    var_size = size(var,1) * size(var,2);
    state.var = [state.var; reshape(var, var_size, 1)];
    state.val = [state.val; reshape(val, var_size, 1)];
end

% 
% sub_var = [x0];
% sub_val = [1 2 3]';
% 
% sub_var = [sub_var; x0d];
% sub_val = [sub_val; -10; -10; -10;];
% 
% % subs(ex0, sub_var, sub_val)
% subs(ex1, sub_var, sub_val)

% x0 = [1 2 3]'

% ex0 = subs(ex0,x0,[1 2 3]')

% ex1 = ex0 + 4*x0

% syms ex1(ex0)
% ex1 = ex0 - x0d;
% subs(ex1)

% x0 = subs(x0, x0, [1 2 3]')

% subs(ex0)