function [state] = addToState(state, var, val)
    var_size = size(var,1) * size(var,2);
    val_size = size(val,1) * size(val,2);
    
    if (var_size ~= val_size)
        error('addToState error: var and val not same size');
    end
    
    state.var = [state.var; reshape(var, var_size, 1)];
    state.val = [state.val; reshape(val, var_size, 1)];
end