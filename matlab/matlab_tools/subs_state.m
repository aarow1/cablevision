function [st_var, st_val] = subs_state(st)
%SUBS_STATE returns the variable and value array for symbolic substitution
%of a state structure

    f_names = fieldnames(st);
    st_val = [];
    st_var = [];
    for ii = 1:length(f_names)
        
        val_ii= st.(f_names{ii});
        n_val = prod(size(val_ii));
        var_ii = sym(f_names{ii}, size(val_ii));
        st_val = [st_val; reshape(val_ii, [n_val 1])];
        st_var = [st_var; reshape(var_ii, [n_val 1])];
    end
end