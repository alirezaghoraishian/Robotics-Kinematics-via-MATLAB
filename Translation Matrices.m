function [T] = Trans(axis, len)
    if axis == 1
        T = [1, 0, 0, len; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
    elseif axis == 2
        T = [1, 0, 0, 0; 0, 1, 0, len; 0, 0, 1, 0; 0, 0, 0, 1];
    elseif axis == 3
        T = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, len; 0, 0, 0, 1];
    else
        wrong axis
        stop
      
    end