function [R] = Rot(axis, Theta)
    if axis == 1
        R = [1, 0, 0, 0; 0, cos(Theta), -sin(Theta), 0; 0, sin(Theta), cos(Theta), 0; 0, 0, 0, 1];
    elseif axis == 2
        R = [cos(Theta), 0, sin(Theta), 0; 0, 1, 0, 0;-sin(Theta), 0, cos(Theta), 0; 0, 0, 0, 1];
    elseif axis == 3
        R = [cos(Theta), -sin(Theta), 0, 0;sin(Theta), cos(Theta), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
    else
        wrong axis
        stop
      
    end