function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;
%disp(params);

% FILL IN YOUR CODE HERE
Kp = 70;
Kv = 10;

m = params.mass;
g = params.gravity;

u = m * (Kp*(s_des(1)-s(1)) + Kv*(s_des(2)-s(2)) + g);

end

