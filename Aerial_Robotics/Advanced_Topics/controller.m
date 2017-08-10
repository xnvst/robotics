function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
F = 0;

% Moment
M = zeros(3,1);

Kd=[1; 1; 1];
Kp=[100; 100; 600];
Kdangle=[1; 1; 1];
Kpangle=[150; 150; 150];

rdot2=des_state.acc + Kd.*(des_state.vel-state.vel) + Kp.*(des_state.pos-state.pos);

F=params.mass*(params.gravity + rdot2(3));

if F<params.minF
    F=params.minF;
end
if F>params.maxF
    F=params.maxF;
end

phi_des=(1/params.gravity)*(rdot2(1)*sin(des_state.yaw) - rdot2(2)*cos(des_state.yaw));
theta_des=(1/params.gravity)*(rdot2(1)*cos(des_state.yaw) + rdot2(2)*sin(des_state.yaw));
rot_des=[phi_des; theta_des; des_state.yaw];
omega_des=[0; 0; des_state.yawdot];

M = Kpangle.*(rot_des-state.rot) + Kdangle.*(omega_des-state.omega);

% =================== Your code ends here ===================

end
