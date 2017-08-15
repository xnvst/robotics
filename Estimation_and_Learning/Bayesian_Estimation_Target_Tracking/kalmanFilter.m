function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
%    vx = (x - state(1)) / (t - previous_t);
%    vy = (y - state(2)) / (t - previous_t);
    % Predict 330ms into the future
%    predictx = x + vx * 0.330;
%    predicty = y + vy * 0.330;
    % State is a four dimensional element
%    state = [x, y, vx, vy];

	dt = t- previous_t;
	A = [1 0 dt 0 ;0 1 0 dt;0 0 1 0;0 0 0 1];
	C = [1 0 -10* dt 0;0 1 0 -10*dt];
	R = eye(4);
	Q =  1e-2*eye(2);
	z = [x,y]';
	
	% control matrix B is ignored for the ball
	state = A*state';
	% P - state covariance
	P = A*param.P*A' + R;
	K = P*C'/(Q+C*P*C');
	state = state + K*(z-C*state);
	param.P = P - K*C*P;
	state = state';
	predictx = state(1);
	predicty = state(2);
end
