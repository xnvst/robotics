function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

% persistent waypoints0 traj_time d0
% if nargin > 2
%     d = waypoints(:,2:end) - waypoints(:,1:end-1);
%     d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
%     traj_time = [0, cumsum(d0)];
%     waypoints0 = waypoints;
% else
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     t_index = find(traj_time >= t,1);
%
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
%     if(t == 0)
%         desired_state.pos = waypoints0(:,1);
%     else
%         scale = t/d0(t_index-1);
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%     end
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
% end
%


%% Fill in your code here

desired_state.pos = zeros(3,1);
desired_state.vel = zeros(3,1);
desired_state.acc = zeros(3,1);
desired_state.yaw = 0;
desired_state.yawdot = 0;

function [T] = get_polyConstant(n, k, t)
	T = zeros(n,1);
	D = zeros(n,1);
	for i=1:n
		D(i) = i-1;
		T(i) = 1;
	end
	for j=1:k
		for i=1:n
			T(i) = T(i) * D(i);
			if D(i) > 0
				D(i) = D(i) - 1;
			end
		end
	end
	for i=1:n
		T(i) = T(i) * t^D(i);
	end
	T = T';	
end

function [alpha, A, b] = get_alpha(waypoints)
	n = size(waypoints,1)-1;

	b = zeros(1,8*n);
	for i=1:n
		b(1,i) = waypoints(i);
		b(1,i+n) = waypoints(i+1);
	end
	
	A=zeros(8*n,8*n);
	for i=1:n
		A(i,((i-1)*8)+1:i*8) = get_polyConstant(8,0,0);
	end
	for i=1:n
		A(i+n,((i-1)*8)+1:i*8) = get_polyConstant(8,0,1);
	end
	for k=1:3
		A(2*n+k,1:8) = get_polyConstant(8,k,0);
	end
	for k=1:3
		A(2*n+3+k,(end-7):end) = get_polyConstant(8,k,1);
	end
	for i=2:n
		for k=1:6
			A(2*n+6+(i-2)*6+k, (i-2)*8+1:((i-2)*8+n*n)) = [get_polyConstant(8,k,1) -get_polyConstant(8,k,0)];
		end
    end

    %disp(A);
	alpha = inv(A)*b'; 
end


persistent a1 a2 a3 waypoints0 traj_time d0
	if nargin > 2
		d = waypoints(:,2:end) - waypoints(:,1:end-1);
		d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
		traj_time = [0, cumsum(d0)];
		waypoints0 = waypoints;

		a1 = get_alpha(waypoints0(1,1:end)');
		a2 = get_alpha(waypoints0(2,1:end)');
		a3 = get_alpha(waypoints0(3,1:end)');
	else
		if(t > traj_time(end))
			t = traj_time(end) - 0.0001;
		end
		t_index = find(traj_time >= t,1)-1; %between 1:n
		
		if (t_index == 0)
			t_index = 1;
		end

		if(t == 0)
			desired_state.pos = waypoints0(:,1);
			desired_state.vel = 0*waypoints0(:,1);
			desired_state.acc = 0*waypoints0(:,1);
		else
			scale = (t-traj_time(t_index))/d0(t_index);			
			index = (t_index-1)*8+1:t_index*8;
			t0 = get_polyConstant(8,0,scale)';
			desired_state.pos = [a1(index)'*t0; a2(index)'*t0; a3(index)'*t0];
			t1 = get_polyConstant(8,1,scale)';
			desired_state.vel = [a1(index)'*t1; a2(index)'*t1; a3(index)'*t1].*(1/d0(t_index));
			t2 = get_polyConstant(8,2,scale)';
			desired_state.acc = [a1(index)'*t2; a2(index)'*t2; a3(index)'*t2].*(1/d0(t_index)^2);
		end

		desired_state.yaw = 0;
		desired_state.yawdot = 0;		
	end
end

