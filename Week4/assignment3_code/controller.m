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
Kp = [10; 10; 80];
Kd = [10; 10; 35];
r_des_ddot = des_state.acc + ...
             Kp.*(des_state.pos-state.pos) + ...
             Kd.*(des_state.vel-state.vel); %eq 17
% Thrust
F = params.mass * (params.gravity + r_des_ddot(3)); %eq 13 u1

if F<params.minF
    F=params.minF;
end
if F>params.maxF
    F=params.maxF;
end

% Moment
%M = zeros(3,1);
Kp_rot = [200; 200; 200];
Kd_rot = [.1; .1; .1];

psi_des = des_state.yaw; %eq 16a
phi_des = (r_des_ddot(1)*sin(psi_des) - r_des_ddot(2)*cos(psi_des))/params.gravity; %eq 14a
theta_des = (r_des_ddot(1)*cos(psi_des) + r_des_ddot(2)*sin(psi_des))/params.gravity; %eq 14b

rot_des = [phi_des; theta_des; psi_des];
omega_des = [0; 0; des_state.yawdot];

M = Kp_rot.*(rot_des - state.rot) + ...
    Kd_rot.*(omega_des - state.omega); %eq 10 u2
% =================== Your code ends here ===================

end
