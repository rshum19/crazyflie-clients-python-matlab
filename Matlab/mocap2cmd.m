% Micro quadrotor control using different types of control for debugging 
% 1. Simple PID control (Create first)
% 2. Centralized CBF-QP control 
% Notification 
% SI units are used here 
% The Euler angle convention is given as Z-Y-X. 
% Input: 
% mocap_data        a structure which contains the current position and yaw
% for each quadrotor 
% t                 the current time 
% Output: 
% quad_cmd          a structure which has the desired command for each
% quadrotor
% there are certain sets of necessary parameters that neeed to be tuned, as
% the reference trajectory parameter and feedback gains 

%%
function [quad_cmd] = mocap2cmd( mocap_data, t)
% the mass of individual quadrotor
M = 30 * 1e-3 * ones(1, 3); % 30 grams for each micro-quadrotor 


% =========================================================================
% additional gain matricies for the online QP-control will be given in
% later version
% =========================================================================


% parameters for the trajectory (hover case)
quadNum = mocap_data.nRigidBodies;
traj_prms.radius = zeros(3, quadNum);
traj_prms.omega = zeros(3, quadNum);
traj_prms.phi0 = zeros(3, quadNum);
delta = 2 * pi/3;
for j = 1 : quadNum

     traj_prms.radius(:, j) = [0.2 * cos(delta); 
                               0.15;
                               0.2 * sin(delta)];
end
traj_prms.psid = zeros(1, quadNum);

% gain parameters for the feedback 
gain_prms.kx = 15 * ones(3, quadNum);
gain_prms.kv = 3.6 * ones(3, quadNum);
gain_prms.alpha = 12 * ones(1, quadNum);

% compute current reference
quad_ref = getRefTraj(t, traj_prms, M);

% get the control feedback
quad_cmd = PID_feedback_control(mocap_data, quad_ref, gain_prms);

end


% compute all the necessary information of the reference for each quadrotor 
% the input M is a vector storing the mass 
% return a strcture variable
function [quad_ref] = getRefTraj(t, traj_prms, M)
% phase angle
phi = traj_prms.omega * t + traj_prms.phi0;
cphi = cos(phi);
sphi = sin(phi);

% reference for position
quad_ref.xd = (traj_prms.radius) .* cphi;
quad_ref.vd = -sphi.*(traj_prms.omega); 
quad_ref.ad = -cphi.*(traj_prms.omega.^2);

% reference for yaw 
quad_ref.psid = traj_prms.psid;

% compute the desired thrust, compensating gravity
g = 9.809915;
quad_ref.G = M * g;
end


% PID controller for computing the desired thrust 
% return a structure variable 
% no Integral term for this version now
function [quad_cmd] = PID_feedback_control(mocap_data, quad_ref, gain_prms)
% compute the desired thrust for each quadrotor 
% Assuming that the initial pose of quadrotor is aligned with the world
% frame in Motion Capture system
g = 9.809915; % gravity constant
% convert to the initial frame of micro-quadrotor 
dx = mocap_data.pos - quad_ref.xd;
dv = mocap_data.vel - quad_ref.vd;

Rq2c = [0, 1, 0;
        0, 0, 1;
        1, 0, 0];
dxq = Rq2c * dx;
dvq = Rq2c * dv;

% desired feedback thrust in initial quad frame
rdq = -gain_prms.kx .* dxq - gain_prms.kv .* dvq + quad_ref.ad;
psiq = mocap_data.psi;
cphid = cos(psiq);
sphid = sin(psiq);

% compute the desired roll and pitch angle
phi_fb = (rdq(1,:) .* sphid - rdq(2,:) .* cphid)/g;
theta_fb = (rdq(1, :) .* cphid + rdq(2,:) .* sphid)/g;
psi_fb = psiq;
thrust_fb = gain_prms.alpha .* rdq(3, :) + quad_ref.G;

% input saturation for potential overflow
thrust_fb(thrust_fb<0) = 0;
thrust_fb(thrust_fb>100) = 100;

% return the whole structure
quad_cmd.phi = phi_fb;
quad_cmd.theta = theta_fb;
quad_cmd.psi = psi_fb;
quad_cmd.thrust = thrust_fb; 
end


% CBF-QP control using centralized version where we know the  
% perform the same functionality as the PID control 
function [quad_cmd] = CBF_QP_control(mocap_data, quad_ref, gain_prms)
end