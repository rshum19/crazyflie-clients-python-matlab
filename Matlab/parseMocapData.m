function [ parsedData ] = parseMocapData( data )
%PARSEMOCAPDATA parse the raw data recieved from the optitrack motion
%capture system
%
% Author: Roberto Shu
% Last edit: 9/13/2015

    nRigidBodies = data.nRigidBodies;
    parsedData.nRigidBodies = nRigidBodies;
    parsedData.pos = zeros(3,nRigidBodies);
    parsedData.vel = zeros(3,nRigidBodies);

    for i = 1:nRigidBodies

        rigidBodyData = data.RigidBodies(i);
        parsedData.pos(:,i) = [rigidBodyData.x; rigidBodyData.y; rigidBodyData.z];
    
        % Calculate yaw angle
        [parsedData.theta, parsedData.psi, parsedData.phi] = getAngles( rigidBodyData );
        
        % Calculate velocity
        parsedData.vel(:,i) = [0;0;0];

    end
    
    
end

% Get yaw angle
function [angleX, angleY, angleZ] = getAngles( rigidBodyData )


    % Note : Motive display euler's is X (Pitch), Y (Yaw), Z (Roll), Right-Handed (RHS), Relative Axes
    % so we decode eulers heres to match that.
    q = quaternion( rigidBodyData.qx, rigidBodyData.qy, rigidBodyData.qz, rigidBodyData.qw );
    qRot = quaternion( 0, 0, 0, 1);     % rotate pitch 180 to avoid 180/-180 flip for nicer graphing
    q = mtimes(q, qRot);
    angles = EulerAngles(q,'zyx');
    angleX = -angles(1) * 180.0 / pi;   % must invert due to 180 flip above
    angleY = angles(2) * 180.0 / pi;
    angleZ = -angles(3) * 180.0 / pi;   % must invert due to 180 flip above
    
end
