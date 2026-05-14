function T = scara_fkine_manual(theta1, theta2, d5, d1, d2, d3, d4)
% SCARA_FKINE_MANUAL
% Manual forward kinematics for the SCARA RRP robot
%
% Inputs:
%   theta1, theta2 : revolute joint angles (rad)
%   d5             : prismatic displacement (m)
%   d1, d2, d3, d4: robot constants (m)
%
% Output:
%   T : 4x4 homogeneous transformation matrix

th12 = theta1 + theta2;

px = d4*cos(th12) + d3*cos(theta1);
py = d4*sin(th12) + d3*sin(theta1);
pz = d1 + d2 - d5;

T = [cos(th12),  sin(th12), 0, px;
     sin(th12), -cos(th12), 0, py;
     0,          0,        -1, pz;
     0,          0,         0, 1];
end