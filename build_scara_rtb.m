function robot = build_scara_rtb(d1, d2, d3, d4, d5)
% BUILD_SCARA_RTB
% Builds the SCARA RRP robot model using Peter Corke's Robotics Toolbox

L1 = Link([0, d1, d3, 0, 0], 'standard');
L1.qlim = [-pi, pi];

L2 = Link([0, d2, d4, pi, 0], 'standard');
L2.qlim = [-pi, pi];

L3 = Link([0, 0, 0, 0, 1], 'standard');
L3.qlim = [0, d5];

robot = SerialLink([L1, L2, L3], 'name', 'SCARA RRP');
end