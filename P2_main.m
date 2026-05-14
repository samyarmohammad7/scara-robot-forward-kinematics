%% =========================================================================
%  P2_main.m
%  SCARA Robot (RRP) - Complete Part 2 Solution
%  =========================================================================

clc; clear; close all;

%% STEP 0: TOOLBOX CHECK
fprintf('============================================\n');
fprintf('STEP 0: ROBOTICS TOOLBOX CHECK\n');
fprintf('============================================\n');

try
    startup_rvc;
    fprintf('RTB loaded successfully.\n');
catch ME
    error('RTB not found. Run startup_rvc first.\n%s', ME.message);
end

if isempty(which('SerialLink')) || isempty(which('Link')) || isempty(which('jtraj'))
    error('Required RTB functions are not available on the MATLAB path.');
end

fprintf('SerialLink found at: %s\n', which('SerialLink'));
fprintf('Link found at:       %s\n', which('Link'));
fprintf('jtraj found at:      %s\n', which('jtraj'));
fprintf('transl found at:     %s\n\n', which('transl'));

%% PARAMETERS
d1 = 0.3;
d2 = 0.06;
d3 = 0.3;
d4 = 0.3;
d5 = 0.2;

%% CREATE OUTPUT FOLDER
outFolder = fullfile(pwd, 'P2_outputs');
if ~exist(outFolder, 'dir')
    mkdir(outFolder);
end

%% =========================================================================
%  P2_Task 1: DERIVATION SUMMARY
%  =========================================================================
fprintf('============================================\n');
fprintf('P2_Task 1: TRANSFORMATION MATRIX DERIVATION\n');
fprintf('============================================\n\n');

fprintf('Robot parameters:\n');
fprintf('d1 = %.2f m\n', d1);
fprintf('d2 = %.2f m\n', d2);
fprintf('d3 = %.2f m\n', d3);
fprintf('d4 = %.2f m\n', d4);
fprintf('d5 = %.2f m\n\n', d5);

fprintf('D-H Table (Standard Convention):\n');
fprintf('+-------+---------+--------+--------+---------+\n');
fprintf('| Joint | theta   |   d    |   a    |  alpha  |\n');
fprintf('+-------+---------+--------+--------+---------+\n');
fprintf('|   1   | theta1* | 0.30   | 0.30   |    0    |\n');
fprintf('|   2   | theta2* | 0.06   | 0.30   |   pi    |\n');
fprintf('|   3   |   0     |  d5*   | 0      |    0    |\n');
fprintf('+-------+---------+--------+--------+---------+\n\n');

fprintf('Final transformation matrix form:\n');
fprintf('      [ cos(th1+th2)   sin(th1+th2)   0   d4*cos(th1+th2)+d3*cos(th1) ]\n');
fprintf('  T = [ sin(th1+th2)  -cos(th1+th2)   0   d4*sin(th1+th2)+d3*sin(th1) ]\n');
fprintf('      [      0              0        -1          d1 + d2 - d5         ]\n');
fprintf('      [      0              0         0                1               ]\n\n');

fprintf('End-effector position equations:\n');
fprintf('px = d4*cos(theta1 + theta2) + d3*cos(theta1)\n');
fprintf('py = d4*sin(theta1 + theta2) + d3*sin(theta1)\n');
fprintf('pz = d1 + d2 - d5\n\n');

theta1_test = deg2rad(30);
theta2_test = deg2rad(45);
d5_test = 0.1;

T_manual_example = scara_fkine_manual(theta1_test, theta2_test, d5_test, d1, d2, d3, d4);

fprintf('Numerical example for theta1 = 30 deg, theta2 = 45 deg, d5 = 0.1 m:\n');
disp(T_manual_example);

%% =========================================================================
%  P2_Task 2: MATLAB IMPLEMENTATION USING RTB
%  =========================================================================
fprintf('============================================\n');
fprintf('P2_Task 2: MATLAB IMPLEMENTATION USING RTB\n');
fprintf('============================================\n\n');

robot = build_scara_rtb(d1, d2, d3, d4, d5);

fprintf('Robot model:\n');
robot.display();

% Test cases
q_home   = [0, 0, 0];
q_test2  = [deg2rad(30), deg2rad(45), 0.10];
q_test3  = [deg2rad(90), deg2rad(-90), 0.20];

q_all = [q_home; q_test2; q_test3];
case_names = {'Home'; 'Case 2'; 'Case 3'};

px_manual = zeros(3,1);
py_manual = zeros(3,1);
pz_manual = zeros(3,1);

px_rtb = zeros(3,1);
py_rtb = zeros(3,1);
pz_rtb = zeros(3,1);

pos_error = zeros(3,1);
mat_error = zeros(3,1);

for i = 1:3
    q = q_all(i,:);

    % Manual FK
    Tm = scara_fkine_manual(q(1), q(2), q(3), d1, d2, d3, d4);

    % RTB FK
    Tr_obj = robot.fkine(q);
    Tr_num = T_to_matrix(Tr_obj);

    % Extract positions
    pm = Tm(1:3,4);
    pr = transl(Tr_obj);

    px_manual(i) = pm(1);
    py_manual(i) = pm(2);
    pz_manual(i) = pm(3);

    px_rtb(i) = pr(1);
    py_rtb(i) = pr(2);
    pz_rtb(i) = pr(3);

    pos_error(i) = norm(pm - pr);
    mat_error(i) = norm(Tm - Tr_num, 'fro');

    fprintf('--------------------------------------------\n');
    fprintf('%s\n', case_names{i});
    fprintf('q = [%.4f rad, %.4f rad, %.4f m]\n', q(1), q(2), q(3));
    fprintf('Manual T:\n');
    disp(Tm);
    fprintf('RTB T:\n');
    disp(Tr_num);
    fprintf('Manual position = [%.4f, %.4f, %.4f]\n', pm(1), pm(2), pm(3));
    fprintf('RTB position    = [%.4f, %.4f, %.4f]\n', pr(1), pr(2), pr(3));
    fprintf('Position error  = %.6e\n', pos_error(i));
    fprintf('Matrix error    = %.6e\n\n', mat_error(i));
end

results_table = table(case_names, ...
    px_manual, py_manual, pz_manual, ...
    px_rtb, py_rtb, pz_rtb, ...
    pos_error, mat_error, ...
    'VariableNames', {'Case', ...
    'px_manual', 'py_manual', 'pz_manual', ...
    'px_rtb', 'py_rtb', 'pz_rtb', ...
    'PositionError', 'MatrixError'});

fprintf('============================================\n');
fprintf('VALIDATION TABLE\n');
fprintf('============================================\n');
disp(results_table);

writetable(results_table, fullfile(outFolder, 'P2_validation_table.csv'));

%% Home position figure
fig1 = figure('Name', 'SCARA Home Position', 'Position', [100 100 850 650]);
robot.plot(q_home, 'workspace', [-0.8 0.8 -0.8 0.8 -0.2 0.6]);
title('SCARA Robot (RRP) - Home Position');
save_fig(fig1, fullfile(outFolder, 'Fig_P2_Home_Position.png'));

%% Test trajectory figure
q_start = q_home;
q_end   = [deg2rad(60), deg2rad(45), 0.15];
q_traj  = jtraj(q_start, q_end, 50);

fig2 = figure('Name', 'SCARA Test Trajectory', 'Position', [100 100 850 650]);
robot.plot(q_traj, 'workspace', [-0.8 0.8 -0.8 0.8 -0.2 0.6], 'trail', 'r-');
title('SCARA Robot - Test Trajectory');
save_fig(fig2, fullfile(outFolder, 'Fig_P2_Test_Trajectory.png'));

%% End-effector position vs step
ee_pos = zeros(size(q_traj,1), 3);
for i = 1:size(q_traj,1)
    T_i = robot.fkine(q_traj(i,:));
    ee_pos(i,:) = transl(T_i)';
end

fig3 = figure('Name', 'End-Effector Position vs Step', 'Position', [100 100 950 500]);
subplot(1,3,1);
plot(ee_pos(:,1), 'LineWidth', 1.5);
xlabel('Step'); ylabel('X (m)'); title('X Position'); grid on;

subplot(1,3,2);
plot(ee_pos(:,2), 'LineWidth', 1.5);
xlabel('Step'); ylabel('Y (m)'); title('Y Position'); grid on;

subplot(1,3,3);
plot(ee_pos(:,3), 'LineWidth', 1.5);
xlabel('Step'); ylabel('Z (m)'); title('Z Position'); grid on;

sgtitle('End-Effector Position Along Trajectory');
save_fig(fig3, fullfile(outFolder, 'Fig_P2_EndEffector_Position_vs_Step.png'));

%% =========================================================================
%  P2_Task 3: SIMULATION WITH TEACHING MODE
%  =========================================================================
fprintf('============================================\n');
fprintf('P2_Task 3: SIMULATION WITH TEACHING MODE\n');
fprintf('============================================\n\n');

fprintf('Launching teaching mode.\n');
fprintf('Move q1, q2 and q3 using the sliders.\n');
fprintf('Take a screenshot of this window manually for the report.\n\n');

fig4 = figure('Name', 'SCARA Teaching Mode', 'Position', [100 100 950 700]);
robot.plot(q_home, 'workspace', [-0.8 0.8 -0.8 0.8 -0.2 0.6]);
robot.teach(q_home);

%% Waypoint recording and playback
fprintf('--------------------------------------------\n');
fprintf('Waypoint recording and playback\n');
fprintf('--------------------------------------------\n');

waypoint1 = [0, 0, 0];
waypoint2 = [deg2rad(30), deg2rad(20), 0.05];
waypoint3 = [deg2rad(60), deg2rad(-30), 0.10];
waypoint4 = [deg2rad(-45), deg2rad(45), 0.15];
waypoint5 = [0, 0, 0];

waypoints = [waypoint1; waypoint2; waypoint3; waypoint4; waypoint5];

fprintf('Recorded waypoints:\n');
for i = 1:size(waypoints,1)
    T_wp = robot.fkine(waypoints(i,:));
    pos = transl(T_wp);
    fprintf('WP%d: [%6.1f deg, %6.1f deg, %5.3f m] -> EE = (%.4f, %.4f, %.4f)\n', ...
        i, rad2deg(waypoints(i,1)), rad2deg(waypoints(i,2)), waypoints(i,3), ...
        pos(1), pos(2), pos(3));
end

steps_per_segment = 30;
q_full = [];

for i = 1:(size(waypoints,1)-1)
    q_seg = jtraj(waypoints(i,:), waypoints(i+1,:), steps_per_segment);
    q_full = [q_full; q_seg];
end

fig5 = figure('Name', 'SCARA Waypoint Playback', 'Position', [100 100 950 700]);
robot.plot(q_full, 'workspace', [-0.8 0.8 -0.8 0.8 -0.2 0.6], 'trail', 'r-');
title('SCARA Robot - Teaching Mode Playback');
save_fig(fig5, fullfile(outFolder, 'Fig_P2_Waypoint_Playback.png'));

%% 3D End-effector path
ee_path = zeros(size(q_full,1), 3);
for i = 1:size(q_full,1)
    T_i = robot.fkine(q_full(i,:));
    ee_path(i,:) = transl(T_i)';
end

fig6 = figure('Name', 'End-Effector 3D Path', 'Position', [100 100 850 650]);
plot3(ee_path(:,1), ee_path(:,2), ee_path(:,3), 'r-', 'LineWidth', 2);
hold on;

for i = 1:size(waypoints,1)
    T_wp = robot.fkine(waypoints(i,:));
    pos = transl(T_wp);
    plot3(pos(1), pos(2), pos(3), 'b*', 'MarkerSize', 12, 'LineWidth', 1.5);
    text(pos(1)+0.02, pos(2)+0.02, pos(3)+0.02, sprintf('WP%d', i), ...
        'FontSize', 10, 'FontWeight', 'bold');
end

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('End-Effector Path Through Waypoints');
grid on;
axis equal;
view(3);
hold off;
save_fig(fig6, fullfile(outFolder, 'Fig_P2_EndEffector_Path_3D.png'));

fprintf('\n============================================\n');
fprintf('All tasks complete.\n');
fprintf('Outputs saved in folder: %s\n', outFolder);
fprintf('============================================\n');

%% =========================================================================
%  LOCAL HELPER FUNCTIONS
%  =========================================================================
function T_num = T_to_matrix(T_obj)
    try
        T_num = double(T_obj);
    catch
        T_num = T_obj.T;
    end
end

function save_fig(figHandle, filename)
    try
        exportgraphics(figHandle, filename, 'Resolution', 300);
    catch
        saveas(figHandle, filename);
    end
end