clc; close all;

%% Model and simulation parameters
N      = 3;
n      = 2;
D      = 0.2;
D_o    = 0.6;
dT     = 50e-3;
iEnd   = 2000;
theta  = 2*pi*(0:N-1)/N;
L      = [cos(theta); sin(theta)];

%% Initial state
p_o    = [-8; 0];
v_o    = zeros(n, 1);
p_a    = p_o + [[1; 1], [-1; 1], [-1; -1]];
v_c    = zeros(n, 1);
x      = [p_o; v_o; p_a(:); v_c];

%% Graphics initialization
handle = [];
handle = graphic(handle);

%% Time-stepping simulation
for i = 0:iEnd
    t = i*dT;
    p_o_before_update = p_o;

    [dx, v, Vs, ~] = closedloop_dynamics(x, t, D, D_o, n, N, L);
    x = x + dx*dT;

    p_o = x(1:n);
    v_o = x(n+(1:n));
    p_a = reshape(x(2*n+1:end-2), [n, N]);
    v_c = x(end-1:end);

    handle = graphic(handle);
end

function [dx, v, Vs, h_ij] = closedloop_dynamics(x, t, ...
    D, D_o, n, N, L)
persistent opt;
if(isempty(opt))
    opt = optimoptions('quadprog', 'Display','off');
end


k_f = 30.0;
k_v = 0.5;

% Position-control gain used for the comparison in the paper (enable one):
% Parameter Set 1: higher gain, producing the nominal circular trajectory.
k_p = 1.0; 
% Parameter Set 2: lower gain, producing degraded trajectory tracking.
% k_p = 0.1;   


% 
p_o = x(1:n, 1);
v_o = x(n+(1:n), 1);
p_a = reshape(x(2*n+1:end-2, 1), [n, N]);


% velocity command signal
v_c  = -[cos(t*2*pi/20); sin(t*2*pi/20)];
dv_c = -2*pi/20*[-sin(t*2*pi/20); cos(t*2*pi/20)];

% function phi
epsilon = 1e-2;
s_tmp   = 2*k_f*L'*(-k_v*(v_o-v_c) + dv_c);
s_ref   = quadprog(2*epsilon*eye(N) + 2*k_f^2*(L'*L), s_tmp, [], [], [], [], zeros(N,1), [], zeros(N,1), opt);
if(max(s_ref) >= D+D_o)
    fprintf('max(s_ref)=%.1f > D+D_o=%.1f\n', max(s_ref), D+D_o);
end
p_ref   = p_o + L.*(D+D_o-s_ref');
v       = -k_p*(p_a-p_ref) + v_o;
% [v, h_ij, ~] = psi_safety_critical_controller(p_a, psi_0, D);
h_ij   = zeros(N, N);
Vs(1) = max(vecnorm(p_a-p_ref));
Vs(2) = norm(v_o-v_c);

% ODE
U  = f_function(p_a - p_o, D, D_o, k_f);
dx = [v_o; U; v(:); dv_c];
end

%  acceleration of the transported object
function U = f_function(p_tilde, D, D_o, k_f)
[n, N] = size(p_tilde);
U      = zeros(n, 1);

for i=1:N
    U  = U - k_f*max(D+D_o-norm(p_tilde(:,i)), 0)*p_tilde(:,i)/norm(p_tilde(:,i));
end
end

function handle = graphic(handle)
%GRAPHIC Initialize and update all simulation figures.

N = evalin('caller', 'N');
D = evalin('caller', 'D');
D_o = evalin('caller', 'D_o');
p_o = evalin('caller', 'p_o');
p_a = evalin('caller', 'p_a');

circle_angle = linspace(0, 2*pi, 100);
circ_Do = [sin(circle_angle)*D_o; cos(circle_angle)*D_o];
circ_D = [sin(circle_angle)*D; cos(circle_angle)*D];
shoot_index = [0, 700];
shoot_facealpha = linspace(0.1, 1.0, numel(shoot_index));

if isempty(handle)
    handle.fig1 = figure(1);
    set(handle.fig1, 'Position', [50, 50, 800, 400], 'Color', 'w');
    handle.fig1_axes = axes(handle.fig1);
    hold(handle.fig1_axes, 'on');
    axis(handle.fig1_axes, 'equal');
    grid(handle.fig1_axes, 'on');
    box(handle.fig1_axes, 'on');
    set(handle.fig1_axes, 'FontSize', 16, 'Color', 'w');

    handle.object_traj = animatedline(handle.fig1_axes, ...
        'Color', 'b', 'LineStyle', '-', 'LineWidth', 3, ...
        'DisplayName', 'Object Trajectory');
    handle.object_R = patch(handle.fig1_axes, ...
        p_o(1)+circ_Do(1,:), p_o(2)+circ_Do(2,:), 'c', ...
        'FaceAlpha', 1, 'LineStyle', '-', 'LineWidth', 2, ...
        'DisplayName', 'Object');

    handle.robots = cell(1, N);
    handle.robots_traj = cell(1, N);
    for j = 1:N
        handle.robots_traj{j} = animatedline(handle.fig1_axes, ...
            'Color', 'k', 'LineStyle', ':', 'LineWidth', 1.5, ...
            'DisplayName', 'Robots Trajectory');
        handle.robots{j} = patch(handle.fig1_axes, ...
            p_a(1,j)+circ_D(1,:), p_a(2,j)+circ_D(2,:), 'k', ...
            'FaceAlpha', 1, 'LineStyle', '-', 'LineWidth', 2, ...
            'DisplayName', 'Robots');
        if j > 1
            handle.robots{j}.HandleVisibility = 'off';
            handle.robots_traj{j}.HandleVisibility = 'off';
        end
    end

    handle.relative_positions = quiver(handle.fig1_axes, ...
        p_o(1)*ones(1,N), p_o(2)*ones(1,N), ...
        p_a(1,:)-p_o(1), p_a(2,:)-p_o(2), 1.0, ...
        'Color', 'b', 'HandleVisibility', 'off', 'LineWidth', 2);
    xlabel(handle.fig1_axes, '$[p_o]_1$', 'Interpreter', 'latex');
    ylabel(handle.fig1_axes, '$[p_o]_2$', 'Interpreter', 'latex');
    legend(handle.fig1_axes, 'Location', 'northwest', 'Interpreter', 'latex');

    handle.fig2 = figure(2);
    set(handle.fig2, 'Position', [850, 50, 800, 250], 'Color', 'w');
    handle.fig2_axes = axes(handle.fig2);
    hold(handle.fig2_axes, 'on');
    grid(handle.fig2_axes, 'on');
    box(handle.fig2_axes, 'on');
    set(handle.fig2_axes, 'FontSize', 16, 'Color', 'w', ...
        'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
    handle.fig2_Vs1 = animatedline(handle.fig2_axes, ...
        'Color', 'b', 'LineStyle', '-', 'LineWidth', 2, ...
        'DisplayName', '$\max_i |p_i-p_i^*|$');
    handle.fig2_Vs2 = animatedline(handle.fig2_axes, ...
        'Color', 'r', 'LineStyle', ':', 'LineWidth', 2, ...
        'DisplayName', '$|v_c-v_o|$');
    xlabel(handle.fig2_axes, '$t$', 'Interpreter', 'latex');
    legend(handle.fig2_axes, 'Location', 'northeast', 'Interpreter', 'latex');

    handle.fig3 = figure(3);
    set(handle.fig3, 'Position', [850, 550, 800, 350], 'Color', 'w');
    handle.fig3_axes1 = subplot(2, 1, 1, 'Parent', handle.fig3);
    handle.fig3_axes2 = subplot(2, 1, 2, 'Parent', handle.fig3);
    velocity_colors = {'r', 'b', 'k', 'm'};
    velocity_styles = {'-', '--', ':', '-.'};
    handle.fig3_v = cell(2, N);
    for component = 1:2
        current_axes = handle.(sprintf('fig3_axes%d', component));
        hold(current_axes, 'on');
        grid(current_axes, 'on');
        box(current_axes, 'on');
        set(current_axes, 'FontSize', 16, 'Color', 'w');
        for j = 1:N
            handle.fig3_v{component,j} = animatedline(current_axes, ...
                'LineStyle', velocity_styles{j}, 'Color', velocity_colors{j}, ...
                'LineWidth', 3, ...
                'DisplayName', sprintf('$[v_%d]_%d$', j, component));
        end
        xlabel(current_axes, '$t$', 'Interpreter', 'latex');
        legend(current_axes, 'Interpreter', 'latex');
    end

    handle.fig5 = figure(5);
    set(handle.fig5, 'Position', [50, 700, 800, 250], 'Color', 'w');
    handle.fig5_axes = axes(handle.fig5);
    hold(handle.fig5_axes, 'on');
    grid(handle.fig5_axes, 'on');
    box(handle.fig5_axes, 'on');
    set(handle.fig5_axes, 'FontSize', 16, 'Color', 'w');
    handle.fig5_vo1 = animatedline(handle.fig5_axes, ...
        'Color', 'b', 'LineStyle', '-', 'LineWidth', 3, 'DisplayName', '$[v_o]_1$');
    handle.fig5_vo2 = animatedline(handle.fig5_axes, ...
        'Color', 'r', 'LineStyle', '-', 'LineWidth', 3, 'DisplayName', '$[v_o]_2$');
    handle.fig5_vc1 = animatedline(handle.fig5_axes, ...
        'Color', 'b', 'LineStyle', ':', 'LineWidth', 3, 'DisplayName', '$[v_c]_1$');
    handle.fig5_vc2 = animatedline(handle.fig5_axes, ...
        'Color', 'r', 'LineStyle', ':', 'LineWidth', 3, 'DisplayName', '$[v_c]_2$');
    xlabel(handle.fig5_axes, '$t$', 'Interpreter', 'latex');
    legend(handle.fig5_axes, 'Location', 'northeast', 'Interpreter', 'latex');
    return;
end

i = evalin('caller', 'i');
t = evalin('caller', 't');
iEnd = evalin('caller', 'iEnd');
p_o_before_update = evalin('caller', 'p_o_before_update');
v_o = evalin('caller', 'v_o');
v_c = evalin('caller', 'v_c');
v = evalin('caller', 'v');
Vs = evalin('caller', 'Vs');

label_position = [];
if any(i == shoot_index) && i ~= shoot_index(end)
    snapshot_index = find(i == shoot_index, 1);
    object_copy = copyobj(handle.object_R, handle.fig1_axes);
    object_copy.FaceAlpha = shoot_facealpha(snapshot_index);
    object_copy.HandleVisibility = 'off';
    copyobj(handle.relative_positions, handle.fig1_axes);

    for j = 1:N
        robot_copy = copyobj(handle.robots{j}, handle.fig1_axes);
        robot_copy.FaceAlpha = shoot_facealpha(snapshot_index);
        robot_copy.HandleVisibility = 'off';
    end
    label_position = p_o_before_update;
elseif i == iEnd
    label_position = p_o_before_update;
end
if ~isempty(label_position)
    text(handle.fig1_axes, label_position(1), ...
        label_position(2)-D_o-3*D, sprintf('$t=%.1f$', t), ...
        'HorizontalAlignment', 'center', 'FontSize', 16, ...
        'Interpreter', 'latex');
end

handle.relative_positions.XData = p_o(1)*ones(1,N);
handle.relative_positions.YData = p_o(2)*ones(1,N);
handle.relative_positions.UData = p_a(1,:)-p_o(1);
handle.relative_positions.VData = p_a(2,:)-p_o(2);
addpoints(handle.object_traj, p_o(1), p_o(2));
handle.object_R.XData = p_o(1)+circ_Do(1,:);
handle.object_R.YData = p_o(2)+circ_Do(2,:);

for j = 1:N
    addpoints(handle.robots_traj{j}, p_a(1,j), p_a(2,j));
    handle.robots{j}.XData = p_a(1,j)+circ_D(1,:);
    handle.robots{j}.YData = p_a(2,j)+circ_D(2,:);
    addpoints(handle.fig3_v{1,j}, t, v(1,j));
    addpoints(handle.fig3_v{2,j}, t, v(2,j));
end

addpoints(handle.fig5_vo1, t, v_o(1));
addpoints(handle.fig5_vo2, t, v_o(2));
addpoints(handle.fig5_vc1, t, v_c(1));
addpoints(handle.fig5_vc2, t, v_c(2));
addpoints(handle.fig2_Vs1, t, Vs(1));
addpoints(handle.fig2_Vs2, t, Vs(2));
drawnow limitrate;
end

