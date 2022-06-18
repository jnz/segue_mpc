function [] = segue()

dbstop if error; % debugger break on error

global RUN_MAIN_LOOP; % the main loop runs as long as this is set to true
RUN_MAIN_LOOP = true; % set to false to stop simulation
global KB; % global ASCII keyboard input map
KB = zeros(intmax('uint8'), 1);

cleanup_figures   = onCleanup(@() eval('close all'));
cleanup_functions = onCleanup(@() eval('clear functions'));

%% Simulation Parameter
model.dt_ctrl_sec = 1/100;
control_every_n_epoch = 20;
model.dt_sim_sec = model.dt_ctrl_sec / control_every_n_epoch;
epoch = control_every_n_epoch;

%% Initialize model
model = model_init(model);
% select control law here:
% model = control_init_linear(model);
model = control_init_linear_ext(model);
% model = control_init_linear_ext_cl1norm(model);
% model = control_init_lqr(model);
model.epochCtrl = 0;

%% Setup Simulation
fprintf('Physic simulation: %.0f Hz\n', 1/model.dt_sim_sec);
fprintf('Control frequency: %.0f Hz\n', 1/model.dt_ctrl_sec);

model = draw_init(model);

u = 0.00;

history_time = zeros(1000, 1);
history_u = zeros(size(history_time));
history_x = zeros(length(history_time), length(model.x));

time_sec = 0;

% <Test code>
% flog = fopen('logfile.txt', 'w');
% </Test code>

%% Simulation main loop
while RUN_MAIN_LOOP == true && ishandle(model.h)

    epoch = epoch+1;
    if epoch > control_every_n_epoch
        tic
        model = drawModel(model);

        [model, u] = model.ctrl_func(model, model.dt_ctrl_sec, u);
        model.epochCtrl = model.epochCtrl + 1;
        epoch = 0;

        % <Test>
        % fprintf(flog, '%.3f 1 %.3f %.3f %.3f\n', time_sec, u, model.x(3)*180/pi, model.x(4)*180/pi);
        % fprintf('%.3f 1 %.3f %.3f %.3f\n', time_sec, u, model.x(3)*180/pi, model.x(4)*180/pi);
        % </Test>

        time_sec = time_sec + model.dt_ctrl_sec;
        history_time = [history_time(2:end); time_sec];
        history_u = [history_u(2:end, :); u'];
        history_x = [history_x(2:end, :); model.x'];

        hPlotU.XData = history_time;
        hPlotU.YData = history_u;

        % plot_prediction(model);
        time_calc_sec = toc;
        pause(max(model.dt_ctrl_sec - time_calc_sec, 0));
        % pause(0);
    end

    %u(u > model.uMax) = model.uMax;
    %u(u < model.uMin) = model.uMin;
    assert(u <= model.uMax+0.0001);
    assert(u >= model.uMin-0.0001);

    uEff = u;

    model = simstep(model, model.dt_sim_sec, uEff);
end

% fclose(flog);

end

function [model] = model_init(model)

%% Model Parameters
model.uMin = -1;
model.uMax =  1;

model.x = zeros(4,1); % [x, xdot, theta, thetadot] (m, m/s, rad, rad/s)
model.x(3) = 25*pi/180;

I_table = [
  % mass (kg),     w (m), d (m), offset to wheel center (m)
    0.200, sqrt(3)*0.085, 0.025, 0.16; ... % top emoji battery
    0.020,         0.084, 0.002, 0.15; ... % top plate
    0.045,         0.054, 0.017, 0.13; ... % raspberry pi 3
    0.040,         0.084, 0.004, 0.11; ... % rpi plate
    0.102,         0.048, 0.019, 0.10; ... % battery
    0.034,         0.084, 0.004, 0.09; ... % battery plate
    0.049,         0.080, 0.010, 0.05; ... % arduino
    0.072,         0.084, 0.004, 0.03; ... % aluminum plate
    0.046,         0.045, 0.010, 0.01; ... % motor brackets
    0.017,         0.040, 0.005, 0.02; ... % kippschutz
    ];

mass_sum_kg = 0; % sum up mass in table
mass_total_kg = 1.063; % total weight in kg
mass_wheel_kg = 0.0360; % single wheel mass in kg
mass_motor_kg = 0.150;
Ir = 0;
CoM = 0;
for i=1:size(I_table,1)
    m = I_table(i,1);
    mass_sum_kg = mass_sum_kg + m;
    w = I_table(i,2);
    h = I_table(i,3);
    d = I_table(i,4);
    Ir = Ir + 1/12*m*(w^2 + h^2) + m*d^2;
    CoM = CoM + m*d;
end

mass_sum_kg = mass_sum_kg + 2*mass_motor_kg;
Ir = Ir + 1/2 * (2*mass_motor_kg)*0.031^2; % 2x motor in CoA (150 g each)
% CoM = CoM + m*d; % skip, as motors are at the wheel center (d=0)

unaccounted_mass_kg = mass_total_kg - 2*mass_wheel_kg - mass_sum_kg;
d = 0.08;
Ir = Ir + 1/12*unaccounted_mass_kg*(0.10^2 + 0.08^2) + unaccounted_mass_kg*d^2;
CoM = CoM + unaccounted_mass_kg*d;
mass_sum_kg = mass_sum_kg + unaccounted_mass_kg;
CoM = CoM/mass_sum_kg;

% dimensions
model.L = CoM; % distance: wheel center to center of mass
model.M = 2*mass_wheel_kg; % mass wheels in kg
model.R = 0.035; % radius wheels in m
model.m = mass_total_kg - model.M;
model.Iw = 0.5*model.M*model.R^2; % m*r^2 (inertia wheel)
model.Ir = Ir; % inertia body
% motor model
model.km_per_R = 0.80; % 0.80 Nm stall torque
model.ke = 0.1/240; % max. no load speed in rad/s (=240 RPM)

end

function [x_k_dot, A, B] = fdot(x_k, u, model)

f1 = -7.54;
f2 = 0.03;
f3 = 0;  % friction
f4 = 30; % estimated from drop tests
b1 = 5.73;
b2 = -200;

%% STATE SPACE MODEL

A = [0      1              0                0;
     0      f1             f2               0;
     0      0              0                1;
     0      f3             f4               0];
B = [     0;
          b1;
          0;
          b2];

x_k_dot = A*x_k + B*u;

end

function [model] = control_init_linear(model)

model.ctrl_func = @control_run_linear;
model.N = 30;
model.Nc = 10;

%% Linear MPC
Np = model.N;
Nc = model.Nc;
[~, A, B] = fdot(model.x, 0, model);
Ap = eye(length(model.x)) + A*model.dt_ctrl_sec;
Bp = B*model.dt_ctrl_sec;
Cp = [0 1 0 0; 0 0 1 0];
% [Ap, Bp, Cp] = c2dm(A, B, Cp, zeros(size(Cp,1),1), model.dt_ctrl_sec);
model.lin_Ap = Ap;
model.lin_Bp = Bp;
model.lin_Cp = Cp;

model.lin_Rs = zeros(Np*size(Cp,1),1); % desired setpoint for next Np epochs

[Phi, F] = linearmpcgains(Ap,Bp,Cp,Nc,Np);
model.lin_Phi = Phi;
model.lin_F = F;
rw = 0.05; % tuning parameter
model.lin_Rbar = rw*eye(Nc);

end

function [model, u] = control_run_linear(model, dt_sec, u)

%% Test Linear MPC Mex module
% u_mex = mpcctrl_mex(model.x, model.x_prev, u);

%% Linear MPC
Nc = model.Nc;
% Add constraints so that uMin <= u <= uMax (limit amplitude):
HU = [tril(ones(Nc)); -tril(ones(Nc))];
ULIM = [ones(Nc,1)*model.uMax; -model.uMin*ones(Nc,1)];

H = (model.lin_Phi'*model.lin_Phi + model.lin_Rbar);
f = -model.lin_Phi'*(model.lin_Rs - model.lin_F*model.x);

%upred = quadprog(H, f, HU, ULIM, [], [], [], [], [], optimset('Display', 'off')); % disable console output
upred = qphild(H, f, HU, ULIM);
upred(upred > model.uMax) = model.uMax;
upred(upred < model.uMin) = model.uMin;
Ypred = model.lin_F*model.x + model.lin_Phi*upred;
% predict_plot(model, upred, model.lin_Cp, Ypred, dt_sec);

u = upred(1);

end

function [] = predict_plot(model, upred, Cp, Ypred, dt_sec)

uhorizon = zeros(model.N, 1);
uhorizon(1:length(upred)) = upred;

epochs = length(uhorizon);
Ysim = zeros(epochs, size(Cp,1));
Ysim_euler = zeros(epochs, size(Cp,1));
Ypred = reshape(Ypred, size(Cp,1), epochs)';
time_sec = zeros(epochs, 1);

current_state = Cp*model.x;

model_euler = model;

for i=1:epochs
    model = simstep(model, dt_sec, uhorizon(i)); % RK4 integrator
    model_euler.x = fstateEuler(dt_sec, model_euler.x, uhorizon(i), model_euler); % simple euler for comparision

    Ysim(i, :) = Cp*model.x;
    Ysim_euler(i, :) = Cp*model_euler.x;
    time_sec(i) = i*dt_sec;
end
% Add current state at time=0
time_sec = [0; time_sec];
Ysim = [current_state'; Ysim];
Ysim_euler = [current_state'; Ysim_euler];
Ypred = [current_state'; Ypred];
uhorizon = [uhorizon; 0];

index = 2;
figure(99);
clf;
hold on;
plot(time_sec, Ysim(:,1), 'k-*');
plot(time_sec, Ysim_euler(:,1), 'b->');
plot(time_sec, Ypred(:,1), 'r--');
stairs(time_sec, uhorizon*max(abs(Ysim(:,1)))/5, 'b');
legend('Velocity sim (RK4)', 'Velocity sim (Euler)', 'Velocity pred');
xlabel('Time (sec');
ylabel('Velocity (m/s)');

figure(98);
clf;
hold on;
plot(time_sec, Ysim(:,index)*180/pi, 'k-*');
plot(time_sec, Ysim_euler(:,index)*180/pi, 'b->');
plot(time_sec, Ypred(:,index)*180/pi, 'r--');
stairs(time_sec, uhorizon*max(abs(Ysim(:,index)*180/pi))/5, 'b');
legend('Theta sim RK4(°)', 'Theta sim Euler(°)', 'Theta pred(°)');
xlabel('Time (sec');
ylabel('Theta (°)');
ylim([-90 90]);

end

function [model] = control_init_linear_ext(model)

model.ctrl_func = @control_run_linear_ext;
model.N = 200;
model.Nc = 40;

%% Ext. Linear MPC
Np = model.N;
Nc = model.Nc;
model.x_prev = model.x;
[~, A, B] = fdot(model.x, 0, model);
Ap = eye(length(model.x)) + A*model.dt_ctrl_sec;
Bp = B*model.dt_ctrl_sec;
Cp = [0 1 0 0; 0 0 1 0];
% [Ap, Bp, Cp] = c2dm(A, B, Cp, zeros(size(Cp,1),1), model.dt_ctrl_sec);
model.lin_Ap = Ap;
model.lin_Bp = Bp;
model.lin_Cp = Cp;

model.lin_Rs = zeros(Np*size(Cp,1),1); % desired setpoint for next Np epochs

%[Phi, F] = mpcgain(Ap,Bp,Cp,Nc,Np);
[Phi, F] = mpcgainEx(Ap,Bp,Cp,Nc,Np);
model.lin_Phi = Phi;
model.lin_F = F;
rw = 0.1; % tuning parameter
model.lin_Rbar = rw*eye(Nc);

end

function [model] = control_init_linear_ext_cl1norm(model)

model.ctrl_func = @control_run_linear_ext_cl1norm;
model.N = 40;
model.Nc = 10;

%% Ext. Linear MPC
Np = model.N;
Nc = model.Nc;
model.x_prev = model.x;
[~, A, B] = fdot(model.x, 0, model);
Ap = eye(length(model.x)) + A*model.dt_ctrl_sec;
Bp = B*model.dt_ctrl_sec;
Cp = [0 1 0 0; 0 0 1 0];
% [Ap, Bp, Cp] = c2dm(A, B, Cp, zeros(size(Cp,1),1), model.dt_ctrl_sec);
model.lin_Ap = Ap;
model.lin_Bp = Bp;
model.lin_Cp = Cp;

model.lin_Rs = zeros(Np*size(Cp,1),1); % desired setpoint for next Np epochs

[Phi, F] = mpcgainEx(Ap,Bp,Cp,Nc,Np);
model.lin_Phi = Phi;
model.lin_F = F;

end

function [model] = control_init_lqr(model)

model.ctrl_func = @control_run_lqr;

[~, A, B] = fdot(model.x, 0, model);
model.K = lqr(A, B, diag([0.00001 0.01 1 0.2]), 10);

end

function [model, u] = control_run_linear_ext(model, dt_sec, u)

%% Linear MPC
Nc = model.Nc;
dx = model.x - model.x_prev; % dx = x(k) - x(k-1)
model.x_prev = model.x;
y = model.lin_Cp * model.x; % y(k)
x_e = [ dx; y ]; % dx = x(k) - x(k-1)
% Add constraints so that uMin <= u <= uMax (limit amplitude):
HU = [tril(ones(Nc)); -tril(ones(Nc))];
ULIM = [ones(Nc,1)*model.uMax - u; -model.uMin*ones(Nc,1) + u];
% Add constraints to changes in input variables |du|<duMax
% duMax = 0.5;
% HU = [HU; eye(Nc); -eye(Nc)];
% ULIM = [ULIM; ones(Nc*2, 1)*duMax];

H = (model.lin_Phi'*model.lin_Phi + model.lin_Rbar);
f = -model.lin_Phi'*(model.lin_Rs - model.lin_F*x_e);

%A = model.lin_Phi;
%A = [A; eye(Nc)];
%l = model.lin_Rs-model.lin_F*x_e;
%l = [l; zeros(Nc, 1)];
%DU = cl1norm(A, l, [],[], HU, ULIM);

DU = qphild(H, f, HU, ULIM);
% DU = quadprog(H, f, HU, ULIM, [], [], [], [], [], optimset('Display', 'off')); % disable console output

% <DEBUG>
uhorizon = DU*0;
uhorizon(1) = u + DU(1);
for i=2:length(uhorizon)
    uhorizon(i) = uhorizon(i-1) + DU(i);
end
Ypred = model.lin_F*x_e + model.lin_Phi*DU;
% predict_plot(model, uhorizon, model.lin_Cp, Ypred, dt_sec);
% </DEBUG>

u = u + DU(1); % u(k) = u(k-1) + du(k)
u(u > model.uMax) = model.uMax;
u(u < model.uMin) = model.uMin;


end

function [model, u] = control_run_linear_ext_cl1norm(model, dt_sec, u)

%% Linear MPC
Nc = model.Nc;
dx = model.x - model.x_prev; % dx = x(k) - x(k-1)
model.x_prev = model.x;
y = model.lin_Cp * model.x; % y(k)
x_e = [ dx; y ]; % dx = x(k) - x(k-1)
% Add constraints so that uMin <= u <= uMax (limit amplitude):
HU = [tril(ones(Nc)); -tril(ones(Nc))];
ULIM = [ones(Nc,1)*model.uMax - u; -model.uMin*ones(Nc,1) + u];
% Add constraints to changes in input variables |du|<duMax
% duMax = 0.5;
% HU = [HU; eye(Nc); -eye(Nc)];
% ULIM = [ULIM; ones(Nc*2, 1)*duMax];

A = model.lin_Phi;
A = [A; eye(Nc)];
l = model.lin_Rs-model.lin_F*x_e;
l = [l; zeros(Nc, 1)];
DU = cl1norm(A, l, [],[], HU, ULIM);

% <DEBUG>
uhorizon = DU*0;
uhorizon(1) = u + DU(1);
for i=2:length(uhorizon)
    uhorizon(i) = uhorizon(i-1) + DU(i);
end
Ypred = model.lin_F*x_e + model.lin_Phi*DU;
% predict_plot(model, uhorizon, model.lin_Cp, Ypred, dt_sec);
% </DEBUG>

u = u + DU(1); % u(k) = u(k-1) + du(k)
u(u > model.uMax) = model.uMax;
u(u < model.uMin) = model.uMin;


end

function [model, u] = control_run_lqr(model, dt_sec, u)

%% LQR
u = -model.K * model.x;

%% PID
% e = 0 - model.x(3);
%
% model.integrator = model.integrator + e*dt_sec;
% derivative = (e - model.previous_error) / dt_sec;
%
% Kp = -1.9;
% Ki = -0.01;
% Kd = -0.005;
% u = Kp*e + Ki*model.integrator + Kd*derivative;

end

function [model] = simstep(model, dt_sec, u)
    u = min(max(u,model.uMin),model.uMax);

    if (model.x(3) > pi/2)
        model.x(3) = pi/2;
    end

    if model.x(3) < -pi/2
        model.x(3) = -pi/2;
    end

    model.x = fstateRK4(dt_sec, model.x, u, model);
end

function fnext = fstateRK4(dt_sec, x_k, u, model)

k1 = fdot(x_k, u, model);
k2 = fdot(x_k + dt_sec/2*k1, u, model);
k3 = fdot(x_k + dt_sec/2*k2, u, model);
k4 = fdot(x_k + dt_sec*k3, u, model);
fnext = x_k + dt_sec/6*(k1 + 2*k2 + 2*k3 + k4);

end

function fnext = fstateEuler(dt_sec, x_k, u, model)

k1 = fdot(x_k, u, model);
fnext = x_k + k1*dt_sec;

end

function [model] = draw_init(model)

model.h = figure(1);
set(model.h, 'DeleteFcn', @helper_StopLoopFcn);
set(model.h, 'KeyPressFcn',@helper_keyDownListener);
set(model.h, 'KeyReleaseFcn', @helper_keyUpListener);
clf;

end

function [model] = drawModel(model)

% figure(model.h);
set(0, 'currentfigure', model.h);

clf;
x = model.x(1);
th = model.x(3);
L = model.L;
R = model.R;

% positions
y = R; % cart vertical position

draw_rectangle([x; y],0.05,L,th,'#ff00ff', '#00ff00');

rectangle('Position', [x-R 0 2*R 2*R],'Curvature',[1 1], 'FaceColor', '#CE54FC', 'EdgeColor', '#ff0000');

% xlim([-4 4]);
% ylim([-2 L+R+2]);
% set(gcf,'Position',get(0, 'Screensize'))

% drawnow limitrate nocallbacks;
drawnow limitrate;

end

function[]= draw_rectangle(center_location,L,H,theta,rgb, rgbface)

center1=center_location(1);
center2=center_location(2);
R= ([cos(theta), sin(theta); -sin(theta), cos(theta)]);
X=([-L/2, L/2, L/2, -L/2]);
Y=([0, 0, H, H]);
for i=1:4
    T(:,i)=R*[X(i); Y(i)];
end
x_lower_left=center1+T(1,1);
x_lower_right=center1+T(1,2);
x_upper_right=center1+T(1,3);
x_upper_left=center1+T(1,4);
y_lower_left=center2+T(2,1);
y_lower_right=center2+T(2,2);
y_upper_right=center2+T(2,3);
y_upper_left=center2+T(2,4);
x_coor=[x_lower_left x_lower_right x_upper_right x_upper_left];
y_coor=[y_lower_left y_lower_right y_upper_right y_upper_left];
patch('Vertices',[x_coor; y_coor]','Faces',[1 2 3 4],'Edgecolor',rgb,'Facecolor',rgbface,'Linewidth',1.2);
axis equal;
end

function [] = plot_prediction(model)

figure(2); % prediction plot
clf;

hold on;
prediction_time = model.N * model.dt_ctrl_sec;

time_plot_sec = 0:model.dt_ctrl_sec:(prediction_time-model.dt_ctrl_sec);
plot(time_plot_sec, model.out.x(2:end, 3)*180/pi); % plot theta (=index 3)
theta_forward_sim = zeros(length(time_plot_sec), 1);

m = model;
for i=1:length(time_plot_sec)
    m = simstep(m, model.dt_ctrl_sec, model.out.u(i));
    theta_forward_sim(i) = m.x(3)*180/pi;
end
plot(time_plot_sec, theta_forward_sim);
legend('Predicted', 'Simulated');
ylim([-90 90]);

end

% Helper functions for keyboard input etc.
% ========================================

function helper_StopLoopFcn(~, ~) % (hObject, event)
% Figure closed event handler function.
global RUN_MAIN_LOOP
RUN_MAIN_LOOP = false;
end

function helper_keyDownListener(~,event)
updateKeys(event, 1);
end

function helper_keyUpListener(~,event)
updateKeys(event, 0);
end

function updateKeys(event, keydown)
global RUN_MAIN_LOOP;
global KB; % global ASCII keyboard input map
if strcmp(event.Key, 'escape') > 0
    RUN_MAIN_LOOP = false;
end
KB(uint8(event.Character)) = keydown;
end
