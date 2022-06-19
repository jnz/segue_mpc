function [] = analyze_log(filepath)

if nargin < 1
    filepath = 'logfile.txt';
end

data = load(filepath);

time_sec = data(:,1) - data(1,1);
theta = data(:,6);
thetadot = data(:,7);
u = data(:,3);
xpos = data(:,4);
vel = data(:,5);

figure(1);
clf;
hold on;
stairs(time_sec, u*10);
plot(time_sec, theta);
legend('u', 'Theta °');

figure(2);
clf;
hold on;
stairs(time_sec, u*10);
plot(time_sec, thetadot);
legend('u', 'Thetadot °/s');

figure(3);
clf;
hold on;
stairs(time_sec, u*10);
plot(time_sec, vel);
legend('u', 'Velocity (m/s)');

end

