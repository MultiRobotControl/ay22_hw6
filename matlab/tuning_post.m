%% Load the file
% Filename
fname = 'cora_tuning.bag';
% Create a bag file object with the file name
% by omitting the semicolon this displays some information about
% the bag file
bag = rosbag(fname);
  
% Display a list of the topics and message types in the bag file
bag.AvailableTopics;
 
%% Create a time series of the Odometry data
% Retrieve the messages as a cell array
odom_msgs = select(bag,'Topic','/cora/sensors/p3d');
 
% Create a timeseries object of the subset of message fields we are interested in
odom_ts = timeseries(odom_msgs,'Pose.Pose.Position.X','Pose.Pose.Position.Y', ...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z', ...
    'Twist.Twist.Linear.X','Twist.Twist.Angular.Z');

cmd_msgs = select(bag,'Topic','/cora/cmd_vel');

cmd_ts = timeseries(cmd_msgs,'Linear.X','Linear.Y','Linear.Z',...
    'Angular.X','Angular.Y','Angular.Z');

%% Plot Surge Velocity and Setpoint/Goal
figure(1); clf();
% Plot the Data index corresponding to Twist.Twist.Linear.X
plot(odom_ts.Time,odom_ts.Data(:,7))
hold on
% Plot the Data index corresponding to Linear.X
plot(cmd_ts.Time, cmd_ts.Data(:,1),'ro')
xlabel('Time [s]')
ylabel('Surge Velocity [m/s]')
legend('Odom Velocity','Setpoint')
grid on

%% Yaw Velocity
figure(2); clf;
% Plot the Data index corresponding to Twist.Twist.Linear.Y
plot(odom_ts.Time,odom_ts.Data(:,8))
hold on
% Plot the Data index corresponding to Linear.X
plot(cmd_ts.Time, cmd_ts.Data(:,6),'ro')
xlabel('Time [s]')
ylabel('Yaw Velocity [rad/s]')
legend('Odom Velocity','Setpoint')
grid on

%% Yaw Displacement
% Convert quaternion to Euler angle
% Note the convention for the quat2eul function is quaternion in order of WXYZ
q = odom_ts.Data(:,3:6);
e = quat2eul(q);
yaw = e(:,1);

figure(3);clf;
plot(odom_ts.Time,yaw)
grid on
xlabel('Time [s]')
ylabel('Yaw Displacement [deg]')

%% XY Plot
figure(4);clf;
plot(odom_ts.Data(:,1),odom_ts.Data(:,2),'b')
xlabel('X [m]')
ylabel('Y [m]')
legend('X-Y Position')
grid on
%% Quiver
% Quiver Plot
% x is the x position of the USV
% y is the y position of the USV
% vel is the Linear.X velocity
% yaw is the heading angle of the USV in radians

x = odom_ts.Data(:,1);
y = odom_ts.Data(:,2);
vel = odom_ts.Data(:,7);
u = vel.*cos(yaw);
v = vel.*sin(yaw);

figure(5); clf;
ii = 1:20:length(x);  % Decimate the data so that it plot only every Nth point.
quiver(x(ii),y(ii),u(ii),v(ii))
grid('on')
xlabel('X [m]')
ylabel('Y [m]')

%% Surge Error
%Interpolation
%linear velocity - u
x1 = cmd_ts.Time;
v1 = cmd_ts.Data(:,1);
xq1 = odom_ts.Time;
vq1 = interp1(x1,v1,xq1,'previous');
s_error = vq1 - odom_ts.Data(:,7);

figure(6);clf;
subplot(211),plot(odom_ts.Time,odom_ts.Data(:,7),'b-',xq1,vq1,'r--')
grid on
ylabel('Surge Velocity [m/s]')
legend('Surge Odom.','Setpoint')
subplot(212),plot(odom_ts.Time,s_error) %surge error
grid on
ylabel('Surge Error [m/s]')
xlabel('Time [s]')

%% Yaw Error
%angular velocity - v
x2 = cmd_ts.Time;
v2 = cmd_ts.Data(:,6);
xq2 = odom_ts.Time;
vq2 = interp1(x2,v2,xq2,'previous');
y_error = vq2 - odom_ts.Data(:,8);

figure(7);clf;
subplot(211),plot(odom_ts.Time,odom_ts.Data(:,8),'b-',xq2,vq2,'r--')
grid on
ylabel('Yaw Velocity [rad/s]')
xlabel('Time [s]')
legend('Yaw Odom.','Setpoint')
subplot(212),plot(odom_ts.Time,y_error) %yaw error
grid on
ylabel('Yaw Error [rad/s]')
xlabel('Time [s]')

%%
% Plot the Data index corresponding to Twist.Twist.Linear.X
% figure(2);
% plot(odom_ts.Time,odom_ts.Data(:,7))
% hold on
% Plot the Data index corresponding to Linear.X
