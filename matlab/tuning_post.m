%% ay22_hw5 - Tim Howarth
%% Load the file
% Filename
fname = 'cora_zigzag.bag';
% Create a bag file object with the file name
% by omitting the semicolon this displays some information about
% the bag file
bag = rosbag(fname);
  
% Display a list of the topics and message types in the bag file
bag.AvailableTopics
 
%% Create a time series of the Odometry data
% Retrieve the messages as a cell array
odom_msgs = select(bag,'Topic','/cora/sensors/p3d');
 
% Create a timeseries object of the subset of message fields we are interested in
odom_ts = timeseries(odom_msgs,'Pose.Pose.Position.X','Pose.Pose.Position.Y', ...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z', ...
    'Twist.Twist.Linear.X','Twist.Twist.Angular.Z');
%% Create a time series of the Velocity data
cmd_msgs = select(bag, 'Topic','/cora/cmd_vel');

% Create a timeseries object of the subset of message fields we are interested in
cmd_ts = timeseries(cmd_msgs, 'Linear.X','Linear.Y', ...
    'Linear.Z','Angular.X','Angular.Y','Angular.Z');

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
%% Plot Yaw Velocity and Setpoint/Goal
figure(2); clf();
plot(odom_ts.Time,odom_ts.Data(:,8))
hold on
plot(cmd_ts.Time, cmd_ts.Data(:,6),'ro')
xlabel('Time [s]')
ylabel('Yaw Velocity [rad/s]')
legend('Odom Velocity','Setpoint')
grid on

%% Yaw displacement and xy
% Note the convention for the quat2eul function is quaternion in order of WXYZ
q = odom_ts.Data(:,3:6);
e = quat2eul(q);
yaw = e(:,1);
yaw = rad2deg(yaw);

figure(3); clf();
% Plot the Data index corresponding to yaw [deg/s]
plot(odom_ts.Time,yaw)
hold on
xlabel('Time [s]')
ylabel('Yaw Displacement [deg/s]')
legend('Yaw Displacement')
grid on

figure(4); clf();
% Plot the Data index corresponding to X Y
plot(odom_ts.Data(:,1), odom_ts.Data(:,2), '.')
hold on
xlabel('X [m]')
ylabel('Y [m]')
grid on

%% generate quiver plot
% Quiver Plot
% x is the x position of the USV
% y is the y position of the USV
% vel is the Linear.X velocity
% yaw is the heading angle of the USV in radians
x = odom_ts.Data(:,1);
y = odom_ts.Data(:,2);
q = odom_ts.Data(:,3:6);
e = quat2eul(q);
yaw = e(:,1);
vel = odom_ts.Data(:,7);
 
u = vel.*cos(yaw);
v = vel.*sin(yaw);
 
figure(5); clf();
ii = 1:20:length(x);  % Decimate the data so that it plot only every Nth point.
quiver(x(ii),y(ii),u(ii),v(ii))
grid on
xlabel('X [m]')
ylabel('Y [m]')

%% surge error
% Linear.X
X1 = cmd_ts.Time;
V1 = cmd_ts.Data(:,1);

% interpolate time series
Xq1 = odom_ts.Time;
Vq1 = interp1(X1,V1,Xq1,'previous');
surge_err = Vq1 - odom_ts.Data(:,7);

figure(6); clf();

    subplot(2,1,1)
    plot(odom_ts.Time,odom_ts.Data(:,7),'-',Xq1,Vq1,'--')
    grid on
    ylabel('Surge [m/s]')
    legend('Surge Odom.','Setpoint')
    
    subplot(2,1,2),
    plot(odom_ts.Time,surge_err)
    grid on
    ylabel('Surge Error [m/s]')
    xlabel('Time [s]')
    
%% yaw error
% Linear.Z
X2 = cmd_ts.Time;
V2 = cmd_ts.Data(:,6);

% interpolate time series
Xq2 = odom_ts.Time;
Vq2 = interp1(X2,V2,Xq2,'previous');
y_err = Vq2 - odom_ts.Data(:,8);

figure(7); clf();

    subplot(2,1,1)
    plot(odom_ts.Time,odom_ts.Data(:,8),'-',Xq2,Vq2,'--')
    grid on
    ylabel('Yaw [rad/s]')
    legend('Yaw Odom.','Setpoint')
    
    subplot(2,1,2)
    plot(odom_ts.Time,y_err)
    grid on
    ylabel('Yaw Error [rad/s]')
    xlabel('Time [s]')