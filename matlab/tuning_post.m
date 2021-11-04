%% Load the file
% Filename
clear all
fname = 'hw6_data.bag';
% Create a bag file object with the file name
% by omitting the semicolon this displays some information about
% the bag file
bag = rosbag(fname);
  
% Display a list of the topics and message types in the bag file
bag.AvailableTopics;
 
% Create a time series of the Odometry data
% Retrieve the messages as a cell array
odom_msgs = select(bag,'Topic','/cora/sensors/p3d');
 
% Create a timeseries object of the subset of message fields we are interested in
odom_ts = timeseries(odom_msgs,'Pose.Pose.Position.X','Pose.Pose.Position.Y', ...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z', ...
    'Twist.Twist.Linear.X','Twist.Twist.Angular.Z');

% Retrieve command messages as a celled array
cmd_msgs=select(bag,'Topic','/cora/cmd_vel');

% Create a timeseries object of command velocity
cmd_ts=timeseries(cmd_msgs,'Linear.X','Linear.Y','Linear.Z','Angular.X','Angular.Y','Angular.Z');
%%
% Plot Surge Velocity and Setpoint/Goal
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

%%
% Plot Yaw Velocity and Setpoint/Goal
figure(2); clf();
% Plot the Data index corresponding to Twist.Twist.Angular.Z
plot(odom_ts.Time,odom_ts.Data(:,8))
hold on
% Plot the Data index corresponding to Angular.Z
plot(cmd_ts.Time, cmd_ts.Data(:,6),'ro')
xlabel('Time [s]')
ylabel('Yaw Velocity [rad/s]')
legend('Odom Velocity','Setpoint')
grid on

%%
% Convert quaternion to Euler angle
% Note the convention for the quat2eul function is quaternion in order of WXYZ
q = odom_ts.Data(:,3:6);
e = quat2eul(q);
yaw = e(:,1);
yawd=rad2deg(yaw);
% Plot Yaw Displacement 
figure(3); clf();
plot(odom_ts.Time,yawd)
hold on
xlabel('Time [s]')
ylabel('Yaw Displacement [deg]')
legend('Yaw Displacement','location',"best")
grid on


%%
% Plot X vs Y position
figure(4); clf();
x=odom_ts.Data(:,1);
y=odom_ts.Data(:,2);
plot(x,y,'.')
hold on
xlabel('X [m]')
ylabel('Y [m]')
legend('USV Position','Location',"best")
grid on

%%
% Quiver Plot
% x is the x position of the USV
% y is the y position of the USV
% vel is the Linear.X velocity
% yaw is the heading angle of the USV in radians

vel=odom_ts.Data(:,7);
u = vel.*cos(yaw);
v = vel.*sin(yaw);
 
figure(5); clf();
ii = 1:20:length(x);  % Decimate the data so that it plot only every Nth point.
quiver(x(ii),y(ii),u(ii),v(ii))
grid('on')
xlabel('X [m]')
ylabel('Y [m]')




%%
%surge_error.png : Interpolate and trim the two timeseries (odometry and commands)...
% to calculate the control error (setpoint - process variable)...
% There are many ways to do this, but we used interp1 to express the two timeseries...
% on a common time basis and then find to remove the NaNs.  

% Use interpolation type "nearest" or "next" instead of default "linear"
cmd_u = interp1(cmd_ts.Time, cmd_ts.Data(:,1), odom_ts.Time, ...
        'previous','extrap');
cmd_y = interp1(cmd_ts.Time, cmd_ts.Data(:,6), odom_ts.Time, ...
        'previous','extrap');
% Calculate surge and yaw error
u_err=cmd_u-odom_ts.Data(:,7);
y_err=cmd_y-odom_ts.Data(:,8);

% Plot Surge Velocity/Setpoint and Surge Error
figure(6); clf();
subplot(2,1,1)
plot(odom_ts.Time,odom_ts.Data(:,7))
hold on
plot(odom_ts.Time, cmd_u,'r--')
xlabel('Time [s]')
ylabel('Surge Velocity [m/s]')
legend('Odom Velocity','Setpoint')
grid on
subplot(2,1,2)
plot(odom_ts.Time, u_err)
xlabel('Time [s]')
ylabel('Surge Error [m/s]')
grid on

%%
% Plot Yaw Velocity/Setpoint and Surge Error
figure(7); clf();
subplot(2,1,1)
plot(odom_ts.Time,odom_ts.Data(:,8))
hold on
plot(odom_ts.Time, cmd_y,'r--')
xlabel('Time [s]')
ylabel('Yaw Velocity [rad/s]')
legend('Odom Velocity','Setpoint')
grid on
subplot(2,1,2)
plot(odom_ts.Time, y_err)
xlabel('Time [s]')
ylabel('Yaw Error [rad/s]')
grid on
