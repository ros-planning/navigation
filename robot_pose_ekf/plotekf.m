load /tmp/odom_file.txt;
%load vo_file.txt;
load /tmp/corr_file.txt;
load /tmp/gps_file.txt;
load /tmp/imu_file.txt;

figure; 
hold on;
axis equal;
plot(odom_file(:,2), odom_file(:,3),'b');
%plot(vo_file(:,2),vo_file(:,3),'g');
plot(corr_file(:,2), corr_file(:,3),'r');
plot(gps_file(:,2), gps_file(:,3),'k');
legend('Wheel Odometry', 'Filter output', 'GPS Measurements');
%legend('Wheel Odometry','Visual Odometry', 'Filter output', 'GPS Measurements');
%legend('Wheel Odometry','Visual Odometry', 'Filter output');
hold off;

figure; 
subplot(3,1,1)
hold on;
plot(odom_file(:,1),odom_file(:,2),'b');
%plot(vo_file(:,1),vo_file(:,2),'g');
plot(corr_file(:,1),corr_file(:,2),'r');
plot(gps_file(:,1), gps_file(:,2), 'k');
legend('Wheel Odometry', 'Filter output', 'GPS Measurements');
%legend('Wheel Odometry','Visual Odometry', 'Filter output', 'GPS Measurements');
%legend('Wheel Odometry','Visual Odometry', 'Filter output');
subplot(3,1,2)
hold on;
plot(odom_file(:,1),odom_file(:,3),'b');
%plot(vo_file(:,1),vo_file(:,3),'g');
plot(corr_file(:,1),corr_file(:,3),'r');
plot(gps_file(:,1), gps_file(:,3), 'k');
legend('Wheel Odometry', 'Filter output', 'GPS Measurements');
%legend('Wheel Odometry','Visual Odometry', 'Filter output', 'GPS Measurements');
%legend('Wheel Odometry','Visual Odometry', 'Filter output');

subplot(3,1,3)
hold on;
plot(odom_file(:,1),odom_file(:,4),'b');
%plot(vo_file(:,1),vo_file(:,7),'g');
plot(corr_file(:,1),corr_file(:,7),'r');
plot(imu_file(:,1), imu_file(:,2), 'k');
legend('Wheel Odometry', 'Filter output', 'IMU Measurements');
%legend('Wheel Odometry','Visual Odometry', 'Filter output', 'IMU Measurements');



error_odom = sqrt( (odom_file(1,2)-odom_file(end,2))^2 + (odom_file(1,3)-odom_file(end,3))^2 )
%error_vo = sqrt( (vo_file(1,2)-vo_file(end,2))^2 + (vo_file(1,3)-vo_file(end,3))^2 )
error_corr = sqrt( (corr_file(1,2)-corr_file(end,2))^2 + (corr_file(1,3)-corr_file(end,3))^2 )
error_gps = sqrt( (gps_file(1,2)-gps_file(end,2))^2 + (gps_file(1,3)-gps_file(end,3))^2 )
