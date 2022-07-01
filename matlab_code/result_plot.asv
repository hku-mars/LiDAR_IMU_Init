clear all
close all

imu_orig = load('../Log/IMU_before_filter.txt');
lidar_orig = load('../Log/Lidar_before_filter.txt');
IMU_real = load('../Log/IMU_meas.txt');
IMU_esti = load('../Log/LiDAR_meas.txt');
Lidar_after_rot = load('../Log/Lidar_omg_after_rot.txt');
acc_cost = load('../Log/acc_cost.txt');
online_calib = load('../Log/mat_out.txt');

orig_imu_omg_norm = imu_orig(:,4);
orig_lidar_omg_norm = lidar_orig(:,4);
imu_omg_norm_after_xcorr = IMU_real(:,4);
lidar_omg_norm_after_xcorr = IMU_esti(:,4);


figure(1)
subplot(2,1,1)
plot(orig_lidar_omg_norm,'linewidth',1.2);
hold on
grid on
plot(orig_imu_omg_norm,'linewidth',1.2);
title('Angvel norm before filter and xcorr time calib');
legend('LIDAR','IMU');

subplot(2,1,2)
plot(lidar_omg_norm_after_xcorr,'linewidth',1.5);
hold on
grid on
plot(imu_omg_norm_after_xcorr,'linewidth',1.2);
title('Angvel norm after filter and xcorr time calib');
legend('LIDAR','IMU');


ang_vel_x = IMU_real(:,1);
ang_vel_y = IMU_real(:,2);
ang_vel_z = IMU_real(:,3);
esti_angvel_x = IMU_esti(:,1);
esti_angvel_y = IMU_esti(:,2);
esti_angvel_z = IMU_esti(:,3);

lidar_rot_angvel_x = Lidar_after_rot(:,1);
lidar_rot_angvel_y = Lidar_after_rot(:,2);
lidar_rot_angvel_z = Lidar_after_rot(:,3);

figure(2)
subplot(3,1,1)
plot(esti_angvel_x,'linewidth',1.2);
hold on
grid on
plot(ang_vel_x,'linewidth',1.2);
title('Angular vel X before Rotation and Time compensation');
legend('LIDAR','IMU');

subplot(3,1,2)
plot(esti_angvel_y,'linewidth',1.2);
hold on
grid on
plot(ang_vel_y,'linewidth',1.2);
title('Angular vel Y before Rotation and Time compensation');
legend('LIDAR','IMU');

subplot(3,1,3)
plot(esti_angvel_z,'linewidth',1.2);
hold on
grid on
plot(ang_vel_z,'linewidth',1.2);
title('Angular vel Z before Rotation and Time compensation');
legend('LIDAR','IMU');

figure(3)
subplot(3,1,1)
plot(lidar_rot_angvel_x,'linewidth',1.2);
hold on
grid on
plot(ang_vel_x,'linewidth',1.2);
title('Angular vel X after Rotation and Time compensation');
legend('LIDAR','IMU');

subplot(3,1,2)
plot(lidar_rot_angvel_y,'linewidth',1.2);
hold on
grid on
plot(ang_vel_y,'linewidth',1.2);
title('Angular vel Y after Rotation and Time compensation');
legend('LIDAR','IMU');

subplot(3,1,3)
plot(lidar_rot_angvel_z,'linewidth',1.2);
hold on
grid on
plot(ang_vel_z,'linewidth',1.2);
title('Angular vel Z after Rotation and Time compensation');
legend('LIDAR','IMU');

acc_I_x_orig = IMU_real(:,5);
acc_I_y_orig = IMU_real(:,6);
acc_I_z_orig = IMU_real(:,7);
acc_L_x_orig = IMU_esti(:,5);
acc_L_y_orig = IMU_esti(:,6);
acc_L_z_orig = IMU_esti(:,7);
figure(4)
subplot(3,1,1)
plot(acc_I_x_orig,'linewidth',1.2);
hold on
grid on
plot(acc_L_x_orig,'linewidth',1.2);
title('Original Linear acc X');
legend('IMU','Lidar - Grav');
subplot(3,1,2)
plot(acc_I_y_orig,'linewidth',1.2);
hold on
grid on
plot(acc_L_y_orig,'linewidth',1.2);
title('Original Linear acc Y');
legend('IMU','Lidar - Grav');
subplot(3,1,3)
plot(acc_I_z_orig,'linewidth',1.2);
hold on
grid on
plot(acc_L_z_orig,'linewidth',1.2);
title('Original Linear acc Z');
legend('IMU','Lidar - Grav');


acc_I_x = acc_cost(:,1);
acc_I_y = acc_cost(:,2);
acc_I_z = acc_cost(:,3);

acc_L_x = acc_cost(:,4);
acc_L_y = acc_cost(:,5);
acc_L_z = acc_cost(:,6);

figure(5)
subplot(3,1,1)
plot(acc_I_x, 'linewidth', 1.2);
hold on
grid on
plot(acc_L_x, 'linewidth', 1.2);
legend('IMU','Lidar ')
title('ACC X')

subplot(3,1,2)
plot(acc_I_y, 'linewidth', 1.2);
hold on
grid on
plot(acc_L_y, 'linewidth', 1.2);
legend('IMU','Lidar ')
title('ACC Y')

subplot(3,1,3)
plot(acc_I_z, 'linewidth', 1.2);
hold on
grid on
plot(acc_L_z, 'linewidth', 1.2);
legend('IMU','Lidar ')
title('ACC Z')


% ext_rot_x = online_calib(:,7);
% ext_rot_y = online_calib(:,8);
% ext_rot_z = online_calib(:,9);
% 
% ext_trans_x = online_calib(:,10);
% ext_trans_y = online_calib(:,11);
% ext_trans_z = online_calib(:,12);
% 
% figure(6)
% subplot(2,1,1)
% plot(ext_rot_x,'linewidth',1.2);
% hold on
% grid on
% plot(ext_rot_y,'linewidth',1.2);
% plot(ext_rot_z,'linewidth',1.2);
% title('Rotation Extrinsic (Euler Angle)');
% legend('roll','pitch','yaw');
% xlabel('frame');ylabel('deg');
% 
% subplot(2,1,2)
% plot(ext_trans_x,'linewidth',1.2);
% hold on
% grid on
% plot(ext_trans_y,'linewidth',1.2);
% plot(ext_trans_z,'linewidth',1.2);
% title('Translation Extrinsic');
% legend('x','y','z');
% xlabel('frame');ylabel('m') 
