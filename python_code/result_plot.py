import numpy as np
import matplotlib.pyplot as plt
# Load data
imu_orig = np.loadtxt('../Log/IMU_before_filter.txt')
lidar_orig = np.loadtxt('../Log/Lidar_before_filter.txt')
IMU_real = np.loadtxt('../Log/IMU_meas.txt')
IMU_esti = np.loadtxt('../Log/LiDAR_meas.txt')
Lidar_after_rot = np.loadtxt('../Log/Lidar_omg_after_rot.txt')
acc_cost = np.loadtxt('../Log/acc_cost.txt')
online_calib = np.loadtxt('../Log/mat_out.txt')
orig_imu_omg_norm = imu_orig[:, 3]
orig_lidar_omg_norm = lidar_orig[:, 3]
imu_omg_norm_after_xcorr = IMU_real[:, 3]
lidar_omg_norm_after_xcorr = IMU_esti[:, 3]
# Plot angular velocity norm before and after xcorr time calibration
plt.figure(1)
plt.subplot(2, 1, 1)
plt.plot(orig_lidar_omg_norm, linewidth=1.2)
plt.plot(orig_imu_omg_norm, linewidth=1.2)
plt.grid()
plt.title('Angvel norm before filter and xcorr time calib')
plt.legend(['LIDAR', 'IMU'])
plt.subplot(2, 1, 2)
plt.plot(lidar_omg_norm_after_xcorr, linewidth=1.5)
plt.plot(imu_omg_norm_after_xcorr, linewidth=1.2)
plt.grid()
plt.title('Angvel norm after filter and xcorr time calib')
plt.legend(['LIDAR', 'IMU'])
ang_vel_x = IMU_real[:, 0]
ang_vel_y = IMU_real[:, 1]
ang_vel_z = IMU_real[:, 2]
esti_angvel_x = IMU_esti[:, 0]
esti_angvel_y = IMU_esti[:, 1]
esti_angvel_z = IMU_esti[:, 2]
lidar_rot_angvel_x = Lidar_after_rot[:, 0]
lidar_rot_angvel_y = Lidar_after_rot[:, 1]
lidar_rot_angvel_z = Lidar_after_rot[:, 2]
# Plot angular velocity before and after rotation and time compensation
plt.figure(2)
plt.subplot(3, 1, 1)
plt.plot(esti_angvel_x, linewidth=1.2)
plt.plot(ang_vel_x, linewidth=1.2)
plt.grid()
plt.title('Angular vel X before Rotation and Time compensation')
plt.legend(['LIDAR', 'IMU'])
plt.subplot(3, 1, 2)
plt.plot(esti_angvel_y, linewidth=1.2)
plt.plot(ang_vel_y, linewidth=1.2)
plt.grid()
plt.title('Angular vel Y before Rotation and Time compensation')
plt.legend(['LIDAR', 'IMU'])
plt.subplot(3, 1, 3)
plt.plot(esti_angvel_z, linewidth=1.2)
plt.plot(ang_vel_z, linewidth=1.2)
plt.grid()
plt.title('Angular vel Z before Rotation and Time compensation')
plt.legend(['LIDAR', 'IMU'])

plt.figure(3)
plt.subplot(3, 1, 1)
plt.plot(lidar_rot_angvel_x, linewidth=1.2)
plt.plot(ang_vel_x, linewidth=1.2)
plt.grid()
plt.title('Angular vel X after Rotation and Time compensation')
plt.legend(['LIDAR', 'IMU'])

plt.subplot(3, 1, 2)
plt.plot(lidar_rot_angvel_y, linewidth=1.2)
plt.plot(ang_vel_y, linewidth=1.2)
plt.grid()
plt.title('Angular vel Y after Rotation and Time compensation')
plt.legend(['LIDAR', 'IMU'])

plt.subplot(3, 1, 3)
plt.plot(lidar_rot_angvel_z, linewidth=1.2)
plt.plot(ang_vel_z, linewidth=1.2)
plt.grid()
plt.title('Angular vel Z after Rotation and Time compensation')
plt.legend(['LIDAR', 'IMU'])


acc_I_x_orig = IMU_real[:, 4]
acc_I_y_orig = IMU_real[:, 5]
acc_I_z_orig = IMU_real[:, 6]
acc_L_x_orig = IMU_esti[:, 4]
acc_L_y_orig = IMU_esti[:, 5]
acc_L_z_orig = IMU_esti[:, 6]

plt.figure(4)
plt.subplot(3, 1, 1)
plt.plot(acc_I_x_orig, linewidth=1.2)
plt.plot(acc_L_x_orig, linewidth=1.2)
plt.grid()
plt.title('Original Linear acc X')
plt.legend(['IMU', 'Lidar - Grav'])

plt.subplot(3, 1, 2)
plt.plot(acc_I_y_orig, linewidth=1.2)
plt.plot(acc_L_y_orig, linewidth=1.2)
plt.grid()
plt.title('Original Linear acc Y')
plt.legend(['IMU', 'Lidar - Grav'])

plt.subplot(3, 1, 3)
plt.plot(acc_I_z_orig, linewidth=1.2)
plt.plot(acc_L_z_orig, linewidth=1.2)
plt.grid()
plt.title('Original Linear acc Z')
plt.legend(['IMU', 'Lidar - Grav'])


acc_I_x = acc_cost[:, 0]
acc_I_y = acc_cost[:, 1]
acc_I_z = acc_cost[:, 2]

acc_L_x = acc_cost[:, 3]
acc_L_y = acc_cost[:, 4]
acc_L_z = acc_cost[:, 5]

plt.figure(5)

plt.subplot(3, 1, 1)
plt.plot(acc_I_x, linewidth=1.2)
plt.plot(acc_L_x, linewidth=1.2)
plt.grid()
plt.legend(['IMU', 'Lidar '])
plt.title('ACC X')

plt.subplot(3, 1, 2)
plt.plot(acc_I_y, linewidth=1.2)
plt.plot(acc_L_y, linewidth=1.2)
plt.grid()
plt.legend(['IMU', 'Lidar '])
plt.title('ACC Y')

plt.subplot(3, 1, 3)
plt.plot(acc_I_z, linewidth=1.2)
plt.plot(acc_L_z, linewidth=1.2)
plt.grid()
plt.legend(['IMU', 'Lidar '])
plt.title('ACC Z')

# plt.show()

ext_rot_x = online_calib[:, 6]
ext_rot_y = online_calib[:, 7]
ext_rot_z = online_calib[:, 8]

ext_trans_x = online_calib[:, 9]
ext_trans_y = online_calib[:, 10]
ext_trans_z = online_calib[:, 11]

plt.figure(6)
plt.subplot(2, 1, 1)
plt.plot(ext_rot_x, linewidth=1.2)
plt.grid()
plt.plot(ext_rot_y, linewidth=1.2)
plt.plot(ext_rot_z, linewidth=1.2)
plt.title('Rotation Extrinsic (Euler Angle)')
plt.legend(['roll', 'pitch', 'yaw'])
plt.xlabel('frame')
plt.ylabel('deg')

plt.subplot(2, 1, 2)
plt.plot(ext_trans_x, linewidth=1.2)
plt.grid()
plt.plot(ext_trans_y, linewidth=1.2)
plt.plot(ext_trans_z, linewidth=1.2)
plt.title('Translation Extrinsic')
plt.legend(['x', 'y', 'z'])
plt.xlabel('frame')
plt.ylabel('m')

plt.show()
