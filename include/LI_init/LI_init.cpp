#include "LI_init.h"

/*
Description: LI-Init: a temporal-spatial lidar-inertial initialization package
Author: Fangcheng Zhu
email: zhufc@connect.hku.hk
*/

LI_Init::LI_Init()
        : time_delay_IMU_wtr_Lidar(0.0), time_lag_1(0.0), time_lag_2(0.0), lag_IMU_wtr_Lidar(0) {
    fout_LiDAR_meas.open(FILE_DIR("LiDAR_meas.txt"), ios::out);
    fout_IMU_meas.open(FILE_DIR("IMU_meas.txt"), ios::out);
    fout_before_filt_IMU.open(FILE_DIR("IMU_before_filter.txt"), ios::out);
    fout_before_filt_Lidar.open(FILE_DIR("Lidar_before_filter.txt"), ios::out);
    fout_acc_cost.open(FILE_DIR("acc_cost.txt"), ios::out);
    fout_after_rot.open(FILE_DIR("Lidar_omg_after_rot.txt"), ios::out);
    data_accum_length = 300;
    Rot_Grav_wrt_Init_Lidar = Eye3d;
    Trans_Lidar_wrt_IMU = Zero3d;
    Rot_Lidar_wrt_IMU = Eye3d;
    gyro_bias = Zero3d;
    acc_bias = Zero3d;
}

LI_Init::~LI_Init() = default;

void LI_Init::set_IMU_state(const deque<CalibState> &IMU_states) {
    IMU_state_group.assign(IMU_states.begin(), IMU_states.end() - 1);
}

void LI_Init::set_Lidar_state(const deque<CalibState> &Lidar_states) {
    Lidar_state_group.assign(Lidar_states.begin(), Lidar_states.end() - 1);
}

void LI_Init::set_states_2nd_filter(const deque<CalibState> &IMU_states, const deque<CalibState> &Lidar_states) {
    for (int i = 0; i < IMU_state_group.size(); i++) {
        IMU_state_group[i].ang_acc = IMU_states[i].ang_acc;
        Lidar_state_group[i].ang_acc = Lidar_states[i].ang_acc;
        Lidar_state_group[i].linear_acc = Lidar_states[i].linear_acc;
    }
}

void LI_Init::fout_before_filter() {
    for (auto it_IMU = IMU_state_group.begin(); it_IMU != IMU_state_group.end() - 1; it_IMU++) {
        fout_before_filt_IMU << setprecision(15) << it_IMU->ang_vel.transpose() << " " << it_IMU->ang_vel.norm() << " "
                             << it_IMU->linear_acc.transpose() << " " << it_IMU->timeStamp << endl;
    }
    for (auto it = Lidar_state_group.begin(); it != Lidar_state_group.end() - 1; it++) {
        fout_before_filt_Lidar << setprecision(15) << it->ang_vel.transpose() << " " << it->ang_vel.norm() << " "
                               << it->timeStamp << endl;
    }
}

void LI_Init::push_ALL_IMU_CalibState(const sensor_msgs::Imu::ConstPtr &msg, const double &mean_acc_norm) {
    CalibState IMUstate;
    IMUstate.ang_vel = V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    IMUstate.linear_acc =
            V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) / mean_acc_norm *
            G_m_s2;
    IMUstate.timeStamp = msg->header.stamp.toSec();
    IMU_state_group_ALL.push_back(IMUstate);
}

void LI_Init::push_IMU_CalibState(const V3D &omg, const V3D &acc, const double &timestamp) {
    CalibState IMUstate;
    IMUstate.ang_vel = omg;
    IMUstate.linear_acc = acc;
    IMUstate.timeStamp = timestamp;
    IMU_state_group.push_back(IMUstate);
}

void LI_Init::push_Lidar_CalibState(const M3D &rot, const V3D &omg, const V3D &linear_vel, const double &timestamp) {
    CalibState Lidarstate;
    Lidarstate.rot_end = rot;
    Lidarstate.ang_vel = omg;
    Lidarstate.linear_vel = linear_vel;
    Lidarstate.timeStamp = timestamp;
    Lidar_state_group.push_back(Lidarstate);
}


void LI_Init::downsample_interpolate_IMU(const double &move_start_time) {

    while (IMU_state_group_ALL.front().timeStamp < move_start_time - 3.0)
        IMU_state_group_ALL.pop_front();
    while (Lidar_state_group.front().timeStamp < move_start_time - 3.0)
        Lidar_state_group.pop_front();


    //Original IMU measurements
    deque<CalibState> IMU_states_all_origin;
    IMU_states_all_origin.assign(IMU_state_group_ALL.begin(), IMU_state_group_ALL.end() - 1);

    //Mean filter to attenuate noise
    int mean_filt_size = 2;
    for (int i = mean_filt_size; i < IMU_state_group_ALL.size() - mean_filt_size; i++) {
        V3D acc_real = Zero3d;
        for (int k = -mean_filt_size; k < mean_filt_size + 1; k++)
            acc_real += (IMU_states_all_origin[i + k].linear_acc - acc_real) / (k + mean_filt_size + 1);
        IMU_state_group_ALL[i].linear_acc = acc_real;
    }



    //Down-sample and interpolation，Fig.4 in the paper
    for (int i = 0; i < Lidar_state_group.size(); i++) {
        for (int j = 1; j < IMU_state_group_ALL.size(); j++) {
            if (IMU_state_group_ALL[j - 1].timeStamp <= Lidar_state_group[i].timeStamp
                && IMU_state_group_ALL[j].timeStamp > Lidar_state_group[i].timeStamp) {
                CalibState IMU_state_interpolation;
                double delta_t = IMU_state_group_ALL[j].timeStamp - IMU_state_group_ALL[j - 1].timeStamp;
                double delta_t_right = IMU_state_group_ALL[j].timeStamp - Lidar_state_group[i].timeStamp;
                double s = delta_t_right / delta_t;
                IMU_state_interpolation.ang_vel = s * IMU_state_group_ALL[j - 1].ang_vel +
                                                  (1 - s) * IMU_state_group_ALL[j].ang_vel;
                IMU_state_interpolation.linear_acc = s * IMU_state_group_ALL[j - 1].linear_acc +
                                                     (1 - s) * IMU_state_group_ALL[j].linear_acc;
                push_IMU_CalibState(IMU_state_interpolation.ang_vel, IMU_state_interpolation.linear_acc,
                                    Lidar_state_group[i].timeStamp);
                break;
            }
        }
    }

}

void LI_Init::central_diff() {
    auto it_IMU_state = IMU_state_group.begin() + 1;
    for (; it_IMU_state != IMU_state_group.end() - 2; it_IMU_state++) {
        auto last_imu = it_IMU_state - 1;
        auto next_imu = it_IMU_state + 1;
        double dt_imu = next_imu->timeStamp - last_imu->timeStamp;
        it_IMU_state->ang_acc =
                (next_imu->ang_vel - last_imu->ang_vel) / dt_imu;
        fout_IMU_meas << setprecision(12) << it_IMU_state->ang_vel.transpose() << " " << it_IMU_state->ang_vel.norm()
                      << " "
                      <<
                      it_IMU_state->linear_acc.transpose() << " " << it_IMU_state->ang_acc.transpose() << " "
                      << it_IMU_state->timeStamp << endl;
    }

    auto it_Lidar_state = Lidar_state_group.begin() + 1;
    for (; it_Lidar_state != Lidar_state_group.end() - 2; it_Lidar_state++) {
        auto last_lidar = it_Lidar_state - 1;
        auto next_lidar = it_Lidar_state + 1;
        double dt_lidar = next_lidar->timeStamp - last_lidar->timeStamp;
        it_Lidar_state->ang_acc =
                (next_lidar->ang_vel - last_lidar->ang_vel) / dt_lidar;
        it_Lidar_state->linear_acc =
                (next_lidar->linear_vel - last_lidar->linear_vel) / dt_lidar;
        fout_LiDAR_meas << setprecision(12) << it_Lidar_state->ang_vel.transpose() << " "
                        << it_Lidar_state->ang_vel.norm()
                        << " " <<
                        (it_Lidar_state->linear_acc - STD_GRAV).transpose() << " "
                        << it_Lidar_state->ang_acc.transpose()
                        << " " << it_Lidar_state->timeStamp << endl;
    }
}

void LI_Init::xcorr_temporal_init(const double &odom_freq) {
    int N = IMU_state_group.size();
    //Calculate mean value of IMU and LiDAR angular velocity
    double mean_IMU_ang_vel = 0, mean_LiDAR_ang_vel = 0;
    for (int i = 0; i < N; i++) {
        mean_IMU_ang_vel += (IMU_state_group[i].ang_vel.norm() - mean_IMU_ang_vel) / (i + 1);
        mean_LiDAR_ang_vel += (Lidar_state_group[i].ang_vel.norm() - mean_LiDAR_ang_vel) / (i + 1);
    }

    //Calculate zero-centered cross correlation
    double max_corr = -DBL_MAX;
    for (int lag = -N + 1; lag < N; lag++) {
        double corr = 0;
        int cnt = 0;
        for (int i = 0; i < N; i++) {
            int j = i + lag;
            if (j < 0 || j > N - 1)
                continue;
            else {
                cnt++;
                corr += (IMU_state_group[i].ang_vel.norm() - mean_IMU_ang_vel) *
                        (Lidar_state_group[j].ang_vel.norm() - mean_LiDAR_ang_vel);  // Zero-centered cross correlation
            }
        }

        if (corr > max_corr) {
            max_corr = corr;
            lag_IMU_wtr_Lidar = -lag;
        }
    }

    time_lag_1 = lag_IMU_wtr_Lidar / odom_freq;
    cout << "Max Cross-correlation: IMU lag wtr Lidar : " << -lag_IMU_wtr_Lidar << endl;
}

void LI_Init::IMU_time_compensate(const double &lag_time, const bool &is_discard) {
    if (is_discard) {
        //Discard first 10 Lidar estimations and corresponding IMU measurements due to long time interval
        int i = 0;
        while (i < 10) {
            Lidar_state_group.pop_front();
            IMU_state_group.pop_front();
            i++;
        }
    }

    auto it_IMU_state = IMU_state_group.begin();
    for (; it_IMU_state != IMU_state_group.end() - 1; it_IMU_state++) {
        it_IMU_state->timeStamp = it_IMU_state->timeStamp - lag_time;
    }

    while (Lidar_state_group.front().timeStamp < IMU_state_group.front().timeStamp)
        Lidar_state_group.pop_front();
    while (Lidar_state_group.front().timeStamp > IMU_state_group[1].timeStamp)
        IMU_state_group.pop_front();

    //Align the size of two sequences
    while (IMU_state_group.size() > Lidar_state_group.size())
        IMU_state_group.pop_back();
    while (IMU_state_group.size() < Lidar_state_group.size())
        Lidar_state_group.pop_back();
}

void LI_Init::cut_sequence_tail() {
    for (int i = 0; i < 20; ++i) {
        Lidar_state_group.pop_back();
        IMU_state_group.pop_back();
    }
    while (Lidar_state_group.front().timeStamp < IMU_state_group.front().timeStamp)
        Lidar_state_group.pop_front();
    while (Lidar_state_group.front().timeStamp > IMU_state_group[1].timeStamp)
        IMU_state_group.pop_front();

    //Align the size of two sequences
    while (IMU_state_group.size() > Lidar_state_group.size())
        IMU_state_group.pop_back();
    while (IMU_state_group.size() < Lidar_state_group.size())
        Lidar_state_group.pop_back();
}

void LI_Init::acc_interpolate() {
    //Interpolation to get acc_I(t_L)
    for (int i = 1; i < Lidar_state_group.size() - 1; i++) {
        double deltaT = Lidar_state_group[i].timeStamp - IMU_state_group[i].timeStamp;
        if (deltaT > 0) {
            double DeltaT = IMU_state_group[i + 1].timeStamp - IMU_state_group[i].timeStamp;
            double s = deltaT / DeltaT;
            IMU_state_group[i].linear_acc = s * IMU_state_group[i + 1].linear_acc +
                                            (1 - s) * IMU_state_group[i].linear_acc;
            IMU_state_group[i].timeStamp += deltaT;
        } else {
            double DeltaT = IMU_state_group[i].timeStamp - IMU_state_group[i - 1].timeStamp;
            double s = -deltaT / DeltaT;
            IMU_state_group[i].linear_acc = s * IMU_state_group[i - 1].linear_acc +
                                            (1 - s) * IMU_state_group[i].linear_acc;
            IMU_state_group[i].timeStamp += deltaT;
        }
    }
}

void LI_Init::Butter_filt(const deque<CalibState> &signal_in, deque<CalibState> &signal_out) {
    LI_Init::Butterworth butter;
    butter.extend_num = 10 * (butter.Coeff_size - 1);
    auto it_front = signal_in.begin() + butter.extend_num;
    auto it_back = signal_in.end() - 1 - butter.extend_num;

    deque<CalibState> extend_front;
    deque<CalibState> extend_back;

    for (int idx = 0; idx < butter.extend_num; idx++) {
        extend_front.push_back(*it_front);
        extend_back.push_front(*it_back);
        it_front--;
        it_back++;
    }

    deque<CalibState> sig_extended(signal_in);
    while (!extend_front.empty()) {
        sig_extended.push_front(extend_front.back());
        extend_front.pop_back();
    }
    while (!extend_back.empty()) {
        sig_extended.push_back(extend_back.front());
        extend_back.pop_front();
    }

    deque<CalibState> sig_out(sig_extended);
    //One-direction Butterworth filter Starts (all states)
    for (int i = butter.Coeff_size; i < sig_extended.size() - butter.extend_num; i++) {
        CalibState temp_state;
        for (int j = 0; j < butter.Coeff_size; j++) {
            auto it_sig_ext = *(sig_extended.begin() + i - j);
            temp_state += it_sig_ext * butter.Coeff_b[j];
        }
        for (int jj = 1; jj < butter.Coeff_size; jj++) {
            auto it_sig_out = *(sig_out.begin() + i - jj);
            temp_state -= it_sig_out * butter.Coeff_a[jj];
        }
        sig_out[i] = temp_state;
    }

    for (auto it = sig_out.begin() + butter.extend_num; it != sig_out.end() - butter.extend_num; it++) {
        signal_out.push_back(*it);
    }
}

void LI_Init::zero_phase_filt(const deque<CalibState> &signal_in, deque<CalibState> &signal_out) {
    deque<CalibState> sig_out1;
    Butter_filt(signal_in, sig_out1);

    deque<CalibState> sig_rev(sig_out1);
    reverse(sig_rev.begin(), sig_rev.end()); //Reverse the elements

    Butter_filt(sig_rev, signal_out);
    reverse(signal_out.begin(), signal_out.end()); //Reverse the elements
}

void LI_Init::solve_Rotation_only() {
    double R_LI_quat[4];
    R_LI_quat[0] = 1;
    R_LI_quat[1] = 0;
    R_LI_quat[2] = 0;
    R_LI_quat[3] = 0;

    ceres::LocalParameterization *quatParam = new ceres::QuaternionParameterization();
    ceres::Problem problem_rot;
    problem_rot.AddParameterBlock(R_LI_quat, 4, quatParam);


    for (int i = 0; i < IMU_state_group.size(); i++) {
        M3D Lidar_angvel_skew;
        Lidar_angvel_skew << SKEW_SYM_MATRX(Lidar_state_group[i].ang_vel);
        problem_rot.AddResidualBlock(Angular_Vel_Cost_only_Rot::Create(IMU_state_group[i].ang_vel,
                                                                       Lidar_state_group[i].ang_vel),
                                     nullptr,
                                     R_LI_quat);

    }
    ceres::Solver::Options options_quat;
    ceres::Solver::Summary summary_quat;
    ceres::Solve(options_quat, &problem_rot, &summary_quat);
    Eigen::Quaterniond q_LI(R_LI_quat[0], R_LI_quat[1], R_LI_quat[2], R_LI_quat[3]);
    Rot_Lidar_wrt_IMU = q_LI.matrix();
}

void LI_Init::solve_Rot_bias_gyro(double &timediff_imu_wrt_lidar) {
    Eigen::Quaterniond quat(Rot_Lidar_wrt_IMU);
    double R_LI_quat[4];
    R_LI_quat[0] = quat.w();
    R_LI_quat[1] = quat.x();
    R_LI_quat[2] = quat.y();
    R_LI_quat[3] = quat.z();

    double bias_g[3]; //Initial value of gyro bias
    bias_g[0] = 0;
    bias_g[1] = 0;
    bias_g[2] = 0;

    double time_lag2 = 0; //Second time lag (IMU wtr Lidar)

    ceres::LocalParameterization *quatParam = new ceres::QuaternionParameterization();
    ceres::Problem problem_ang_vel;

    problem_ang_vel.AddParameterBlock(R_LI_quat, 4, quatParam);
    problem_ang_vel.AddParameterBlock(bias_g, 3);

    for (int i = 0; i < IMU_state_group.size(); i++) {
        double deltaT = Lidar_state_group[i].timeStamp - IMU_state_group[i].timeStamp;
        problem_ang_vel.AddResidualBlock(Angular_Vel_Cost::Create(IMU_state_group[i].ang_vel,
                                                                  IMU_state_group[i].ang_acc,
                                                                  Lidar_state_group[i].ang_vel,
                                                                  deltaT),
                                         nullptr,
                                         R_LI_quat,
                                         bias_g,
                                         &time_lag2);
    }


    ceres::Solver::Options options_quat;
    ceres::Solver::Summary summary_quat;
    ceres::Solve(options_quat, &problem_ang_vel, &summary_quat);

    Eigen::Quaterniond q_LI(R_LI_quat[0], R_LI_quat[1], R_LI_quat[2], R_LI_quat[3]);
    Rot_Lidar_wrt_IMU = q_LI.matrix();
    V3D euler_angle = RotMtoEuler(q_LI.matrix());
    gyro_bias = V3D(bias_g[0], bias_g[1], bias_g[2]);

    time_lag_2 = time_lag2;
    time_delay_IMU_wtr_Lidar = time_lag_1 + time_lag_2;
    cout << "Total time delay (IMU wtr Lidar): " << time_delay_IMU_wtr_Lidar + timediff_imu_wrt_lidar << " s" << endl;
    cout << "Using LIO: SUBTRACT this value from IMU timestamp" << endl
         << "           or ADD this value to LiDAR timestamp." << endl <<endl;

    //The second temporal compensation
    IMU_time_compensate(get_lag_time_2(), false);

    for (int i = 0; i < Lidar_state_group.size(); i++) {
        fout_after_rot << setprecision(12) << (Rot_Lidar_wrt_IMU * Lidar_state_group[i].ang_vel + gyro_bias).transpose()
                       << " " << Lidar_state_group[i].timeStamp << endl;
    }

}

void LI_Init::solve_trans_biasacc_grav() {
    M3D Rot_Init = Eye3d;
    Rot_Init.diagonal() = V3D(1, 1, 1);
    Eigen::Quaterniond quat(Rot_Init);
    double R_GL0_quat[4];
    R_GL0_quat[0] = quat.w();
    R_GL0_quat[1] = quat.x();
    R_GL0_quat[2] = quat.y();
    R_GL0_quat[3] = quat.z();

    double bias_aL[3]; //Initial value of acc bias
    bias_aL[0] = 0;
    bias_aL[1] = 0;
    bias_aL[2] = 0;

    double Trans_IL[3]; //Initial value of Translation of IL (IMU with respect to Lidar)
    Trans_IL[0] = 0.0;
    Trans_IL[1] = 0.0;
    Trans_IL[2] = 0.0;

    ceres::LocalParameterization *quatParam = new ceres::QuaternionParameterization();
    ceres::Problem problem_acc;

    problem_acc.AddParameterBlock(R_GL0_quat, 4, quatParam);
    problem_acc.AddParameterBlock(bias_aL, 3);
    problem_acc.AddParameterBlock(Trans_IL, 3);

    //Jacobian of acc_bias, gravity, Translation
    int Jaco_size = 3 * Lidar_state_group.size();
    MatrixXd Jacobian(Jaco_size, 9);
    Jacobian.setZero();

    //Jacobian of Translation
    MatrixXd Jaco_Trans(Jaco_size, 3);
    Jaco_Trans.setZero();

    for (int i = 0; i < IMU_state_group.size(); i++) {
        problem_acc.AddResidualBlock(Linear_acc_Cost::Create(Lidar_state_group[i],
                                                             Rot_Lidar_wrt_IMU,
                                                             IMU_state_group[i].linear_acc),
                                     nullptr,
                                     R_GL0_quat,
                                     bias_aL,
                                     Trans_IL);

        Jacobian.block<3, 3>(3 * i, 0) = -Lidar_state_group[i].rot_end;
        Jacobian.block<3, 3>(3 * i, 3) << SKEW_SYM_MATRX(STD_GRAV);
        M3D omg_skew, angacc_skew;
        omg_skew << SKEW_SYM_MATRX(Lidar_state_group[i].ang_vel);
        angacc_skew << SKEW_SYM_MATRX(Lidar_state_group[i].ang_acc);
        M3D Jaco_trans_i = omg_skew * omg_skew + angacc_skew;
        Jaco_Trans.block<3, 3>(3 * i, 0) = Jaco_trans_i;
        Jacobian.block<3, 3>(3 * i, 6) = Jaco_trans_i;
    }

    for (int index = 0; index < 3; ++index) {
        problem_acc.SetParameterUpperBound(bias_aL, index, 0.01);
        problem_acc.SetParameterLowerBound(bias_aL, index, -0.01);
    }

    ceres::Solver::Options options_acc;
    ceres::Solver::Summary summary_acc;
    ceres::Solve(options_acc, &problem_acc, &summary_acc);


    Eigen::Quaterniond q_GL0(R_GL0_quat[0], R_GL0_quat[1], R_GL0_quat[2], R_GL0_quat[3]);
    Rot_Grav_wrt_Init_Lidar = q_GL0.matrix();
    Grav_L0 = Rot_Grav_wrt_Init_Lidar * STD_GRAV;

    V3D bias_a_Lidar(bias_aL[0], bias_aL[1], bias_aL[2]);
    acc_bias = Rot_Lidar_wrt_IMU * bias_a_Lidar;

    V3D Trans_IL_vec(Trans_IL[0], Trans_IL[1], Trans_IL[2]);
    Trans_Lidar_wrt_IMU = -Rot_Lidar_wrt_IMU * Trans_IL_vec;

    for (int i = 0; i < IMU_state_group.size(); i++) {
        V3D acc_I = Lidar_state_group[i].rot_end * Rot_Lidar_wrt_IMU.transpose() * IMU_state_group[i].linear_acc -
                    Lidar_state_group[i].rot_end * bias_a_Lidar;
        V3D acc_L = Lidar_state_group[i].linear_acc +
                    Lidar_state_group[i].rot_end * Jaco_Trans.block<3, 3>(3 * i, 0) * Trans_IL_vec - Grav_L0;
        fout_acc_cost << setprecision(10) << acc_I.transpose() << " " << acc_L.transpose() << " "
                      << IMU_state_group[i].timeStamp << " " << Lidar_state_group[i].timeStamp << endl;
    }

    M3D Hessian_Trans = Jaco_Trans.transpose() * Jaco_Trans;
    EigenSolver<M3D> es_trans(Hessian_Trans);
    M3D EigenValue_mat_trans = es_trans.pseudoEigenvalueMatrix();
    M3D EigenVec_mat_trans = es_trans.pseudoEigenvectors();

}

void LI_Init::normalize_acc(deque<CalibState> &signal_in) {
    V3D mean_acc(0, 0, 0);

    for (int i = 1; i < 10; i++) {
        mean_acc += (signal_in[i].linear_acc - mean_acc) / i;
    }

    for (int i = 0; i < signal_in.size(); i++) {
        signal_in[i].linear_acc = signal_in[i].linear_acc / mean_acc.norm() * G_m_s2;
    }
}

bool LI_Init::data_sufficiency_assess(MatrixXd &Jacobian_rot, int &frame_num, V3D &lidar_omg, int &orig_odom_freq,
                                      int &cut_frame_num) {
    //Calculation of Rotation Jacobian
    M3D lidar_omg_skew;
    lidar_omg_skew << SKEW_SYM_MATRX(lidar_omg);
    Jacobian_rot.block<3, 3>(3 * frame_num, 0) = lidar_omg_skew;
    bool data_sufficient = false;

    //Give a Data Appraisal every second
    if (frame_num % orig_odom_freq * cut_frame_num == 0) {
        M3D Hessian_rot = Jacobian_rot.transpose() * Jacobian_rot;
        EigenSolver<M3D> es(Hessian_rot);
        V3D EigenValue = es.eigenvalues().real();
        M3D EigenVec_mat = es.eigenvectors().real();

        M3D EigenMatCwise = EigenVec_mat.cwiseProduct(EigenVec_mat);
        std::vector<double> EigenMat_1_col{EigenMatCwise(0, 0), EigenMatCwise(1, 0), EigenMatCwise(2, 0)};
        std::vector<double> EigenMat_2_col{EigenMatCwise(0, 1), EigenMatCwise(1, 1), EigenMatCwise(2, 1)};
        std::vector<double> EigenMat_3_col{EigenMatCwise(0, 2), EigenMatCwise(1, 2), EigenMatCwise(2, 2)};

        int maxPos[3] = {0};
        maxPos[0] = max_element(EigenMat_1_col.begin(), EigenMat_1_col.end()) - EigenMat_1_col.begin();
        maxPos[1] = max_element(EigenMat_2_col.begin(), EigenMat_2_col.end()) - EigenMat_2_col.begin();
        maxPos[2] = max_element(EigenMat_3_col.begin(), EigenMat_3_col.end()) - EigenMat_3_col.begin();

        V3D Scaled_Eigen = EigenValue / data_accum_length;   //the larger data_accum_length is, the more data is needed
        V3D Rot_percent(Scaled_Eigen[1] * Scaled_Eigen[2],
                        Scaled_Eigen[0] * Scaled_Eigen[2],
                        Scaled_Eigen[0] * Scaled_Eigen[1]);

        V3D Rot_percent_scaled(Rot_percent[0] < 0.99 ? Rot_percent[0] : 1,
                               Rot_percent[1] < 0.99 ? Rot_percent[1] : 1,
                               Rot_percent[2] < 0.99 ? Rot_percent[2] : 1);

        int axis[3];
        axis[2] = max_element(maxPos, maxPos + 3) - maxPos;
        axis[0] = min_element(maxPos, maxPos + 3) - maxPos;
        axis[1] = 3 - (axis[0] + axis[2]);


        clear(); //clear the screen
        printf("\033[3A\r");
        printProgress(Rot_percent_scaled[axis[0]], 88);
        printProgress(Rot_percent_scaled[axis[1]], 89);
        printProgress(Rot_percent_scaled[axis[2]], 90);
        fflush(stdout);
        if (Rot_percent[0] > 0.99 && Rot_percent[1] > 0.99 && Rot_percent[2] > 0.99) {
            printf(BOLDCYAN "[Initialization] Data accumulation finished, Lidar IMU initialization begins.\n\n" RESET);
            printf(BOLDBLUE"============================================================ \n\n" RESET);
            data_sufficient = true;
        }
    }
    if (data_sufficient)
        return true;
    else
        return false;
}


void LI_Init::printProgress(double percentage, int axis_ascii) {
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf(BOLDCYAN "[Initialization] ");
    if (percentage < 1) {
        printf(BOLDYELLOW "Rotation around Lidar %c Axis: ", char(axis_ascii));
        printf(YELLOW "%3d%% [%.*s%*s]\n", val, lpad, PBSTR, rpad, "");
        cout << RESET;
    } else {
        printf(BOLDGREEN "Rotation around Lidar %c Axis: ", char(axis_ascii));
        printf(GREEN "%3d%% [%.*s%*s]\n", val, lpad, PBSTR, rpad, "");
        cout << RESET;
    }
}

void LI_Init::clear() {
    // CSI[2J clears screen, CSI[H moves the cursor to top-left corner
    cout << "\x1B[2J\x1B[H";
}

void LI_Init::LI_Initialization(int &orig_odom_freq, int &cut_frame_num, double &timediff_imu_wrt_lidar,
                                const double &move_start_time) {

    TimeConsuming time("Batch optimization");

    downsample_interpolate_IMU(move_start_time);
    fout_before_filter();
    IMU_time_compensate(0.0, true);


    deque<CalibState> IMU_after_zero_phase;
    deque<CalibState> Lidar_after_zero_phase;
    zero_phase_filt(get_IMU_state(), IMU_after_zero_phase);
    normalize_acc(IMU_after_zero_phase);
    zero_phase_filt(get_Lidar_state(), Lidar_after_zero_phase);
    set_IMU_state(IMU_after_zero_phase);
    set_Lidar_state(Lidar_after_zero_phase);
    cut_sequence_tail();

    xcorr_temporal_init(orig_odom_freq * cut_frame_num);
    IMU_time_compensate(get_lag_time_1(), false);

    central_diff();

    deque<CalibState> IMU_after_2nd_zero_phase;
    deque<CalibState> Lidar_after_2nd_zero_phase;
    zero_phase_filt(get_IMU_state(), IMU_after_2nd_zero_phase);
    zero_phase_filt(get_Lidar_state(), Lidar_after_2nd_zero_phase);
    set_states_2nd_filter(IMU_after_2nd_zero_phase, Lidar_after_2nd_zero_phase);


    solve_Rotation_only();

    solve_Rot_bias_gyro(timediff_imu_wrt_lidar);

    acc_interpolate();

    solve_trans_biasacc_grav();

    printf(BOLDBLUE"============================================================ \n\n" RESET);
    double time_L_I = timediff_imu_wrt_lidar + time_delay_IMU_wtr_Lidar;
    print_initialization_result(time_L_I, Rot_Lidar_wrt_IMU, Trans_Lidar_wrt_IMU, gyro_bias, acc_bias, Grav_L0);

    printf(BOLDBLUE"============================================================ \n\n" RESET);
    printf(BOLDCYAN "[Initialization] Lidar IMU initialization done.\n");
    printf("" RESET);
}

void
LI_Init::print_initialization_result(double &time_L_I, M3D &R_L_I, V3D &p_L_I, V3D &bias_g, V3D &bias_a, V3D gravity) {
    cout.setf(ios::fixed);
    printf(BOLDCYAN "[Init Result] " RESET);
    cout << setprecision(6)
         << "Rotation LiDAR to IMU    = " << RotMtoEuler(R_L_I).transpose() * 57.3 << " deg" << endl;
    printf(BOLDCYAN "[Init Result] " RESET);
    cout << "Translation LiDAR to IMU = " << p_L_I.transpose() << " m" << endl;
    printf(BOLDCYAN "[Init Result] " RESET);
    printf("Time Lag IMU to LiDAR    = %.8lf s \n", time_L_I);
    printf(BOLDCYAN "[Init Result] " RESET);
    cout << "Bias of Gyroscope        = " << bias_g.transpose() << " rad/s" << endl;
    printf(BOLDCYAN "[Init Result] " RESET);
    cout << "Bias of Accelerometer    = " << bias_a.transpose() << " m/s^2" << endl;
    printf(BOLDCYAN "[Init Result] " RESET);
    cout << "Gravity in World Frame   = " << gravity.transpose() << " m/s^2" << endl << endl;
}

void LI_Init::plot_result() {
    vector<vector<double>> IMU_omg(3), IMU_acc(3), IMU_ang_acc(3), Lidar_omg(3), Lidar_acc(3), Lidar_ang_acc(3);
    for (auto it_IMU_state = IMU_state_group.begin(); it_IMU_state != IMU_state_group.end() - 1; it_IMU_state++) {
        for (int i = 0; i < 3; i++) {
            IMU_omg[i].push_back(it_IMU_state->ang_vel[i]);
            IMU_acc[i].push_back(it_IMU_state->linear_acc[i]);
            IMU_ang_acc[i].push_back(it_IMU_state->ang_acc[i]);
        }
    }
    for (auto it_Lidar_state = Lidar_state_group.begin();
         it_Lidar_state != Lidar_state_group.end() - 1; it_Lidar_state++) {
        for (int i = 0; i < 3; i++) {
            Lidar_omg[i].push_back(it_Lidar_state->ang_vel[i]);
            Lidar_acc[i].push_back(it_Lidar_state->linear_acc[i]);
            Lidar_ang_acc[i].push_back(it_Lidar_state->ang_acc[i]);
        }
    }

    plt::figure(1);
    plt::subplot(2, 3, 1);
    plt::named_plot("IMU omg x", IMU_omg[0]);
    plt::named_plot("IMU omg y", IMU_omg[1]);
    plt::named_plot("IMU omg z", IMU_omg[2]);
    plt::legend();
    plt::grid(true);

    plt::subplot(2, 3, 2);
    plt::named_plot("IMU acc x", IMU_acc[0]);
    plt::named_plot("IMU acc y", IMU_acc[1]);
    plt::named_plot("IMU acc z", IMU_acc[2]);
    plt::legend();
    plt::grid(true);

    plt::subplot(2, 3, 3);
    plt::named_plot("IMU ang acc x", IMU_ang_acc[0]);
    plt::named_plot("IMU ang acc y", IMU_ang_acc[1]);
    plt::named_plot("IMU ang acc z", IMU_ang_acc[2]);
    plt::legend();
    plt::grid(true);

    plt::subplot(2, 3, 4);
    plt::named_plot("Lidar omg x", Lidar_omg[0]);
    plt::named_plot("Lidar omg y", Lidar_omg[1]);
    plt::named_plot("Lidar omg z", Lidar_omg[2]);
    plt::legend();
    plt::grid(true);

    plt::subplot(2, 3, 5);
    plt::named_plot("Lidar acc x", Lidar_acc[0]);
    plt::named_plot("Lidar acc y", Lidar_acc[1]);
    plt::named_plot("Lidar acc z", Lidar_acc[2]);
    plt::legend();
    plt::grid(true);

    plt::subplot(2, 3, 6);
    plt::named_plot("Lidar ang acc x", Lidar_ang_acc[0]);
    plt::named_plot("Lidar ang acc y", Lidar_ang_acc[1]);
    plt::named_plot("Lidar ang acc z", Lidar_ang_acc[2]);
    plt::legend();
    plt::grid(true);

    plt::show();
    plt::pause(0);
    plt::close();
}
