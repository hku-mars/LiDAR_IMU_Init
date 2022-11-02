#ifndef LI_INIT_H
#define LI_INIT_H

#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <csignal>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <condition_variable>
#include <eigen_conversions/eigen_msg.h>
#include <common_lib.h>
#include <algorithm>
#include <ceres/ceres.h>
#include <sys/time.h>
#include "matplotlibcpp.h"

#define FILE_DIR(name)     (string(string(ROOT_DIR) + "Log/"+ name))

namespace plt = matplotlibcpp;
using namespace std;
using namespace Eigen;

typedef Vector3d V3D;
typedef Matrix3d M3D;
const V3D STD_GRAV = V3D(0, 0, -G_m_s2);

// Lidar Inertial Initialization
// States needed by Initialization
struct CalibState {
    M3D rot_end;
    V3D ang_vel;
    V3D linear_vel;
    V3D ang_acc;
    V3D linear_acc;
    double timeStamp;

    CalibState() {
        rot_end = Eye3d;
        ang_vel = Zero3d;
        linear_vel = Zero3d;
        ang_acc = Zero3d;
        linear_acc = Zero3d;
        timeStamp = 0.0;
    };

    CalibState(const CalibState &b) {
        this->rot_end = b.rot_end;
        this->ang_vel = b.ang_vel;
        this->ang_acc = b.ang_acc;
        this->linear_vel = b.linear_vel;
        this->linear_acc = b.linear_acc;
        this->timeStamp = b.timeStamp;
    };

    CalibState operator*(const double &coeff) {
        CalibState a;
        a.ang_vel = this->ang_vel * coeff;
        a.ang_acc = this->ang_acc * coeff;
        a.linear_vel = this->linear_vel * coeff;
        a.linear_acc = this->linear_acc * coeff;
        return a;
    };

    CalibState &operator+=(const CalibState &b) {
        this->ang_vel += b.ang_vel;
        this->ang_acc += b.ang_acc;
        this->linear_vel += b.linear_vel;
        this->linear_acc += b.linear_acc;
        return *this;
    };

    CalibState &operator-=(const CalibState &b) {
        this->ang_vel -= b.ang_vel;
        this->ang_acc -= b.ang_acc;
        this->linear_vel -= b.linear_vel;
        this->linear_acc -= b.linear_acc;
        return *this;
    };

    CalibState &operator=(const CalibState &b) {
        this->ang_vel = b.ang_vel;
        this->ang_acc = b.ang_acc;
        this->linear_vel = b.linear_vel;
        this->linear_acc = b.linear_acc;
        return *this;
    };
};

struct Angular_Vel_Cost_only_Rot {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Angular_Vel_Cost_only_Rot(V3D IMU_ang_vel_, V3D Lidar_ang_vel_) :
            IMU_ang_vel(IMU_ang_vel_), Lidar_ang_vel(Lidar_ang_vel_) {}

    template<typename T>
    bool operator()(const T *q, T *residual) const {
        Eigen::Matrix<T, 3, 1> IMU_ang_vel_T = IMU_ang_vel.cast<T>();
        Eigen::Matrix<T, 3, 1> Lidar_ang_vel_T = Lidar_ang_vel.cast<T>();
        Eigen::Quaternion<T> q_LI{q[0], q[1], q[2], q[3]};
        Eigen::Matrix<T, 3, 3> R_LI = q_LI.toRotationMatrix();  //Rotation
        Eigen::Matrix<T, 3, 1> resi = R_LI * Lidar_ang_vel_T - IMU_ang_vel_T;
        residual[0] = resi[0];
        residual[1] = resi[1];
        residual[2] = resi[2];
        return true;
    }

    static ceres::CostFunction *Create(const V3D IMU_ang_vel_, const V3D Lidar_ang_vel_) {
        return (new ceres::AutoDiffCostFunction<Angular_Vel_Cost_only_Rot, 3, 4>(
                new Angular_Vel_Cost_only_Rot(IMU_ang_vel_, Lidar_ang_vel_)));
    }

    V3D IMU_ang_vel;
    V3D Lidar_ang_vel;
};

struct Angular_Vel_Cost {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Angular_Vel_Cost(V3D IMU_ang_vel_, V3D IMU_ang_acc_, V3D Lidar_ang_vel_, double deltaT_LI_) :
            IMU_ang_vel(IMU_ang_vel_), IMU_ang_acc(IMU_ang_acc_), Lidar_ang_vel(Lidar_ang_vel_),
            deltaT_LI(deltaT_LI_) {}

    template<typename T>
    bool operator()(const T *q, const T *b_g, const T *t, T *residual) const {
        //Known parameters used for Residual Construction
        Eigen::Matrix<T, 3, 1> IMU_ang_vel_T = IMU_ang_vel.cast<T>();
        Eigen::Matrix<T, 3, 1> IMU_ang_acc_T = IMU_ang_acc.cast<T>();
        Eigen::Matrix<T, 3, 1> Lidar_ang_vel_T = Lidar_ang_vel.cast<T>();
        T deltaT_LI_T{deltaT_LI};

        //Unknown Parameters, needed to be estimated
        Eigen::Quaternion<T> q_LI{q[0], q[1], q[2], q[3]};
        Eigen::Matrix<T, 3, 3> R_LI = q_LI.toRotationMatrix();  //Rotation
        Eigen::Matrix<T, 3, 1> bias_g{b_g[0], b_g[1], b_g[2]};  //Bias of gyroscope
        T td{t[0]};                                             //Time lag (IMU wtr Lidar)

        //Residual
        Eigen::Matrix<T, 3, 1> resi =
                R_LI * Lidar_ang_vel_T - IMU_ang_vel_T - (deltaT_LI_T + td) * IMU_ang_acc_T + bias_g;
        residual[0] = resi[0];
        residual[1] = resi[1];
        residual[2] = resi[2];
        return true;
    }

    static ceres::CostFunction *
    Create(const V3D IMU_ang_vel_, const V3D IMU_ang_acc_, const V3D Lidar_ang_vel_, const double deltaT_LI_) {
        return (new ceres::AutoDiffCostFunction<Angular_Vel_Cost, 3, 4, 3, 1>(
                new Angular_Vel_Cost(IMU_ang_vel_, IMU_ang_acc_, Lidar_ang_vel_, deltaT_LI_)));
    }

    V3D IMU_ang_vel;
    V3D IMU_ang_acc;
    V3D Lidar_ang_vel;
    double deltaT_LI;
};

struct Linear_acc_Cost {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Linear_acc_Cost(CalibState LidarState_, M3D R_LI_, V3D IMU_linear_acc_) :
            LidarState(LidarState_), R_LI(R_LI_), IMU_linear_acc(IMU_linear_acc_) {}

    template<typename T>
    bool operator()(const T *q, const T *b_a, const T *trans, T *residual) const {
        //Known parameters used for Residual Construction
        Eigen::Matrix<T, 3, 3> R_LL0_T = LidarState.rot_end.cast<T>();
        Eigen::Matrix<T, 3, 3> R_LI_T_transpose = R_LI.transpose().cast<T>();
        Eigen::Matrix<T, 3, 1> IMU_linear_acc_T = IMU_linear_acc.cast<T>();
        Eigen::Matrix<T, 3, 1> Lidar_linear_acc_T = LidarState.linear_acc.cast<T>();

        //Unknown Parameters, needed to be estimated
        Eigen::Quaternion<T> q_GL0{q[0], q[1], q[2], q[3]};
        Eigen::Matrix<T, 3, 3> R_GL0 = q_GL0.toRotationMatrix();   //Rotation from Gravitational to First Lidar frame
        Eigen::Matrix<T, 3, 1> bias_aL{b_a[0], b_a[1], b_a[2]};    //Bias of Linear acceleration
        Eigen::Matrix<T, 3, 1> T_IL{trans[0], trans[1], trans[2]}; //Translation of I-L (IMU wtr Lidar)

        //Residual Construction
        M3D Lidar_omg_SKEW, Lidar_angacc_SKEW;
        Lidar_omg_SKEW << SKEW_SYM_MATRX(LidarState.ang_vel);
        Lidar_angacc_SKEW << SKEW_SYM_MATRX(LidarState.ang_acc);
        M3D Jacob_trans = Lidar_omg_SKEW * Lidar_omg_SKEW + Lidar_angacc_SKEW;
        Eigen::Matrix<T, 3, 3> Jacob_trans_T = Jacob_trans.cast<T>();

        Eigen::Matrix<T, 3, 1> resi = R_LL0_T * R_LI_T_transpose * IMU_linear_acc_T - R_LL0_T * bias_aL
                                      + R_GL0 * STD_GRAV - Lidar_linear_acc_T - R_LL0_T * Jacob_trans_T * T_IL;

        residual[0] = resi[0];
        residual[1] = resi[1];
        residual[2] = resi[2];
        return true;
    }

    static ceres::CostFunction *Create(const CalibState LidarState_, const M3D R_LI_, const V3D IMU_linear_acc_) {
        return (new ceres::AutoDiffCostFunction<Linear_acc_Cost, 3, 4, 3, 3>(
                new Linear_acc_Cost(LidarState_, R_LI_, IMU_linear_acc_)));
    }

    CalibState LidarState;
    M3D R_LI;
    V3D IMU_linear_acc;
};


class LI_Init {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ofstream fout_LiDAR_meas, fout_IMU_meas, fout_before_filt_IMU, fout_before_filt_Lidar, fout_acc_cost, fout_after_rot;
    double data_accum_length;

    LI_Init();

    ~LI_Init();

    struct Butterworth {
        //Coefficients of 6 order butterworth low pass filter, omega = 0.15
        double Coeff_b[7] = {0.000076, 0.000457, 0.001143, 0.001524, 0.0011, 0.000457, 0.000076};
        double Coeff_a[7] = {1.0000, -4.182389, 7.491611, -7.313596, 4.089349, -1.238525, 0.158428};
        int Coeff_size = 7;
        int extend_num = 0;
    };


    void plot_result();

    void push_ALL_IMU_CalibState(const sensor_msgs::Imu::ConstPtr &msg, const double &mean_acc_norm);

    void push_IMU_CalibState(const V3D &omg, const V3D &acc, const double &timestamp);

    void push_Lidar_CalibState(const M3D &rot, const V3D &omg, const V3D &linear_vel, const double &timestamp);

    void downsample_interpolate_IMU(const double &move_start_time);

    void central_diff();

    void xcorr_temporal_init(const double &odom_freq);

    void IMU_time_compensate(const double &lag_time, const bool &is_discard);

    void acc_interpolate();

    void Butter_filt(const deque<CalibState> &signal_in, deque<CalibState> &signal_out);

    void zero_phase_filt(const deque<CalibState> &signal_in, deque<CalibState> &signal_out);

    void cut_sequence_tail();

    void set_IMU_state(const deque<CalibState> &IMU_states);

    void set_Lidar_state(const deque<CalibState> &Lidar_states);

    void set_states_2nd_filter(const deque<CalibState> &IMU_states, const deque<CalibState> &Lidar_states);

    void normalize_acc(deque<CalibState> &signal_in);

    bool data_sufficiency_assess(MatrixXd &Jacobian_rot, int &frame_num, V3D &lidar_omg, int &orig_odom_freq,
                                 int &cut_frame_num);

    void solve_Rotation_only();

    void solve_Rot_bias_gyro(double &timediff_imu_wrt_lidar);

    void solve_trans_biasacc_grav();

    void LI_Initialization(int &orig_odom_freq, int &cut_frame_num, double &timediff_imu_wrt_lidar,
                           const double &move_start_time);

    void printProgress(double percentage, int axis);

    void clear();

    void fout_before_filter();

    void print_initialization_result(double &time_L_I, M3D &R_L_I, V3D &p_L_I, V3D &bias_g, V3D &bias_a, V3D gravity);

    inline double get_lag_time_1() {
        return time_lag_1;
    }

    inline double get_lag_time_2() {
        return time_lag_2;
    }

    inline double get_total_time_lag() {
        return time_delay_IMU_wtr_Lidar;
    }

    inline V3D get_Grav_L0() {
        return Grav_L0;
    }

    inline M3D get_R_LI() {
        return Rot_Lidar_wrt_IMU;
    }

    inline V3D get_T_LI() {
        return Trans_Lidar_wrt_IMU;
    }

    inline V3D get_gyro_bias() {
        return gyro_bias;
    }

    inline V3D get_acc_bias() {
        return acc_bias;
    }

    inline void IMU_buffer_clear() {
        IMU_state_group_ALL.clear();
    }

    deque<CalibState> get_IMU_state() {
        return IMU_state_group;
    }

    deque<CalibState> get_Lidar_state() {
        return Lidar_state_group;
    }

    void IMU_state_group_ALL_pop_front(){
        IMU_state_group_ALL.pop_front();
    }

    void Lidar_state_group_pop_front(){
        Lidar_state_group.pop_front();
    }

    int IMU_state_group_ALL_size(){
        return IMU_state_group_ALL.size();
    }

    int Lidar_state_group_size(){
        return Lidar_state_group.size();
    }

private:
    deque<CalibState> IMU_state_group;
    deque<CalibState> Lidar_state_group;
    deque<CalibState> IMU_state_group_ALL;


    /// Parameters needed to be calibrated
    M3D Rot_Grav_wrt_Init_Lidar;  // Rotation from inertial frame G to initial Lidar frame L_0
    V3D Grav_L0;                  // Gravity vector in the initial Lidar frame L_0
    M3D Rot_Lidar_wrt_IMU;        // Rotation from Lidar frame L to IMU frame I
    V3D Trans_Lidar_wrt_IMU;      // Translation from Lidar frame L to IMU frame I
    V3D gyro_bias;                // gyro bias
    V3D acc_bias;                 // acc bias
    double time_delay_IMU_wtr_Lidar; //(Soft) time delay between IMU and Lidar = time_lag_1 + time_lag_2
    double time_lag_1;            //Time offset estimated by cross-correlation
    double time_lag_2;            //Time offset estimated by unified optimization
    int lag_IMU_wtr_Lidar;        //positive: timestamp of IMU is larger than that of LiDAR
};

#endif