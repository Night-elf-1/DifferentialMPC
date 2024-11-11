#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include "cubic_spline.hpp"
#include "OsqpEigen/OsqpEigen.h"
#include "eigen3/Eigen/Core"

static inline bool finish = true;

struct parameters {
    int L = 3.0;
    int NX = 3, NU = 2, NP = 60, NC = 5;
    double dt = 0.5, row = 10;
};

class KinematicModel_MPC{
public:
    double x, y, yaw, v, L, dt;
public:
    KinematicModel_MPC(double x, double y, double psi, double v, double L, double dt) : x(x), y(y), yaw(yaw), v(v), L(L), dt(dt){};
    ~KinematicModel_MPC(){};
    // 更新AGV状态
    void updatestate(double accel, double delta_f);
    // 获取AGV状态
    std::tuple<double, double, double, double> getstate();
};

class diffMpcController {
public:
    int NX, NU, NP, NC;
    Eigen::VectorXd U;
    Eigen::MatrixXd R;
    Eigen::MatrixXd RB;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd QB;


public:
    diffMpcController(int nx, int nu, int np, int nc) : NX(nx), NU(nu), NP(np), NC(nc), R(Eigen::MatrixXd::Identity(nu, nu)), RB(Eigen::MatrixXd::Identity(nc * nu, nc * nu)), Q(Eigen::MatrixXd::Identity(nx, nx)), QB(100*Eigen::MatrixXd::Identity(np * nx, np * nx)), U(Eigen::VectorXd::Constant(nu, 0.01)) {};
    ~diffMpcController(){};
    // 计算参考点的速度
    std::vector<double> calculateReferenceSpeeds(const std::vector<double>& curvatures, const double& max_speed);
    // 平缓航向角
    void smooth_yaw(vector<double>& cyaw);
    // 计算最近点
    std::tuple<int, double> calc_ref_trajectory(double current_x, double current_y, vector<double> cx, vector<double> cy, vector<double> cyaw);
    // MPC控制器
    std::tuple<double, double> mpc_solve(vector<double>& cx, vector<double>& cy, vector<double>& cyaw, vector<double>& ck, vector<double>& speed, Eigen::Vector3d inital_x, int min_index, double min_errors, KinematicModel_MPC agv_model, parameters params_);

};
