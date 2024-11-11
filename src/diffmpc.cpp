#include "diffmpc.hpp"

std::vector<double> diffMpcController::calculateReferenceSpeeds(const std::vector<double>& curvatures, const double& max_speed){
    std::vector<double> referenceSpeeds;
    // double max_speed = 2.0; // 最大速度设为2.0 m/s
    for (double k : curvatures) {
        // 假设曲率半径与速度成线性关系，曲率越大，速度越低
        double speed = max_speed * (1 - 2.5*k); // 假设最大曲率对应于2π的曲率半径
        speed = std::max(1.0, std::min(max_speed, speed)); // 限制速度在0到max_speed之间
        referenceSpeeds.push_back(speed);
    }
    return referenceSpeeds;
}

void diffMpcController::smooth_yaw(vector<double>& cyaw){
    for(int i=0; i<cyaw.size()-1; i++){
        double dyaw = cyaw[i+1] - cyaw[i];

        while (dyaw > M_PI/2.0){
            cyaw[i+1] -= M_PI*2.0;
            dyaw = cyaw[i+1] - cyaw[i];
        }
        while (dyaw < -M_PI/2.0){
            cyaw[i+1] += M_PI*2.0;
            dyaw = cyaw[i+1] - cyaw[i];
        }
    }
}

std::tuple<int, double> calc_nearest_index(double current_x, double current_y, vector<double> cx, vector<double> cy, vector<double> cyaw){
    double mind = numeric_limits<double>::max();        // 初始化一个变量 mind，用于记录最小的距离平方值
    double ind = 0;                                     // 初始化索引 ind，用于存储找到的最近轨迹点的索引
    
    for (int i = 0; i < cx.size(); i++)
    {
        double idx = current_x - cx[i];
        double idy = current_y - cy[i];
        double d_e = std::sqrt(idx*idx + idy*idy);

        if (d_e < mind)
        {
            mind = d_e;
            ind = i;
        }
        
    }
    
    return std::make_tuple(ind, mind);
}

std::tuple<int, double> diffMpcController::calc_ref_trajectory(double current_x, double current_y, vector<double> cx, vector<double> cy, vector<double> cyaw){
    auto [ind, d_e] = calc_nearest_index(current_x, current_y, cx, cy, cyaw);

    return std::make_tuple(ind, d_e);
}

std::tuple<double, double> diffMpcController::mpc_solve(vector<double>& cx, vector<double>& cy, vector<double>& cyaw, vector<double>& ck, vector<double>& speed, Eigen::Vector3d inital_x, int min_index, double min_errors, KinematicModel_MPC agv_model, parameters params_){
    const double row = 10;
    Eigen::Vector2d u_min(-0.01,-0.52333);
    Eigen::Vector2d u_max(0.01,0.52333);
    Eigen::Vector2d delta_umin(-0.01, -0.35);
    Eigen::Vector2d delta_umax(0.01, 0.35);
    // std::cout << "u_min = " << u_min 
    //           << "  u_max = " << u_max 
    //           << "  delta_umin = " << delta_umin 
    //           << "  delta_umax = " << delta_umax 
    //           << std::endl;

    double yaw_r = cyaw[min_index];
    double v_r = speed[min_index];
    // std::cout << "(1)v_r = " << v_r << std::endl;
    double k_r = ck[min_index];
    // double lat_error = min_errors;
    double delta_f_r = atan2(3.7 * k_r, 1);
    // std::cout << "yaw_r = " << yaw_r << "  v_r = " << v_r << "  k_r = " << k_r << "  lat_error = " << lat_error << "  delta_f_r = " << delta_f_r << std::endl;

    Eigen::Matrix3d Ad(3, 3);                               // Ad矩阵
    Ad << 1, 0, (-1*v_r) * sin(yaw_r) * params_.dt,
         0, 1, v_r * cos(yaw_r) * params_.dt,
         0, 0, 1;
    // std::cout << "Ad = " << std::endl;
    // std::cout << Ad << std::endl;

    Eigen::MatrixXd Bd(3, 2);       // Bd矩阵
    Bd << cos(yaw_r) * params_.dt, 0,
          sin(yaw_r) * params_.dt, 0,
          0, params_.dt;
    // std::cout << "Bd = " << std::endl;
    // std::cout << Bd << std::endl;

    // 状态空间方程的相关矩阵
    Eigen::VectorXd kesi(NX + NU);             // 新状态变量kesi  3 + 2
    Eigen::Vector3d x_r(cx[min_index], cy[min_index], yaw_r);
    // std::cout << "x_r = " << x_r << std::endl;
    kesi.head(NX) = inital_x - x_r;
    kesi.tail(NU) = U;                          // U为初始控制量

    Eigen::MatrixXd A_3 = Eigen::MatrixXd::Zero(NX + NU, NX + NU);              // A矩阵为A3矩阵
    A_3.topLeftCorner(NX, NX) = Ad;
    A_3.topRightCorner(NX, NU) = Bd;
    A_3.bottomRightCorner(NU, NU) = Eigen::MatrixXd::Identity(NU, NU);       // 往A3矩阵中添加值
    // std::cout << "A_3 = " << std::endl;
    // std::cout << A_3 << std::endl;

    Eigen::MatrixXd B_3 = Eigen::MatrixXd::Zero(NX + NU, NU);                       // B为B3矩阵
    B_3.topLeftCorner(NX, NU) = Bd;                                    // 往B3矩阵里面添加Bd矩阵和I矩阵(单位矩阵)
    B_3.bottomRightCorner(NU, NU) = Eigen::MatrixXd::Identity(NU, NU);
    // std::cout << "B_3 = " << std::endl;
    // std::cout << B_3 << std::endl;

    // C 矩阵
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(NX, NX + NU);               // 设置C矩阵
    C.topLeftCorner(NX, NX) = Eigen::MatrixXd::Identity(NX, NX);          // C=[E 0]
    // std::cout << "C = " << std::endl;
    // std::cout << C << std::endl;

    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(NP * NX, NX + NU);       // PHI为W矩阵 
    for (int i = 0; i < NP; ++i) {
        Eigen::MatrixXd result = A_3;
        // int count = 1;
        //Eigen::MatrixXd result = Eigen::MatrixXd::Identity(NX + NU, NX + NU);
        for (int j = 0; j < i; ++j)
        {
            result = result * A_3;
            // count += 1;
        }
        
        W.middleRows(i * NX, NX) = C * result;
        // std::cout << "count = " << count << std::endl;
        //W.middleRows(i * NX, NX) = C * result.topLeftCorner(NX, NX + NU);
    }
    // std::cout << "W = " << std::endl;
    // std::cout << W << std::endl;

    Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(NP * NX, NC * NU);          // THETA矩阵为Z矩阵
    for (int i = 0; i < NP; ++i) {
        for (int j = 0; j < NC; ++j) {
            if (j <= i) {
                // 计算 A_3^(i-j)
                Eigen::MatrixXd result = Eigen::MatrixXd::Identity(A_3.rows(), A_3.cols()); // 初始化为单位矩阵
                for (int k = 0; k < (i - j); ++k) {
                    result *= A_3; // 逐步累乘A_3
                }

                Z.middleRows(i * NX, NX).middleCols(j * NU, NU) = C * result * B_3;
            }
        }
    }
    // std::cout << "z = " << std::endl;
    // std::cout << Z << std::endl;

    // H 矩阵
    Eigen::MatrixXd H = Z.transpose() * QB * Z + RB;
    Eigen::VectorXd g = kesi.transpose() * W.transpose() * QB * Z;
    // std::cout << "H = " << std::endl;
    // std::cout << H << std::endl;
    // std::cout << "g = " << std::endl;
    // std::cout << g << std::endl;

    // 约束
    Eigen::MatrixXd A_e = Eigen::MatrixXd::Zero(NC * NU, NC * NU);            // A_I对应Ae矩阵
    // for (int i = 0; i < NC; ++i) {
    //     A_e.block(i * NU, 0, NU, (i + 1) * NU) = Eigen::MatrixXd::Identity(NU, NU*(i+1));
    //     //cout << 1 << endl;
    // }
    for (int i = 0; i < NC; ++i) {
        for (int j = 0; j <= i; ++j) {
            A_e.block(i * NU, j * NU, NU, NU) = Eigen::MatrixXd::Identity(NU, NU);
        }
    }
    // std::cout << "A_e" << std::endl;
    // std::cout << A_e << std::endl;

    Eigen::VectorXd U_t = Eigen::VectorXd::Zero(NC * NU);
    for (int i = 0; i < NC; ++i) {
        U_t.segment(i * NU, NU) = U; // 复制 U 到 U_t 的每个区段
    }
    // std::cout << "u_t = " << std::endl;
    // std::cout << U_t << std::endl;

    Eigen::VectorXd Umax = Eigen::VectorXd::Ones(NC * NU);
    for (int i = 0; i < NC; ++i) {
        Umax.segment(i * NU, NU) = u_max;
    }
    // std::cout << "umax = " << std::endl;
    // std::cout << Umax << std::endl;

    Eigen::VectorXd Umin = Eigen::VectorXd::Ones(NC * NU);
    for (int i = 0; i < NC; ++i) {
        Umin.segment(i * NU, NU) = u_min;
    }
    // std::cout << "umin = " << std::endl;
    // std::cout << Umin << std::endl;

    Eigen::VectorXd delta_Umin = Eigen::VectorXd::Ones(NC * NU);
    for (int i = 0; i < NC; ++i) {
        delta_Umin.segment(i * NU, NU) = delta_umin;
    }
    // std::cout << "delta_umin = " << std::endl;
    // std::cout << delta_Umin << std::endl;

    Eigen::VectorXd delta_Umax = Eigen::VectorXd::Ones(NC * NU);
    for (int i = 0; i < NC; ++i) {
        delta_Umax.segment(i * NU, NU) = delta_umax;
    }
    // std::cout << "delta_umax = " << std::endl;
    // std::cout << delta_Umax << std::endl;

    OsqpEigen::Solver solver;
    int num_variables = H.rows();
    int num_constraints = A_e.rows();

    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(num_variables);
    solver.data()->setNumberOfConstraints(num_constraints);

    // 设置二次规划问题的矩阵和向量
    Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    solver.data()->setHessianMatrix(H_sparse);  // H为稀疏矩阵
    solver.data()->setGradient(g);                   // 线性项 g

    // 设置约束的矩阵和边界
    Eigen::SparseMatrix<double> A_e_sparse = A_e.sparseView();
    solver.data()->setLinearConstraintsMatrix(A_e_sparse);  // A为稀疏矩阵
    solver.data()->setLowerBound(delta_Umin);                    // 下界
    solver.data()->setUpperBound(delta_Umax);                    // 上界

    // 初始化并求解问题
    if (!solver.initSolver()) {
        throw std::runtime_error("Solver initialization failed");
    }

    solver.solveProblem();

    // 获取求解结果
    Eigen::VectorXd solution = solver.getSolution();
    // 更新控制量U
    Eigen::VectorXd delta_U = solution.head(U.size());
    U += delta_U;

    // 计算实际的控制量
    double v_real = U(0) + v_r;
    double delta_real = U(1);

    return std::make_tuple(v_real, delta_real);
}

void KinematicModel_MPC::updatestate(double accel, double delta_f){
    // 计算转弯半径
    // double R = L / (2*tan(delta_f));
    // 计算角速度 omega
    // double omega = accel / R;
    double omega = delta_f;
    // 转弯半径
    double R = accel / omega;
    // 计算左右轮速度
    double v_left = omega * (R - L/2);
    double v_right = omega * (R + L/2);
    std::cout << "左轮轮速 = " << v_left << "   右轮轮速 = " << v_right << std::endl;
    // 更新状态
    x += (accel / omega) * (sin(yaw + omega * dt) - sin(yaw));
    y += (accel / omega) * (-cos(yaw + omega * dt) + cos(yaw));
    yaw += omega * dt;

    // if (abs(delta_f < 0.0001)) {
    //     x += accel * cos(yaw) * dt;
    //     y += accel * sin(yaw) * dt;
    // }else {
    //     x += (accel / omega) * (sin(yaw + omega * dt) - sin(yaw));
    //     y += (accel / omega) * (-cos(yaw + omega * dt) + cos(yaw));
    //     yaw += omega * dt;
    // }
}

std::tuple<double, double, double, double> KinematicModel_MPC::getstate(){
    return std::make_tuple(x, y, yaw, v);
}
