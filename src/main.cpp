#include "diffmpc.hpp"
#include <chrono>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace std;

int main(int argc, char const *argv[])
{
    //生成参考路线
    vector<double> wx({10.0, 60.0, 125.0,  50.0,   60.0,  35.0,  -10.0});
    vector<double> wy({0.0,  0.0,  50.0,  65.0,   45.0,  50.0,  -20.0});

    Spline2D csp_obj(wx, wy);
    vector<double> r_x;
    vector<double> r_y;
    vector<double> ryaw;                                                                        // 航向角
    vector<double> rcurvature;                                                                  // 曲率
    vector<double> rs;
    for(double i=0; i<csp_obj.s.back(); i+=1.0){                                                // 计算出路径点X 和 Y
        vector<double> point_= csp_obj.calc_postion(i);
        r_x.push_back(point_[0]);
        r_y.push_back(point_[1]);
        ryaw.push_back(csp_obj.calc_yaw(i));
        rcurvature.push_back(csp_obj.calc_curvature(i));
        rs.push_back(i);
    }
    double target_speed = 1.5;

    // 初始化车辆状态
    parameters param;
    // 初始化MPC状态
    diffMpcController mpc(param.NX, param.NU, param.NP, param.NC);
    // 计算出参考路径的速度信息
    vector<double> speed_profile = mpc.calculateReferenceSpeeds(rcurvature, target_speed);
    // 平缓航向角
    mpc.smooth_yaw(ryaw);
    // 初始化agv初始状态 x y yaw
    Eigen::Vector3d initial_x;
    initial_x << 10.0, 5.0, 0.1;
    // 初始化agv运动学模型状态
    KinematicModel_MPC agv(initial_x(0), initial_x(1), initial_x(2), target_speed, param.L, 0.5);
    // 创建agv历史点容器
    std::vector<double> x_history, y_history;

    // 创建绘图对象
    plt::figure_size(800, 600);
    // 进入仿真循环
    while ( finish )
    {
        // 计算最近点
        auto [min_index, min_e] = mpc.calc_ref_trajectory(initial_x(0), initial_x(1), r_x, r_y, ryaw);
        // std::cout << "最近点索引：" << min_index << " 误差：" << min_e << std::endl;
        // 计算MPC控制
        auto [v_real, omega] = mpc.mpc_solve(r_x, r_y, ryaw, rcurvature, speed_profile, initial_x, min_index, min_e, agv, param);
        // std::cout << "求解速度:" << v_real << " 求解角速度：" << omega << std::endl;
        // 更新agv状态
        agv.updatestate(v_real, omega);
        // 获取agv状态
        auto [current_x,  current_y, current_yaw, current_v] = agv.getstate();
        // 更新当前状态
        initial_x << current_x, current_y, current_yaw;
        // 存储agv历史点
        x_history.push_back(current_x);
        y_history.push_back(current_y);

        plt::clf();                                                             // 清除当前图像
        plt::plot(r_x, r_y, "b--");                                             // 绘制参考路径
        plt::plot(x_history, y_history, "r-");                                  // 绘制AGV的实际轨迹
        plt::scatter(std::vector<double>{initial_x(0)}, std::vector<double>{initial_x(1)}, 20.0, {{"color", "green"}});
        plt::pause(0.01);                                                       // 暂停以更新图形

        // 判断结束
        if ( min_index >= r_x.size() - 15 )
        {
            finish = false;
            std::cout << "航向角误差：" << std::endl;
            std::cout << current_yaw - ryaw[min_index] << std::endl;
            std::cout << "横向误差：" << std::endl;
            std::cout << min_e << std::endl;
        }
    }
    
    // 显示图形
    plt::show();

    return 0;
}

