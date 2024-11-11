cmake_minimum_required(VERSION 3.10)
project(DifferentialMpc)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 寻找依赖库Eigen
find_package(Eigen3 REQUIRED)
# 寻找依赖库osqp和OsqpEigen
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PythonLibs REQUIRED)

# 添加include目录
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${OSQP_INCLUDE_DIRS}
    ${OSQPEIGEN_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${PYTHON_INCLUDE_DIRS}
    /home/hamster/mycode/Differential_MPC_controller/include
    #/usr/include/python3.10
)

# 指定编译输出路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# 指定源文件
set(MPC 
    src/main.cpp
    src//diffmpc.cpp
)

# 添加源码目录
add_executable(diffmpc ${MPC})
# add_executable(mpc_speed_steel_ ${mpc_speed_steel})

# 链接依赖库
target_link_libraries(diffmpc 
    Eigen3::Eigen
    osqp::osqp
    OsqpEigen::OsqpEigen
    ${PYTHON_LIBRARIES}
    python3.10.so
    pthread
    dl
)

# target_link_libraries(testdraw
#     ${PYTHON_LIBRARIES}
#     python3.10.so
#     pthread
#     dl
# )

# target_link_libraries(mpc_speed_steel_
#     Eigen3::Eigen
#     osqp::osqp
#     OsqpEigen::OsqpEigen
#     ${PYTHON_LIBRARIES}
#     python3.10.so
#     pthread
#     dl
# )

# 其它可选项
# 可以在这里定义更多的CMake选项，比如编译标志、调试信息等
