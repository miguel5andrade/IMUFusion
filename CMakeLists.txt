cmake_minimum_required(VERSION 3.10)
project(IMUFusion)

find_package(Eigen3 REQUIRED)

add_executable(imu_fusion main.cpp IMUFusion.cpp)
target_link_libraries(imu_fusion Eigen3::Eigen)
