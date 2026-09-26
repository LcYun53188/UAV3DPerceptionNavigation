cmake_minimum_required(VERSION 3.16)
project(ego_planner_vendor)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
set(EGO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.deps/ego_planner_src" CACHE PATH "Prepared EGO checkout")
if(NOT EXISTS "${EGO_ROOT}/src/planner/bspline_opt/src/bspline_optimizer.cpp")
  message(FATAL_ERROR "Run scripts/prepare_ego_vendor.sh before building")
endif()
set(P "${EGO_ROOT}/src/planner")
add_library(ego_core SHARED
  ${P}/path_searching/src/dyn_a_star.cpp
  ${P}/bspline_opt/src/bspline_optimizer.cpp
  ${P}/bspline_opt/src/uniform_bspline.cpp
  ${P}/bspline_opt/src/gradient_descent_optimizer.cpp
  ${P}/traj_utils/src/polynomial_traj.cpp
  ${P}/plan_env/src/obj_predictor.cpp)
target_compile_features(ego_core PUBLIC cxx_std_17)
target_compile_options(ego_core PRIVATE -O2)
target_link_libraries(ego_core Eigen3::Eigen)
ament_target_dependencies(ego_core rclcpp geometry_msgs visualization_msgs)
foreach(part plan_env path_searching bspline_opt traj_utils)
  target_include_directories(ego_core PUBLIC $<BUILD_INTERFACE:${P}/${part}/include> $<INSTALL_INTERFACE:include>)
  install(DIRECTORY ${P}/${part}/include/ DESTINATION include)
endforeach()
install(TARGETS ego_core EXPORT ego_core_targets LIBRARY DESTINATION lib)
install(FILES ${EGO_ROOT}/LICENSE DESTINATION share/${PROJECT_NAME})
ament_export_targets(ego_core_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs visualization_msgs Eigen3)
ament_package()
