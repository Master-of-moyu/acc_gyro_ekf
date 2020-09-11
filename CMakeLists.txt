# Find Pangolin (https://github.com/stevenlovegrove/Pangolin)
find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

add_executable(main_ekf)
target_sources(main_ekf
    PRIVATE
        main_ekf.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter/gyroscope_bias_estimator.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter/lowpass_filter.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter/matrix_3x3.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter/matrix_4x4.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter/matrixutils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter/mean_filter.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter/median_filter.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter/pose_prediction.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter/rotation.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter/sensor_fusion_ekf.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter/vectorutils.cpp
)
target_link_libraries(main_ekf
    PRIVATE
        ${Pangolin_LIBRARIES}
)
