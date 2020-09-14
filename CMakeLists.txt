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

add_executable(main_ekf_with_eigen)
target_sources(main_ekf_with_eigen
    PRIVATE
        main_ekf_with_eigen.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter_eigen/gyroscope_bias_estimator.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter_eigen/lowpass_filter.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter_eigen/mean_filter.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter_eigen/median_filter.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter_eigen/pose_prediction.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter_eigen/pose_state.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter_eigen/sensor_data.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter_eigen/sensor_fusion_ekf.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter_eigen/gyroscope_bias_estimator.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter_eigen/lowpass_filter.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter_eigen/mean_filter.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter_eigen/median_filter.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter_eigen/pose_prediction.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ekf_filter_eigen/sensor_fusion_ekf.cpp
)
target_link_libraries(main_ekf_with_eigen
    PRIVATE
        ${Pangolin_LIBRARIES}
)

add_executable(pangolin_test1)
target_sources(pangolin_test1
    PRIVATE
        pangolin_test1.cpp
)
target_link_libraries(pangolin_test1
    PRIVATE
        ${Pangolin_LIBRARIES}
)
