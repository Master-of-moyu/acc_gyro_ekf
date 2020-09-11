#pragma once

#include <Eigen/Eigen>

template <int Rows = Eigen::Dynamic, int Cols = Rows, bool UseRowMajor = false, typename T = double>
using matrix = typename std::conditional<
    Rows != 1 && Cols != 1,
    Eigen::Matrix<T, Rows, Cols, UseRowMajor ? Eigen::RowMajor : Eigen::ColMajor>,
    Eigen::Matrix<T, Rows, Cols>>::type;

template <int Dimension = Eigen::Dynamic, bool RowVector = false, typename T = double>
using vector = typename std::conditional<
    RowVector,
    matrix<1, Dimension, false, T>,
    matrix<Dimension, 1, false, T>>::type;

using quaternion = Eigen::Quaternion<double>;

struct Pose {
    quaternion q = quaternion::Identity();
    vector<3> p = vector<3>::Zero();
};

struct GyroscopeData {
    double t = -1.0;
    vector<3> g = vector<3>::Zero();
};

struct AccelerometerData {
    double t = -1.0;
    vector<3> a = vector<3>::Zero();
};
