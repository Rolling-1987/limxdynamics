// Copyright information
//
// © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <ostream>
#include <vector>

/** size_t trajectory type. */
using size_array_t = std::vector<size_t>;
/** Array of size_t trajectory type. */
using size_array2_t = std::vector<size_array_t>;

/** Scalar type. */
using scalar_t = double;
/** Scalar trajectory type. */
using scalar_array_t = std::vector<scalar_t>;
/** Array of scalar trajectory type. */
using scalar_array2_t = std::vector<scalar_array_t>;
/** Array of arrays of scalar trajectory type. */
using scalar_array3_t = std::vector<scalar_array2_t>;

/** Dynamic-size vector type. */
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using vector4_t = Eigen::Matrix<scalar_t, 4, 1>;
using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;
/** Dynamic vector's trajectory type. */
using vector_array_t = std::vector<vector_t>;
/** Array of dynamic vector's trajectory type. */
using vector_array2_t = std::vector<vector_array_t>;
/** Array of arrays of dynamic vector trajectory type. */
using vector_array3_t = std::vector<vector_array2_t>;

/** Dynamic-size row vector type. */
using row_vector_t = Eigen::Matrix<scalar_t, 1, Eigen::Dynamic>;

/** Dynamic-size matrix type. */
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using matrix3x_t = Eigen::Matrix<scalar_t, 3, Eigen::Dynamic>;
using matrix6_t = Eigen::Matrix<scalar_t, 6, 6>;
using matrix6x_t = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;
/** Dynamic matrix's trajectory type. */
using matrix_array_t = std::vector<matrix_t>;
/** Array of dynamic matrix's trajectory type. */
using matrix_array2_t = std::vector<matrix_array_t>;
/** Array of arrays of dynamic matrix trajectory type. */
using matrix_array3_t = std::vector<matrix_array2_t>;

template<typename T>
using feet_array_t = std::array<T, 4>;
using contact_flag_t = feet_array_t<bool>;
using quaternion_t = Eigen::Quaternion<scalar_t>;