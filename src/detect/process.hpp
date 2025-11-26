#ifndef __PROCESS_HPP__
#define __PROCESS_HPP__

#include <vector>
#include <array>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include "pointpillar/lidar-postprocess.hpp"
#include "common/dtype.hpp"
#include "config.hpp"

namespace detect {

// 判断点是否在长方体内部 
bool point_in_3d_box(float px, float py, float pz, const RangeConfig& cfg);

// 预处理：过滤点云（去除地面点，保留ROI区域内的点）
void pre_processing(std::vector<float> &src, std::vector<float> &points_filtered, const RangeConfig& range_config);

// 计算绕Y轴旋转矩阵（相机坐标系）
void rot_y_matrix(float angle, float R[3][3]);

// 计算绕Z轴旋转矩阵（雷达坐标系，在XY平面内旋转）
void rot_z_matrix(float angle, float R[3][3]);

// 计算最小外接贴地长方体（雷达坐标系，绕Z轴旋转）
bool compute_min_box_with_rot(const std::vector<nvtype::Float3> &points_in_box, 
                               float rot_y,
                               float &height, float &width, float &length,
                               float &pos_x, float &pos_y, float &pos_z);

// 校准3D框：获取最小外接贴地长方体
void calib_3d_box(const std::vector<float> &points_filtered, 
                  pointpillar::lidar::BoundingBox &box);

// 后处理：过滤和校准检测框
void post_processing(const std::vector<pointpillar::lidar::BoundingBox> &bboxes,
                     std::vector<pointpillar::lidar::BoundingBox> &bboxes_result,
                     const std::vector<float> &points_filtered);

} // namespace detect

#endif // __PROCESS_HPP__
