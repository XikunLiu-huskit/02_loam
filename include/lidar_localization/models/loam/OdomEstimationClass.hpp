//
// Created by xkhuskit on 06.02.21.
//

#ifndef LIDAR_LOCALIZATION_FLOAM_FACTOR_HPP
#define LIDAR_LOCALIZATION_FLOAM_FACTOR_HPP

//std lib
#include <string>
#include <math.h>
#include <vector>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

//LOCAL LIB
#include <ros/ros.h>
#include "floam_opt.h"


class OdomEstimationClass
{
public:
    Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
    //optimization variable
    double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
    Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
    Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

    Eigen::Isometry3d last_odom = Eigen::Isometry3d::Identity();

//private:
    //optimization count
    //int optimization_count;
};


#endif //LIDAR_LOCALIZATION_FLOAM_FACTOR_HPP
