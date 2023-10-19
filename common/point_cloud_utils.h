/*
 * @Author: gongjun136 gongjun136@gmail.com
 * @Date: 2023-10-18 17:57:18
 * @LastEditors: gongjun136 gongjun136@gmail.com
 * @LastEditTime: 2023-10-19 09:57:50
 * @FilePath: /test/pointcloud_registration/common/point_cloud_utils.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
//
// Created by xiang on 2021/8/25.
//

#ifndef SLAM_IN_AUTO_DRIVING_POINT_CLOUD_UTILS_H
#define SLAM_IN_AUTO_DRIVING_POINT_CLOUD_UTILS_H

#include "../common/point_types.h"

/// 点云的一些工具函数

namespace sad {

/// 体素滤波
void VoxelGrid(CloudPtr cloud, float voxel_size = 0.05);

/// 移除地面
void RemoveGround(CloudPtr cloud, float z_min = 0.5);

/// 写点云文件
template<typename CloudType> 
void SaveCloudToFile(const std::string &filePath, CloudType &cloud);

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_POINT_CLOUD_UTILS_H
