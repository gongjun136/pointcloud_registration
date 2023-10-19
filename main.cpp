//
// Created by xiang on 2022/7/7.
//

// #include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <chrono>

// 自定义头文件
#include "icp_3d.h"                   // 3D的ICP配准方法
#include "ndt_3d.h"                   // 3D的NDT配准方法
#include "common/point_cloud_utils.h" // 点云处理的通用工具
#include "common/sys_utils.h"         // 系统的通用工具
#include <iomanip>

// 定义命令行参数
// DEFINE_string(source, "./data/ch7/EPFL/kneeling_lady_source.pcd", "第1个点云路径");
// DEFINE_string(target, "./data/ch7/EPFL/kneeling_lady_target.pcd", "第2个点云路径");
// DEFINE_string(ground_truth_file, "./data/ch7/EPFL/kneeling_lady_pose.txt", "真值Pose");

int main(int argc, char **argv)
{
    // 初始化Google日志系统
    google::InitGoogleLogging(argv[0]);
    // FLAGS_stderrthreshold = google::INFO;
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = "/home/gj/c++_study/test/pointcloud_registration/log";

    // EPFL 雕像数据集：./ch7/EPFL/aquarius_{sourcd.pcd, target.pcd}，真值在对应目录的_pose.txt中
    // EPFL 模型比较精细，配准时应该采用较小的栅格
    if (argc != 3)
    {
        std::cerr << "Usage: ./main <source_pcd_file> <target_pcd_file>" << std::endl;
        return -1;
    }

    // std::ifstream fin(FLAGS_ground_truth_file);
    // SE3 gt_pose;
    // if (fin)
    // {
    // double tx = 0, ty = 0, tz = 0, qw = 1, qx = 0, qy = 0, qz = 0;
    //     fin >> tx >> ty >> tz >> qw >> qx >> qy >> qz;
    //     fin.close();
    // gt_pose = SE3(Quatd(qw, qx, qy, qz), Vec3d(tx, ty, tz));
    // }

    sad::CloudPtr source(new sad::PointCloudType), target(new sad::PointCloudType);
    pcl::io::loadPCDFile(argv[1], *source);
    pcl::io::loadPCDFile(argv[2], *target);
    // Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // // 设置x轴的平移为1米
    // transform(0, 3) = 1.0;
    // // 使用transformPointCloud函数
    // pcl::transformPointCloud(*target, *target, transform);
    // sad::SaveCloudToFile("/home/gj/c++_study/test/pointcloud_registration/output/target.pcd", *target);

    bool success;

    sad::evaluate_and_call(
        [&]()
        {
            sad::Icp3d icp;
            icp.SetSource(source);
            icp.SetTarget(target);
            // icp.SetGroundTruth(gt_pose);
            SE3 pose;
            success = icp.AlignP2P(pose);
            if (success)
            {
                LOG(INFO) << "icp p2p align success, pose: " << std::fixed << std::setprecision(8) << pose.so3().unit_quaternion().coeffs().transpose()
                          << ", " << std::fixed << std::setprecision(8) << pose.translation().transpose();
                sad::CloudPtr source_trans(new sad::PointCloudType);
                pcl::transformPointCloud(*source, *source_trans, pose.matrix().cast<float>());
                sad::SaveCloudToFile("/home/gj/c++_study/test/pointcloud_registration/output/icp_trans.pcd", *source_trans);
            }
            else
            {
                LOG(ERROR) << "align failed.";
            }
        },
        "ICP P2P", 1);

    /// 点到面
    sad::evaluate_and_call(
        [&]()
        {
            sad::Icp3d icp;
            icp.SetSource(source);
            icp.SetTarget(target);
            // icp.SetGroundTruth(gt_pose);
            SE3 pose;
            success = icp.AlignP2Plane(pose);
            if (success)
            {
                LOG(INFO) << "icp p2plane align success, pose: " << std::fixed << std::setprecision(8) << pose.so3().unit_quaternion().coeffs().transpose()
                          << ", " << std::fixed << std::setprecision(8) << pose.translation().transpose();
                sad::CloudPtr source_trans(new sad::PointCloudType);
                pcl::transformPointCloud(*source, *source_trans, pose.matrix().cast<float>());
                sad::SaveCloudToFile("/home/gj/c++_study/test/pointcloud_registration/output/icp_plane_trans.pcd", *source_trans);
            }
            else
            {
                LOG(ERROR) << "align failed.";
            }
        },
        "ICP P2Plane", 1);

    /// 点到线
    sad::evaluate_and_call(
        [&]()
        {
            sad::Icp3d icp;
            icp.SetSource(source);
            icp.SetTarget(target);
            // icp.SetGroundTruth(gt_pose);
            SE3 pose;
            success = icp.AlignP2Line(pose);
            if (success)
            {

                LOG(INFO) << "icp p2line align success, pose: " << std::fixed << std::setprecision(8) << pose.so3().unit_quaternion().coeffs().transpose()
                          << ", " << std::fixed << std::setprecision(8) << pose.translation().transpose();
                sad::CloudPtr source_trans(new sad::PointCloudType);
                pcl::transformPointCloud(*source, *source_trans, pose.matrix().cast<float>());
                sad::SaveCloudToFile("/home/gj/c++_study/test/pointcloud_registration/output/icp_line_trans.pcd", *source_trans);
            }
            else
            {
                LOG(ERROR) << "align failed.";
            }
        },
        "ICP P2Line", 1);

    /// 第７章的NDT
    sad::evaluate_and_call(
        [&]()
        {
            sad::Ndt3d::Options options;
            options.voxel_size_ = 0.5;
            options.remove_centroid_ = true;
            options.nearby_type_ = sad::Ndt3d::NearbyType::CENTER;
            sad::Ndt3d ndt(options);
            ndt.SetSource(source);
            ndt.SetTarget(target);
            // ndt.SetGtPose(gt_pose);
            SE3 pose;
            success = ndt.AlignNdt(pose);
            if (success)
            {
                // LOG(INFO) << "ndt align success, pose: " << pose.so3().unit_quaternion().coeffs().transpose() << ", "
                //           << pose.translation().transpose();
                LOG(INFO) << "ndt align success, pose: " << std::fixed << std::setprecision(8) << pose.so3().unit_quaternion().coeffs().transpose()
                          << ", " << std::fixed << std::setprecision(8) << pose.translation().transpose();
                sad::CloudPtr source_trans(new sad::PointCloudType);
                pcl::transformPointCloud(*source, *source_trans, pose.matrix().cast<float>());
                sad::SaveCloudToFile("/home/gj/c++_study/test/pointcloud_registration/output/ndt_trans.pcd", *source_trans);
            }
            else
            {
                LOG(ERROR) << "align failed.";
            }
        },
        "NDT", 1);

    /// PCL ICP 作为备选
    sad::evaluate_and_call(
        [&]()
        {
            pcl::IterativeClosestPoint<sad::PointType, sad::PointType> icp_pcl;
            icp_pcl.setInputSource(source);
            icp_pcl.setInputTarget(target);
            sad::CloudPtr output_pcl(new sad::PointCloudType);
            icp_pcl.align(*output_pcl);
            SE3f T = SE3f(icp_pcl.getFinalTransformation());
            LOG(INFO) << "pose from icp pcl: " << std::fixed << std::setprecision(8) << T.so3().unit_quaternion().coeffs().transpose() << ", "
                      << std::fixed << std::setprecision(8) << T.translation().transpose();
            sad::SaveCloudToFile("/home/gj/c++_study/test/pointcloud_registration/output/pcl_icp_trans.pcd", *output_pcl);

            // 计算GT pose差异
            // double pose_error = (gt_pose.inverse() * T.cast<double>()).log().norm();
            // LOG(INFO) << "ICP PCL pose error: " << pose_error;
        },
        "ICP PCL", 1);

    /// PCL NDT 作为备选
    sad::evaluate_and_call(
        [&]()
        {
            pcl::NormalDistributionsTransform<sad::PointType, sad::PointType> ndt_pcl;
            ndt_pcl.setInputSource(source);
            ndt_pcl.setInputTarget(target);
            ndt_pcl.setResolution(0.5);
            sad::CloudPtr output_pcl(new sad::PointCloudType);
            ndt_pcl.align(*output_pcl);
            SE3f T = SE3f(ndt_pcl.getFinalTransformation());
            LOG(INFO) << "pose from ndt pcl: " << std::fixed << std::setprecision(8) << T.so3().unit_quaternion().coeffs().transpose() << ", "
                      << T.translation().transpose() << ', trans: ' << std::fixed << std::setprecision(8) << ndt_pcl.getTransformationProbability();
            sad::SaveCloudToFile("/home/gj/c++_study/test/pointcloud_registration/output/pcl_ndt_trans.pcd", *output_pcl);
            LOG(INFO) << "score: " << ndt_pcl.getTransformationProbability();

            // 计算GT pose差异
            // double pose_error = (gt_pose.inverse() * T.cast<double>()).log().norm();
            // LOG(INFO) << "NDT PCL pose error: " << pose_error;
        },
        "NDT PCL", 1);

    return 0;
}