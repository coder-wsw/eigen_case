#pragma once
#include "algo_visual.hpp"
#include "com_header.h"
#include "keypoints.hpp"
#include "viewer.hpp"
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/range_image/range_image_spherical.h>

/// 点云转深度图
void range_image(pcl::PointCloud<pcl::PointXYZ>::Ptr& data, Viewer& viewer)
{
    float centroid[3];

    if (nullptr == data) {
        std::cout << "load data error\n";
    }

    int num = std::ceil(data->size() / 30.f);

    if (0 == num) {
        std::cout << "data num is zero" << std::endl;
    }

    int flag = 0;
    for (int i = 0; i < data->size(); i++) {
        if (0 == (i % 30)) {
            centroid[0] += data->points[i].x;
            centroid[1] += data->points[i].y;
            centroid[2] += data->points[i].z;
            if (flag == 0) {
                std::cout << "point:" << data->points[i].x << ','
                          << data->points[i].y << ',' << data->points[i].z
                          << std::endl;
                flag = 1;
            }
        }
    }

    std::cout << "centroid total:" << centroid[0] << ',' << centroid[1] << ','
              << centroid[2] << std::endl;

    centroid[0] /= num;
    centroid[1] /= num;
    centroid[2] /= num;

    std::cout << "centroid:" << centroid[0] << ',' << centroid[1] << ','
              << centroid[2] << std::endl;
    std::cout << "num:" << num << ',' << std::ceil(data->size() / 30.f)
              << std::endl;

    pcl::RangeImage::CoordinateFrame coordinate_frame =
        pcl::RangeImage::CAMERA_FRAME;
    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
    Eigen::Matrix3f ro;
    ro.setIdentity();
#if 1
    ro << 0.f, -1.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, -1.f;
#else
    ro << 1.f, 0.f, 0.f, 0.f, -1.f, 0.f, 0.f, 0.f, -1.f;
#endif
    scene_sensor_pose.translation() << centroid[0], centroid[1],
        centroid[2] + 100.f;
    scene_sensor_pose.rotate(ro);

    matrix2euler(ro, RotationOrder::ZYX);
    Debug(ro);

    // 将scene_sensor_pose转成Eigen::Matrix4f类型
    Eigen::Matrix4f scene_sensor_pose_matrix = scene_sensor_pose.matrix();
    // viewer.visual_coord(scene_sensor_pose_matrix, 1, 2);
    viewer.add_coord(scene_sensor_pose_matrix, 1, 2, 500);
    float rDt_len = 1800;
    int di_width = std::ceil(rDt_len * 1.2);
    int di_height = 1000;
    float di_center_x = di_width * 0.5f;
    float di_center_y = di_height * 0.5f;
    float di_focal_length_x = 200.f;
    float di_focal_length_y = 200.f;
    float noise_level = 0.0;
    float min_range = 0.0f;

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter1(
        new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(data);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(centroid[2] - 50.f, centroid[2] + 50.f);
    pass.filter(*filter1);

    viewer.show_cloud(filter1, 2, 2);
    pcl::RangeImagePlanar range_image;
    range_image.createFromPointCloudWithFixedSize(
        *filter1, di_width, di_height, di_center_x, di_center_y,
        di_focal_length_x, di_focal_length_y, scene_sensor_pose,
        coordinate_frame, noise_level, min_range);

    // viewer.show_range_image(range_image, 3);

    // 用opencv显示深度图
    cv::Mat* img = new cv::Mat(range_image.height, range_image.width, CV_32FC1,
                               range_image.getRangesArray());

    // 转为3通道图像
    cv::Mat img3;
    cv::cvtColor(*img, img3, cv::COLOR_GRAY2BGR);

    extract_narf_keypoint(range_image, viewer, img3);

    cv::imshow("depth image", img3);
    cv::waitKey(0);
}

/// 点云转深度图2
void range_image2(pcl::PointCloud<pcl::PointXYZ>::Ptr& data, Viewer& viewer)
{

    Eigen::Affine3f sensorPose = Eigen::Affine3f::Identity();
    sensorPose.translation() << 0, 0, 0;

    float noiseLevel = 0.0;
    float minRange = 0.0f;

    pcl::RangeImage::CoordinateFrame coordinate_frame =
        pcl::RangeImage::CAMERA_FRAME;

    // pcl::RangeImage rangeImage;
    pcl::RangeImagePlanar rangeImage;
    rangeImage.createFromPointCloudWithFixedSize(
        *data, 640, 480, 320, 240, 1.f, 1.f, sensorPose, coordinate_frame,
        noiseLevel, minRange);

    viewer.show_range_image(rangeImage, 3);
}