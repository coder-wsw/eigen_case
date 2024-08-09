#pragma once
#include "com_header.h"
#include "viewer.hpp"
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>

/// 提取NARF关键点
void extract_narf_keypoint(pcl::RangeImage& range_image, Viewer& viewer,
                           cv::Mat& img)
{
    pcl::RangeImageBorderExtractor border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector(&border_extractor);
    pcl::PointCloud<int> keypoint_indices;

    narf_keypoint_detector.setRangeImage(&range_image);
    narf_keypoint_detector.getParameters().support_size = 0.2f;
    // narf_keypoint_detector.getParameters().add_points_on_straight_edges =
    // true;
    // narf_keypoint_detector.getParameters().distance_for_additional_points =
    //     0.5f;

    narf_keypoint_detector.compute(keypoint_indices);
    spdlog::info("NARF keypoint num: {}", keypoint_indices.points.size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(
        new pcl::PointCloud<pcl::PointXYZ>());
    keypoints->points.resize(keypoint_indices.points.size());
    for (size_t i = 0; i < keypoint_indices.points.size(); i++) {
        keypoints->points[i].getVector3fMap() =
            range_image.points[keypoint_indices.points[i]].getVector3fMap();
    }
    viewer.show_feature(keypoints, 1, 2);

    pcl::visualization::RangeImageVisualizer range_image_visualizer(
        "Range Image");

    pcl::visualization::Vector3ub fgcolor(255, 255, 255);
    for (std::size_t i = 0; i < keypoint_indices.size(); ++i) {
        range_image_visualizer.markPoint(
            keypoint_indices[i] % range_image.width,
            keypoint_indices[i] / range_image.width, fgcolor);
    }
    range_image_visualizer.showRangeImage(range_image);

    // 给img添加红色关键点显示
    for (size_t i = 0; i < keypoint_indices.size(); i++) {
        int x = keypoint_indices[i] % img.cols;
        int y = keypoint_indices[i] / img.cols;
        img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);

        cv::circle(img, cv::Point(x, y), 10, cv::Scalar(0, 0, 255), -1);
    }

    // Keep the visualizer window open
    range_image_visualizer.spin();
    range_image_visualizer.close();
}