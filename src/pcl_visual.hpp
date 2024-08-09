#pragma once
#include "algorithms/features.hpp"
#include "algorithms/filters.hpp"
#include "algorithms/range_image.hpp"
#include "gen_coord.hpp"
#include "load_file.hpp"
#include "viewer.hpp"

void pcl_visual()
{
    Viewer viewer;

    std::vector<Point> a_vec;
    std::vector<Point> b_vec;

    a_vec.push_back(Point{0, 0, 0});
    a_vec.push_back(Point{1, 0, 0});
    a_vec.push_back(Point{0, 0, 1});

    viewer.visual_point(a_vec, 0);

    b_vec.push_back(Point{1, 1, 1});
    b_vec.push_back(Point{2, 1, 1});
    b_vec.push_back(Point{1, 1, 2});

    viewer.visual_point(b_vec, 1);

    Eigen::Matrix4f a_coord;
    gen_coord_system(a_vec, a_coord);
    viewer.visual_coord(a_coord, 0, 1);

    Eigen::Matrix4f b_coord;
    gen_coord_system(b_vec, b_coord);
    viewer.visual_coord(b_coord, 1, 1);

    viewer.run();
};

void filter_examples()
{
    Viewer viewer;
    int v1 = 1;
    int v2 = 2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(
        new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr feature1(
        new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(
        new pcl::PointCloud<pcl::PointXYZ>());

    // LoadCSVFile("D:/personalSpace/self_create/cpp_space/eigen_case/dataset/"
    //             "2022-06-09-10-09-44_cloud.csv",
    //             cloud1, 0);

    // LoadCSVFile("D:/personalSpace/self_create/cpp_space/eigen_case/dataset/"
    //             "2022-06-09-10-09-44_feature.csv",
    //             feature1, 1);

    LoadCSVFile("D:/personalSpace/self_create/cpp_space/eigen_case/dataset/"
                "2023-12-07-11-10-15_cloud.csv",
                cloud1, 0);

    // viewer.show_feature(feature1, 0);
    viewer.show_cloud(cloud1, 1, v1);

    // pass_through_filter(cloud1, "z", 0.0, 2000.0);

    voxel_grid_filter(cloud1, 2);

    // statistical_outlier_removal(cloud1, 100, 1.0);

    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // coefficients->values.resize(4);
    // coefficients->values[0] = 0.0;
    // coefficients->values[1] = 0.0;
    // coefficients->values[2] = 1.0;
    // coefficients->values[3] = 0.0;
    // project_inliers(cloud1, coefficients, cloud2);

    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // pcl::SACSegmentation<pcl::PointXYZ> seg;
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(500);
    // seg.setDistanceThreshold(0.5);
    // seg.setInputCloud(cloud1);
    // seg.segment(*inliers, *coefficients);
    // extract_indices(cloud1, inliers, cloud2);

    // radius_outlier_removal(cloud1, 20, 40);

    // conditional_removal(cloud1, "z", 0.0, 2000.0);
    // viewer.clear(1, v1);
    viewer.show_cloud(cloud1, 2, v2);
    // viewer.show_cloud(cloud2, 2);
    // viewer.cloud2image(cloud1);
    // viewer.show_cloud(cloud2, 1);
}

void feature_examples()
{
    Viewer viewer;
    int v1 = 1;
    int v2 = 2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(
        new pcl::PointCloud<pcl::PointXYZ>());

    LoadCSVFile("D:/personalSpace/self_create/cpp_space/eigen_case/dataset/"
                "2023-12-07-11-10-15_cloud.csv",
                cloud1, 0);

    viewer.show_cloud(cloud1, 1, v1);

    voxel_grid_filter(cloud1, 2);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    normal_estimation(cloud1, normals, 10);
    // flip_normals(normals);

    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors(
        new pcl::PointCloud<pcl::PFHSignature125>);

    pfh_estimation(cloud1, normals, descriptors, 0.05);

    viewer.show_normal(cloud1, normals, 2, v2);
}

void range_image_examples()
{
    Viewer viewer;
    int v1 = 1;
    int v2 = 2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(
        new pcl::PointCloud<pcl::PointXYZ>());

    LoadCSVFile("D:/personalSpace/self_create/cpp_space/eigen_case/dataset/"
                "2023-12-07-11-10-15_cloud.csv",
                cloud1, 0);

    viewer.show_cloud(cloud1, 1, v1);
    voxel_grid_filter(cloud1, 3);
    range_image(cloud1, viewer);
}