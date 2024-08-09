#pragma once
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/point_cloud.h>

/// 法线估计
void normal_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       pcl::PointCloud<pcl::Normal>::Ptr& normals, float radius)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius);
    ne.compute(*normals);
}

/// 法线方向反转
void flip_normals(pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
    for (int i = 0; i < normals->size(); i++) {
        normals->points[i].normal_x *= -1;
        normals->points[i].normal_y *= -1;
        normals->points[i].normal_z *= -1;
    }
}

/// PFH特征估计
void pfh_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    pcl::PointCloud<pcl::Normal>::Ptr& normals,
                    pcl::PointCloud<pcl::PFHSignature125>::Ptr& descriptors,
                    float radius)
{
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(cloud);
    pfh.setInputNormals(normals);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    pfh.setSearchMethod(tree);
    pfh.setRadiusSearch(radius);
    pfh.compute(*descriptors);
}
