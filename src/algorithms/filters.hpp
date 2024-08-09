#pragma once
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>

/// 直通滤波器
void pass_through_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& data,
                         std::string field_name, float min, float max)
{
    float centroid[3];
    int num = std::ceil(data->size() / 30.f);
    for (int i = 0; i < data->size(); i++) {
        if (0 == (i % 30)) {
            centroid[0] += data->points[i].x;
            centroid[1] += data->points[i].y;
            centroid[2] += data->points[i].z;
        }
    }

    std::cout << "centroid b:" << centroid[0] << ',' << centroid[1] << ','
              << centroid[2] << std::endl;

    centroid[0] /= num;
    centroid[1] /= num;
    centroid[2] /= num;

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(data);
    pass.setFilterFieldName(field_name);
    pass.setFilterLimits(centroid[2] - 30.f, centroid[2] + 30.f);
    pass.filter(*data);
}

/// 体素滤波器
void voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       float leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud);
}

/// 统计滤波器
void statistical_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                 int mean_k, float stddev_mul_thresh)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev_mul_thresh);
    sor.filter(*cloud);
}

/// 投影滤波器
/// Example:
///         pcl::ModelCoefficients::Ptr coefficients(new
///         pcl::ModelCoefficients()); coefficients->values.resize(4);
///         coefficients->values[0] = 0.0;
///         coefficients->values[1] = 0.0;
///         coefficients->values[2] = 1.0;
///         coefficients->values[3] = 0.0;
///         project_inliers(cloud1, coefficients, cloud2);
///
void project_inliers(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                     pcl::ModelCoefficients::Ptr& coefficients,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_projected)
{
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);
}

/// 提取索引滤波器
void extract_indices(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                     pcl::PointIndices::Ptr& inliers,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_out);
}

/// 半径滤波器
void radius_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                            float radius, int min_neighbors)
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(radius);
    outrem.setMinNeighborsInRadius(min_neighbors);
    // outrem.setKeepOrganized(true);
    outrem.filter(*cloud);
}

/// 条件滤波器
void conditional_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                         std::string field_name, float min, float max)
{
    float centroid[3];
    int num = std::ceil(cloud->size() / 30.f);
    for (int i = 0; i < cloud->size(); i++) {
        if (0 == (i % 30)) {
            centroid[0] += cloud->points[i].x;
            centroid[1] += cloud->points[i].y;
            centroid[2] += cloud->points[i].z;
        }
    }

    std::cout << "centroid b:" << centroid[0] << ',' << centroid[1] << ','
              << centroid[2] << std::endl;

    centroid[0] /= num;
    centroid[1] /= num;
    centroid[2] /= num;

    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(
        new pcl::ConditionAnd<pcl::PointXYZ>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>(
            field_name, pcl::ComparisonOps::GT, centroid[2] - 30.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>(
            field_name, pcl::ComparisonOps::LT, centroid[2] + 30.0)));
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(true);
    condrem.filter(*cloud);
}