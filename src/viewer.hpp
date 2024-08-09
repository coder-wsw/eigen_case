#pragma once
#include "com_header.h"
#include "gen_coord.hpp"

#define ENABLE_MULTI_WINDOWS 1
class Viewer {
public:
    Viewer()
    {
        // 9, 89, 131
        // 14 140 207
        // 6 70 105
        viewer = new pcl::visualization::PCLVisualizer("viewer");
        double r = 6 / 255.0;
        double g = 70 / 255.0;
        double b = 105 / 255.0;

#if ENABLE_MULTI_WINDOWS
        double r1 = 9 / 255.0;
        double g1 = 89 / 255.0;
        double b1 = 131 / 255.0;
        v1 = 1;
        v2 = 2;
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer->setBackgroundColor(r, g, b, v1);
        viewer->setBackgroundColor(r1, g1, b1, v2);
        viewer->createViewPortCamera(v1);
        viewer->createViewPortCamera(v2);
#else
        viewer->setBackgroundColor(r, g, b);
#endif
    }
    ~Viewer()
    {
        delete viewer;
    }

    void add_coord(Eigen::Matrix4f coord, int id, int v, double scale)
    {
        Eigen::Affine3f coord_affine;
        coord_affine.matrix() = coord;
        viewer->addCoordinateSystem(scale, coord_affine,
                                    "coord_affine" + std::to_string(id), v);
    }

    void visual_coord(Eigen::Matrix4f coord, int id, int v)
    {
#if ENABLE_MULTI_WINDOWS
#else
        v = 0;
#endif
        Eigen::Vector3f origin;
        Eigen::Vector3f x_axis;
        Eigen::Vector3f y_axis;
        Eigen::Vector3f z_axis;

        origin = coord.block<3, 1>(0, 3);
        x_axis = coord.block<3, 1>(0, 0);
        y_axis = coord.block<3, 1>(0, 1);
        z_axis = coord.block<3, 1>(0, 2);

        std::stringstream origin_str;
        origin_str << "origin" + std::to_string(id);

        std::stringstream x_axis_str;
        x_axis_str << "x_axis" + std::to_string(id);

        std::stringstream y_axis_str;
        y_axis_str << "y_axis" + std::to_string(id);

        std::stringstream z_axis_str;
        z_axis_str << "z_axis" + std::to_string(id);

        viewer->addSphere(pcl::PointXYZ(origin[0], origin[1], origin[2]),
                          0.0005, origin_str.str(), v);

        viewer->addArrow(pcl::PointXYZ(x_axis[0] + origin[0],
                                       x_axis[1] + origin[1],
                                       x_axis[2] + origin[2]),
                         pcl::PointXYZ(origin[0], origin[1], origin[2]), 1.0, 0,
                         0, false, x_axis_str.str(), v);

        viewer->addText3D(
            "x" + std::to_string(id),
            pcl::PointXYZ(x_axis[0] + origin[0], x_axis[1] + origin[1],
                          x_axis[2] + origin[2]),
            0.05, 1.0, 0.0, 0.0, "x_text" + std::to_string(id), v);

        viewer->addArrow(pcl::PointXYZ(y_axis[0] + origin[0],
                                       y_axis[1] + origin[1],
                                       y_axis[2] + origin[2]),
                         pcl::PointXYZ(origin[0], origin[1], origin[2]), 0, 1.0,
                         0, false, y_axis_str.str(), v);

        viewer->addText3D(
            "y" + std::to_string(id),
            pcl::PointXYZ(y_axis[0] + origin[0], y_axis[1] + origin[1],
                          y_axis[2] + origin[2]),
            0.05, 0.0, 1.0, 0.0, "y_text" + std::to_string(id), v);

        viewer->addArrow(pcl::PointXYZ(z_axis[0] + origin[0],
                                       z_axis[1] + origin[1],
                                       z_axis[2] + origin[2]),
                         pcl::PointXYZ(origin[0], origin[1], origin[2]), 0, 0,
                         1.0, false, z_axis_str.str(), v);

        viewer->addText3D(
            "z" + std::to_string(id),
            pcl::PointXYZ(z_axis[0] + origin[0], z_axis[1] + origin[1],
                          z_axis[2] + origin[2]),
            0.05, 0.0, 0.0, 1.0, "z_text" + std::to_string(id), v);
    }

    void visual_point(std::vector<Point>& p, int id)
    {
        for (int i = 0; i < p.size(); i++) {
            std::stringstream ss;
            ss << "point" << id << i;
            double r = i == 0 ? 1.0 : 0.0;
            double g = i == 1 ? 1.0 : 0.0;
            double b = i == 2 ? 1.0 : 0.0;
            viewer->addSphere(pcl::PointXYZ(p[i].x, p[i].y, p[i].z), 0.03, r, g,
                              b, ss.str());
        }
    }

    void show_cloud(typename pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int id,
                    int v)
    {
        spdlog::info("cloud size: {}", cloud->size());

        int r = 250;
        int g = 250;
        int b = 250;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(
            cloud, r, g, b);
#if ENABLE_MULTI_WINDOWS
        viewer->addPointCloud(cloud, color, "cloud" + std::to_string(id), v);
#else
        v = 0;
        viewer->addPointCloud(cloud, color, "cloud" + std::to_string(id), v);
#endif
        viewer->spin();
    }

    void show_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                     pcl::PointCloud<pcl::Normal>::Ptr& normal, int id, int v)
    {
        spdlog::info("cloud size: {}", cloud->size());
        spdlog::info("normal size: {}", normal->size());

        int r = 250;
        int g = 250;
        int b = 250;

        int r1 = 255;
        int g1 = 102;
        int b1 = 102;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(
            cloud, r, g, b);

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
            new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud, *normal, *cloud_with_normals);
#if ENABLE_MULTI_WINDOWS
        viewer->addPointCloud(cloud, color, "cloud" + std::to_string(id), v);
        viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
            cloud, normal, 10, 2, "normal" + std::to_string(id), v);
#else
        v = 0;
        viewer->addPointCloud(cloud, color, "cloud" + std::to_string(id), v);
        viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
            cloud, normal, 10, 2, "normal" + std::to_string(id), v);
#endif
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, r1 / 255.0, g1 / 255.0,
            b1 / 255.0, "normal" + std::to_string(id));

        viewer->spin();
    }

    void show_feature(pcl::PointCloud<pcl::PointXYZ>::Ptr& feature, int id,
                      int v)
    {
        spdlog::info("feature size: {}", feature->size());

        int r = 255;
        int g = 102;
        int b = 102;

        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>
            color(feature, "z");

        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        // color(
        //     feature, r, g, b);
#if ENABLE_MULTI_WINDOWS
        viewer->addPointCloud(feature, color, "feature" + std::to_string(id),
                              v);
#else
        v = 0;
        viewer->addPointCloud(feature, color, "feature" + std::to_string(id),
                              v);
#endif
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8,
            "feature" + std::to_string(id), v);
    }

    void show_range_image(pcl::RangeImage& range_image, int id)
    {
        pcl::visualization::RangeImageVisualizer range_image_visualizer(
            "Range Image");
        range_image_visualizer.showRangeImage(range_image);

        // Keep the visualizer window open
        range_image_visualizer.spin();
        range_image_visualizer.close();
    }

    void clear(int id, int v)
    {
#if ENABLE_MULTI_WINDOWS
        viewer->removePointCloud("cloud" + std::to_string(id), v);
        viewer->removePointCloud("feature" + std::to_string(id), v);
#else
        viewer->removePointCloud("cloud" + std::to_string(id));
        viewer->removePointCloud("feature" + std::to_string(id));
#endif
    }

    void spin()
    {
        viewer->spin();
        viewer->close();
    }

    void run()
    {
        while (!viewer->wasStopped()) {
            viewer->spinOnce();
        }
        viewer->close();
    }

private:
    pcl::visualization::PCLVisualizer* viewer;
    int v1;
    int v2;
};