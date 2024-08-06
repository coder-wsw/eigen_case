#pragma once
#include "com_header.h"
#include "gen_coord.hpp"
#include "load_file.hpp"

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
        viewer->setBackgroundColor(r, g, b);
    }
    ~Viewer()
    {
        delete viewer;
    }

    void visual_coord(Eigen::Matrix4f coord, int id)
    {
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
                          0.0005, origin_str.str());

        viewer->addArrow(pcl::PointXYZ(x_axis[0] + origin[0],
                                       x_axis[1] + origin[1],
                                       x_axis[2] + origin[2]),
                         pcl::PointXYZ(origin[0], origin[1], origin[2]), 1.0, 0,
                         0, false, x_axis_str.str());

        viewer->addText3D("x" + std::to_string(id),
                          pcl::PointXYZ(x_axis[0] + origin[0],
                                        x_axis[1] + origin[1],
                                        x_axis[2] + origin[2]),
                          0.05, 1.0, 0.0, 0.0, "x_text" + std::to_string(id));

        viewer->addArrow(pcl::PointXYZ(y_axis[0] + origin[0],
                                       y_axis[1] + origin[1],
                                       y_axis[2] + origin[2]),
                         pcl::PointXYZ(origin[0], origin[1], origin[2]), 0, 1.0,
                         0, false, y_axis_str.str());

        viewer->addText3D("y" + std::to_string(id),
                          pcl::PointXYZ(y_axis[0] + origin[0],
                                        y_axis[1] + origin[1],
                                        y_axis[2] + origin[2]),
                          0.05, 0.0, 1.0, 0.0, "y_text" + std::to_string(id));

        viewer->addArrow(pcl::PointXYZ(z_axis[0] + origin[0],
                                       z_axis[1] + origin[1],
                                       z_axis[2] + origin[2]),
                         pcl::PointXYZ(origin[0], origin[1], origin[2]), 0, 0,
                         1.0, false, z_axis_str.str());

        viewer->addText3D("z" + std::to_string(id),
                          pcl::PointXYZ(z_axis[0] + origin[0],
                                        z_axis[1] + origin[1],
                                        z_axis[2] + origin[2]),
                          0.05, 0.0, 0.0, 1.0, "z_text" + std::to_string(id));
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

    void show_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int id)
    {
        spdlog::info("cloud size: {}", cloud->size());

        int r = 250;
        int g = 250;
        int b = 250;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(
            cloud, r, g, b);

        viewer->addPointCloud(cloud, color, "cloud" + std::to_string(id));
        viewer->spin();
        viewer->removePointCloud("cloud" + std::to_string(id));
        viewer->removePointCloud("feature" + std::to_string(id));
    }

    void show_feature(pcl::PointCloud<pcl::PointXYZ>::Ptr& feature, int id)
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

        viewer->addPointCloud(feature, color, "feature" + std::to_string(id));

        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8,
            "feature" + std::to_string(id));
    }

    void spin()
    {
        viewer->spin();
    }

    void run()
    {
        while (!viewer->wasStopped()) {
            viewer->spinOnce();
        }
    }

private:
    pcl::visualization::PCLVisualizer* viewer;
};

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
    viewer.visual_coord(a_coord, 0);

    Eigen::Matrix4f b_coord;
    gen_coord_system(b_vec, b_coord);
    viewer.visual_coord(b_coord, 1);

    viewer.run();
};

void visual_from_file()
{
    Viewer viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(
        new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr feature1(
        new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(
        new pcl::PointCloud<pcl::PointXYZ>());
    LoadCSVFile("D:/personalSpace/self_create/cpp_space/eigen_case/dataset/"
                "2022-06-09-10-09-44_cloud.csv",
                cloud1, 0);

    LoadCSVFile("D:/personalSpace/self_create/cpp_space/eigen_case/dataset/"
                "2022-06-09-10-09-44_feature.csv",
                feature1, 1);

    LoadCSVFile("D:/personalSpace/self_create/cpp_space/eigen_case/dataset/"
                "2023-12-07-11-10-15_cloud.csv",
                cloud2, 0);

    viewer.show_feature(feature1, 0);
    viewer.show_cloud(cloud1, 0);
    // viewer.show_cloud(cloud2, 1);
}