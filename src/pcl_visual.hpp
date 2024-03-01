#include "com_header.h"

class Viewer {
public:
    Viewer()
    {
        viewer = new pcl::visualization::PCLVisualizer("viewer");
        viewer->setBackgroundColor(0, 0, 0);
        // viewer.addCoordinateSystem(0.5);
        viewer->setCameraPosition(4, 0, 4, 0, 0, 0, 0, 0, 1);
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
        viewer->addArrow(pcl::PointXYZ(y_axis[0] + origin[0],
                                       y_axis[1] + origin[1],
                                       y_axis[2] + origin[2]),
                         pcl::PointXYZ(origin[0], origin[1], origin[2]), 0, 1.0,
                         0, false, y_axis_str.str());
        viewer->addArrow(pcl::PointXYZ(z_axis[0] + origin[0],
                                       z_axis[1] + origin[1],
                                       z_axis[2] + origin[2]),
                         pcl::PointXYZ(origin[0], origin[1], origin[2]), 0, 0,
                         1.0, false, z_axis_str.str());
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

    b_vec.push_back(Point{1, 1, 1});
    b_vec.push_back(Point{2, 1, 1});
    b_vec.push_back(Point{1, 1, 2});

    Eigen::Matrix4f a_coord;
    gen_coord_system(a_vec, a_coord);
    viewer.visual_coord(a_coord, 0);

    Eigen::Matrix4f b_coord;
    gen_coord_system(b_vec, b_coord);
    viewer.visual_coord(b_coord, 1);

    viewer.run();
}