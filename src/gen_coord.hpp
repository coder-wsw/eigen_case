#pragma once
#include "com_header.h"

struct Point
{
    float x;
    float y;
    float z;
};

void gen_coord_system(std::vector<Point> a_vec, Eigen::Matrix4f& a_coord)
{
    Eigen::Vector3f ax, ay, az;

    Eigen::Vector3f v1, v2;
    v1 << a_vec[1].x - a_vec[0].x, a_vec[1].y - a_vec[0].y,
        a_vec[1].z - a_vec[0].z;
    ax << v1.normalized();
    Debug(ax);

    v2 << a_vec[2].x - a_vec[0].x, a_vec[2].y - a_vec[0].y,
        a_vec[2].z - a_vec[0].z;
    v2 << v2.normalized();
    ay << v2.cross(ax);
    Debug(ay);

    az << ax.cross(ay);
    Debug(az);

    a_coord.setIdentity();
    a_coord.block<3, 3>(0, 0) << ax, ay, az;
    a_coord.block<3, 1>(0, 3) << a_vec[0].x, a_vec[0].y, a_vec[0].z;
}

Eigen::Matrix4f get_trans_matrix(std::vector<Point> a_vec,
                                 std::vector<Point> b_vec)
{
    Eigen::Matrix4f a_coord, b_coord;
    Eigen::Matrix4f res;

    res.setIdentity();
    gen_coord_system(a_vec, a_coord);
    Debug(a_coord);
    gen_coord_system(b_vec, b_coord);
    Debug(b_coord);

    res = b_coord.inverse() * a_coord;
    Debug(res);

    return res;
}

void gen_coord_example()
{
    std::vector<Point> a_vec;
    std::vector<Point> b_vec;

    a_vec.push_back(Point{0, 0, 0});
    a_vec.push_back(Point{1, 0, 0});
    a_vec.push_back(Point{0, 0, 1});

    b_vec.push_back(Point{1, 1, 1});
    b_vec.push_back(Point{2, 1, 1});
    b_vec.push_back(Point{1, 1, 2});

    auto tm = get_trans_matrix(a_vec, b_vec);

    Eigen::Vector4f testp;
    testp << a_vec[0].x, a_vec[0].y, a_vec[0].z, 1;

    auto resp = tm * testp;
    Debug(resp);
}
