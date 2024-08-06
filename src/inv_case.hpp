#pragma once
#include "com_header.h"

void inv_case()
{
    Eigen::Matrix3f matrix;
    matrix.setIdentity();
    matrix.block<2, 2>(0, 0) << 0.0f, -1.0f, 1.0f, 0.0f;
    matrix.block<2, 1>(0, 2) << 0.0f, 0.0f;
    {
        std::stringstream buf1;
        buf1 << matrix;
        SPDLOG_INFO("matrix:\n{}", buf1.str());
    }

    Eigen::Vector3f p;
    p << 2.0f, 2.0f, 1.0f;
    {
        std::stringstream buf1;
        buf1 << p;
        SPDLOG_INFO("p:\n{}", buf1.str());
    }

    {
        std::stringstream buf1;
        buf1 << matrix.inverse();
        SPDLOG_INFO("inv:\n{}", buf1.str());
    }

    auto res = matrix * p;
    {
        std::stringstream buf1;
        buf1 << res;
        SPDLOG_INFO("res:\n{}", buf1.str());
    }

    auto res1 = matrix.inverse() * p;
    {
        std::stringstream buf1;
        buf1 << res1;
        SPDLOG_INFO("res.inv:\n{}", buf1.str());
    }
}
