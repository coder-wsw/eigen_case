#ifndef COM_HEADER_H
#define COM_HEADER_H

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <matplot/matplot.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <random>
#include <spdlog/spdlog.h>

namespace mp = matplot;

#define Debug(msg)                                                             \
    {                                                                          \
        std::stringstream buf1;                                                \
        buf1 << msg;                                                           \
        SPDLOG_INFO("{}:\n{}", #msg, buf1.str());                              \
    }

#define Debug_Vec(vec)                                                         \
    {                                                                          \
        std::string delimiter = ",";                                           \
        std::stringstream str;                                                 \
        std::copy(vec.begin(), vec.end(),                                      \
                  std::ostream_iterator<double>(str, delimiter.c_str()));      \
        SPDLOG_INFO("{}:{},[{}]", #vec, vec.size(), str.str());                \
    }

#define Run(val, func, ...)                                                    \
    if (val == 1)                                                              \
                                                                               \
    {                                                                          \
        func(__VA_ARGS__);                                                     \
    }

#endif
