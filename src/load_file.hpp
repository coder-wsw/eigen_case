#include "csv.h" // 确保包含正确的CSV库头文件
#include <chrono>
#include <iostream>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <thread>
#include <vector>

std::mutex mtx;

void LoadCSVChunk(const std::vector<float>& x, const std::vector<float>& y,
                  const std::vector<float>& z,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, size_t start,
                  size_t end)
{
    std::vector<pcl::PointXYZ> points;
    for (size_t i = start; i < end; ++i) {
        points.emplace_back(x[i], y[i], z[i]);
    }

    std::lock_guard<std::mutex> lock(mtx);
    cloud->points.insert(cloud->points.end(), points.begin(), points.end());
}

void LoadCloud(const std::string& file_path, std::vector<float>& x,
               std::vector<float>& y, std::vector<float>& z)
{
    io::CSVReader<3, io::trim_chars<' ', '\t'>, io::no_quote_escape<','>,
                  io::throw_on_overflow, io::no_comment>
        in(file_path);

    float x_val, y_val, z_val;
    while (in.read_row(x_val, y_val, z_val)) {
        x.push_back(x_val);
        y.push_back(y_val);
        z.push_back(z_val);
    }
}

void LoadFeature(const std::string& file_path, std::vector<float>& x,
                 std::vector<float>& y, std::vector<float>& z)
{
    io::CSVReader<10, io::trim_chars<' ', '\t'>, io::no_quote_escape<','>,
                  io::throw_on_overflow, io::no_comment>
        in(file_path);

    float x_val, y_val, z_val, dummy1, dummy2, dummy3, dummy4, dummy5, dummy6,
        dummy7;
    while (in.read_row(x_val, y_val, z_val, dummy1, dummy2, dummy3, dummy4,
                       dummy5, dummy6, dummy7)) {
        x.push_back(x_val);
        y.push_back(y_val);
        z.push_back(z_val);
    }
}

void LoadCSVFile(const std::string& file_path,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int type)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    try {
        // io::CSVReader<3, io::trim_chars<' ', '\t'>, io::no_quote_escape<','>,
        //               io::throw_on_overflow, io::no_comment>
        //     in(file_path);

        std::vector<float> x, y, z;
        // float x_val, y_val, z_val;

        // while (in.read_row(x_val, y_val, z_val)) {
        //     x.push_back(x_val);
        //     y.push_back(y_val);
        //     z.push_back(z_val);
        // }

        if (type == 0) {
            LoadCloud(file_path, x, y, z);
        }
        else {
            LoadFeature(file_path, x, y, z);
        }

        size_t num_threads = std::thread::hardware_concurrency();
        size_t chunk_size = x.size() / num_threads;

        std::vector<std::thread> threads;
        for (size_t i = 0; i < num_threads; ++i) {
            size_t start = i * chunk_size;
            size_t end = (i == num_threads - 1) ? x.size()
                                                : (i + 1) * chunk_size;
            threads.emplace_back(LoadCSVChunk, std::ref(x), std::ref(y),
                                 std::ref(z), std::ref(cloud), start, end);
        }

        for (auto& thread : threads) {
            thread.join();
        }

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        std::cout << "Time taken: " << elapsed.count() << " seconds"
                  << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Error reading CSV file: " << e.what() << std::endl;
    }
}