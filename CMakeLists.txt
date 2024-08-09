cmake_minimum_required(VERSION 3.15)

project(eigen_case
    VERSION 0.1.0
    LANGUAGES C CXX
)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

set(PROJECT_SOURCES 
    src/algorithms/features.hpp
    src/algorithms/filters.hpp
    src/algorithms/range_image.hpp
    src/algorithms/keypoints.hpp
    src/csv.h
    src/main.cpp
    src/gen_coord.hpp
    src/inv_case.hpp
    src/com_header.h
    src/pcl_visual.hpp
    src/algo_visual.hpp
    src/load_file.hpp
    src/viewer.hpp
)

find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")
find_package(Eigen3 REQUIRED)
# find_package(VTK CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED io visualization filters segmentation kdtree features keypoints)
find_package(OpenCV CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(Matplot++ CONFIG REQUIRED)

add_executable(eigen_case ${PROJECT_SOURCES})

target_include_directories(eigen_case PRIVATE ${RAPIDCSV_INCLUDE_DIRS})

target_link_libraries(eigen_case 
    Eigen3::Eigen
    # ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    spdlog::spdlog
    Matplot++::cimg  
    Matplot++::matplot
)

# vtk_module_autoinit(
#     TARGETS ${PROJECT_NAME}
#     MODULES ${VTK_LIBRARIES}
# )