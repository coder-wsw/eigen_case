cmake_minimum_required(VERSION 3.15)

project(eigen_case
    VERSION 0.1.0
    LANGUAGES C CXX
)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

set(PROJECT_SOURCES 
    src/main.cpp
    src/gen_coord.hpp
    src/inv_case.hpp
    src/com_header.h
    src/pcl_visual.hpp
    src/algo_visual.hpp
)

find_package(Eigen3 REQUIRED)
# find_package(VTK CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED io visualization)
find_package(spdlog CONFIG REQUIRED)
find_package(Matplot++ CONFIG REQUIRED)

add_executable(eigen_case ${PROJECT_SOURCES})

target_link_libraries(eigen_case 
    Eigen3::Eigen
    # ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
    spdlog::spdlog
    Matplot++::cimg  
    Matplot++::matplot
)

# vtk_module_autoinit(
#     TARGETS ${PROJECT_NAME}
#     MODULES ${VTK_LIBRARIES}
# )