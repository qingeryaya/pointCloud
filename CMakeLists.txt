cmake_minimum_required(VERSION 3.10)
project(pcl_io_example)
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)


# 找到PCL库
find_package(PCL 1.9.1 REQUIRED)

# 添加编译选项
add_definitions(${PCL_DEFINITIONS})

# 包含头文件路径
include_directories(${PCL_INCLUDE_DIRS})

# 链接库文件路径
link_directories(${PCL_LIBRARY_DIRS})

# 添加可执行文件 PCD
add_executable(pcd_io_example PCL_IO/PCD_IO.cpp)
target_link_libraries(pcd_io_example ${PCL_LIBRARIES})

# 添加可执行文件 PLY
add_executable(ply_io_example PCL_IO/PLY_IO.cpp)
target_link_libraries(ply_io_example ${PCL_LIBRARIES})

add_executable(obj_io_example PCL_IO/OBJ_IO.cpp)
target_link_libraries(obj_io_example ${PCL_LIBRARIES})

# PCL_IO2  点云上色, 绘制形状, 绘制法线, 设置窗口
add_executable(pcl_viewer PCL_IO2/PCL_VIEWER.cpp)
target_link_libraries(pcl_viewer ${PCL_LIBRARIES})

# PCL_IO3  选取点云上的单个点
add_executable(pcl_single_point_select PCL_IO3/singlePointCloudSelect.cpp)
target_link_libraries(pcl_single_point_select ${PCL_LIBRARIES})

# PCL_IO3  选取点云上的单个点
add_executable(pcl_roi_select PCL_IO3/pointCloudROISelect.cpp)
target_link_libraries(pcl_roi_select ${PCL_LIBRARIES})