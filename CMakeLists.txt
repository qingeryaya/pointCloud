cmake_minimum_required(VERSION 3.10)
project(pcl_io_example)

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
# 链接PCL相关库
target_link_libraries(pcd_io_example ${PCL_LIBRARIES})


# 添加可执行文件 PLY
add_executable(ply_io_example PCL_IO/PLY_IO.cpp)
# 链接PCL相关库
target_link_libraries(ply_io_example ${PCL_LIBRARIES})

# 添加可执行文件 OBJ
add_executable(obj_io_example PCL_IO/OBJ_IO.cpp)
# 链接PCL相关库
target_link_libraries(obj_io_example ${PCL_LIBRARIES})

# 添加可执行文件 OBJ
add_executable(pcl_viewer PCL_IO2/PCL_VIEWER.cpp)
# 链接PCL相关库
target_link_libraries(pcl_viewer ${PCL_LIBRARIES})