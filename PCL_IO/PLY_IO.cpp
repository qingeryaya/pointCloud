// PLY文件读取、写入

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
typedef pcl::PointXYZ pointT;

pcl::PointCloud<pointT>::Ptr cloud(new pcl::PointCloud<pointT>); // 存放XYZ点云

/* 读取PLY文件 */
bool load_ply(const string &save_file, pcl::PointCloud<pointT>::Ptr &cloud_ptr);
/* 显示PLY点云 */
void cloud_viewer_pcd(pcl::PointCloud<pointT>::Ptr &cloud_ptr);

bool load_ply(const string &save_file, pcl::PointCloud<pointT>::Ptr &cloud_ptr)
{
        if (pcl::io::loadPLYFile<pointT>(save_file, *cloud_ptr) == -1)
        {
                PCL_ERROR("Couldn't read file *.pcd \n");
                return false;
        }
        return true;
}

void cloud_viewer(pcl::PointCloud<pointT>::Ptr &cloud_ptr)
{
        // 创建PCL可视化对象（每次都创建新的）
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));  // 可视化
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_ptr, 0, 255, 0); // 颜色渲染
        viewer->addPointCloud<pointT>(cloud_ptr, single_color, "point_cloud");

        // 显示可视化界面
        while (!viewer->wasStopped())
        {
                viewer->spinOnce();
        }
        viewer->close();
}

int main()
{
        // PLY格式加载
        string pcd_file = "../data/test_data.ply";
        load_ply(pcd_file, cloud);
        cloud_viewer(cloud);
        return 1;
}