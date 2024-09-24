// PCD文件读取、写入
#include <iostream>
#include <pcl/point_types.h>                  // 包含数据类型
#include <pcl/io/pcd_io.h>                    // PCD格式的IO
#include <pcl/visualization/pcl_visualizer.h> // 可视化模块

using namespace std;
typedef pcl::PointXYZ pointT;

pcl::PointCloud<pointT>::Ptr cloud(new pcl::PointCloud<pointT>); // 存放XYZ点云

/* 写入PCD文件 */
bool write_pcd(const string &save_file, pcl::PointCloud<pointT>::Ptr &cloud_ptr);
/* 读取PCD文件 */
bool load_pcd(const string &save_file, pcl::PointCloud<pointT>::Ptr &cloud_ptr);
/* 显示pcd点云 */
void cloud_viewer_pcd(pcl::PointCloud<pointT>::Ptr &cloud_ptr);

bool write_pcd(const string &save_file, pcl::PointCloud<pointT>::Ptr &cloud_ptr)
{
        // 清空之前的点云
        cloud->clear();
        cloud->width = 10000;
        cloud->height = 1;                                  // 无序点云
        cloud->is_dense = true;                             // 所有点云都有值，则为true。否则为false
        cloud->points.resize(cloud->width * cloud->height); // 重置点云大小

        for (int i = 0; i < cloud->size(); i++)
        {
                cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
                cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
                cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
        }
        pcl::io::savePCDFileASCII(save_file, *cloud);
        cout << "保存点云在路径:" << save_file << endl;
        return true;
}

bool load_pcd(const string &save_file, pcl::PointCloud<pointT>::Ptr &cloud_ptr)
{
        if (pcl::io::loadPCDFile<pointT>(save_file, *cloud_ptr) == -1)
        {
                PCL_ERROR("Couldn't read file *.pcd \n");
                return false;
        }
        return true;
}

void cloud_viewer_pcd(pcl::PointCloud<pointT>::Ptr &cloud_ptr)
{
        // 创建PCL可视化对象（每次都创建新的）
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer")); // 可视化
        viewer->addPointCloud<pointT>(cloud_ptr, "point_cloud");
        // 显示可视化界面
        while (!viewer->wasStopped())
        {
                viewer->spinOnce();
        }
        viewer->close();
}

int main()
{
        // 01 PCD格式加载
        string pcd_file = "../data/test_data.pcd";
        load_pcd(pcd_file, cloud);
        cloud_viewer_pcd(cloud);

        // 02 PCD格式写入
        string pcd_file_random = "./random.pcd";
        write_pcd(pcd_file_random, cloud);
        cloud_viewer_pcd(cloud);
}