#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::visualization::PCLVisualizer ::Ptr viewer(new pcl::visualization::PCLVisualizer("point cloud viewer"));
pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);

// 用来计数
int num = 0;

void pp_callback(const pcl::visualization::AreaPickingEvent &event, void *args)
{
        std::vector<int> indices;
        if (event.getPointsIndices(indices) == -1)
        {
                cout << "结束" << endl;
                return;
        }

        for (int i = 0; i < indices.size(); ++i)
        {
                clicked_points_3d->points.push_back(cloud->points.at(indices[i]));
        }
        // 设置颜色
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);

        std::stringstream ss;
        std::string cloudName;
        ss << num++;
        ss >> cloudName;
        cloudName += "_cloudName";

        viewer->addPointCloud(clicked_points_3d, red, cloudName);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
}

int main()
{
        if (pcl::io::loadPCDFile("../data/arm.pcd", *cloud))
        {
                std::cerr << "ERROR: Cannot open file " << std::endl;
                return -1;
        }
        cout << "按X选择区域点云，按Q结束程序" << endl;
        viewer->addPointCloud(cloud, "arm");
        viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);

        // 注册区域选择回调
        viewer->registerAreaPickingCallback(pp_callback, (void *)&cloud);

        while (!viewer->wasStopped())
        {
                viewer->spinOnce(100);
                usleep(50000);
        }
        return 0;
}