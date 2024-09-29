#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
struct callback_args
{
        PointCloudT ::Ptr clicked_points_3d = nullptr;
        pcl::visualization::PCLVisualizer::Ptr viewerPtr = nullptr;
};

PointCloudT::Ptr pointCloud(new PointCloudT());        // 用于存储pcd点云
PointCloudT::Ptr clicked_points_3d(new PointCloudT()); // 用于存储选择的点云

pcl::visualization::PCLVisualizer ::Ptr viewer(new pcl::visualization::PCLVisualizer("point cloud viewer"));

int point_selected_index = 0;

void pp_callback(const pcl::visualization::PointPickingEvent &event, void *args)
{
        if (event.getPointIndex() == -1)
        {
                return;
        }
        PointT currentPoint, previous_point;
        event.getPoint(currentPoint.x, currentPoint.y, currentPoint.z);

        callback_args *data = (callback_args *)args;
        if (!data->clicked_points_3d->points.empty())
        {
                size_t num = data->clicked_points_3d->points.size();
                for (size_t i = 0; i < num; i++)
                {
                        previous_point = data->clicked_points_3d->points[i];
                        if ((currentPoint.x == previous_point.x) && (currentPoint.y == previous_point.y) && (currentPoint.z == previous_point.z))
                        {
                                return;
                        }
                }
        }
        data->clicked_points_3d->points.push_back(currentPoint);
        // 设置点云的颜色
        pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
        // 增加点云
        if (data->clicked_points_3d->size() == 1)
        {
                data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
        }
        else
        {
                data->viewerPtr->updatePointCloud(data->clicked_points_3d, red, "clicked_points");
        }
        data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "clicked_points");
        std::cout << "第" << point_selected_index << "个选择点:  X:" << currentPoint.x << ",  Y:" << currentPoint.y << ",  Z:" << currentPoint.z << std::endl;
        point_selected_index++;
}

int main(int argc, char const *argv[])
{
        //  1、读取点云
        const std::string pcd_file = "../data/arm.pcd";
        if (pcl::io::loadPCDFile(pcd_file, *pointCloud))
        {
                PCL_ERROR("读取点云失败~");
                return -1;
        }
        // 2、显示点云
        viewer->addPointCloud(pointCloud, "arm");
        // 3、设置相机位置
        viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
        // 4、参数声明
        callback_args cb_args;
        cb_args.clicked_points_3d = clicked_points_3d;
        cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);

        // viewer注册回调函数
        viewer->registerPointPickingCallback(pp_callback, (void *) &cb_args);
        std::cout<<"按住shift+左键选择单个点云, 按Q退出"<<std::endl;

        while (!viewer->wasStopped())
        {
                viewer->spinOnce();
                usleep(50000);
        }

        return 0;
}
