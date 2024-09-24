// obj文件读取, 显示

#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

bool load_obj(const string &file, pcl::PolygonMesh &mesh);
void cloud_viewer(pcl::PolygonMesh &mesh);

bool load_obj(const string &file, pcl::PolygonMesh &mesh)
{
        if (pcl::io::loadOBJFile(file, mesh) == -1)
        {
                PCL_ERROR("Couldn't read file *.obj \n");
                return false;
        }
        return true;
}

void cloud_viewer(pcl::PolygonMesh &mesh)
{
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer")); // 可视化
        viewer->addPolygonMesh(mesh, "mesh");
        while (!viewer->wasStopped())
        {
                viewer->spinOnce();
        }
        viewer->close();
}

int main()
{
        // 01 obj文件读取
        const string file = "../data/test_data.obj";
        pcl::PolygonMesh mesh; // mesh格式存储

        load_obj(file, mesh);
        cloud_viewer(mesh);
        return 0;
}
