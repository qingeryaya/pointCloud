#include <iostream>
#include <thread>
#include <pcl/point_types.h>				    // 数据类型
#include <pcl/io/pcd_io.h>						// 读取点云
#include <pcl/visualization/pcl_visualizer.h>   // 可视化界面
#include <pcl/common/angles.h>					// 角度相关函数
#include <pcl/features/normal_3d.h>				// 计算法向量
#include <vtkRenderWindow.h>


using namespace std;


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);						// 存储点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);			// 彩色点云
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_1(new pcl::PointCloud <pcl::Normal>);				// 法向量1
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_2(new pcl::PointCloud <pcl::Normal>);				// 法向量2
pcl::visualization::PCLVisualizer::Ptr viewer = nullptr;	// 可视化浏览器


// 生成数据
void make_data();
// 01 普通点云显示
void  viewer_simple(pcl::PointCloud <pcl::PointXYZ>::ConstPtr cloud_ptr); // 常数指针方式传入
// 02 彩色点云显示
void  viewer_rgb(pcl::PointCloud <pcl::PointXYZRGB>::ConstPtr cloud_ptr);
// 03 指定颜色显示
void viewer_color(pcl::PointCloud <pcl::PointXYZ>::ConstPtr cloud_ptr);
// 04 深度渲染
void viewer_depth(pcl::PointCloud <pcl::PointXYZ>::ConstPtr cloud_ptr);
// 05 绘制法线
void viewer_normals(
	pcl::PointCloud <pcl::PointXYZRGB>::ConstPtr cloud_ptr, // 点云
	pcl::PointCloud<pcl::Normal>::ConstPtr normals_ptr);	// 法向量
// 06 绘制几何体
void viewer_shape(pcl::PointCloud <pcl::PointXYZRGB>::ConstPtr cloud_ptr);
// 07 多个窗口对比
void viewer_2(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_ptr,
	pcl::PointCloud<pcl::Normal>::ConstPtr normals1_ptr, pcl::PointCloud<pcl::Normal>::ConstPtr normals2_ptr);
// 设置点云其他属性
void cloud_viewer();



void make_data() {
	uint8_t r = 255, g = 15, b = 15;  // 颜色

	for (float z = -1.; z <= 1.0; z += 0.05)  // 距离范围
	{
		for (float angle = 0.0; angle <= 360.0; angle += 5) // 角度范围
		{
			pcl::PointXYZ p;
			p.x = 0.5 * std::cos(pcl::deg2rad(angle));
			p.y = sinf(pcl::deg2rad(angle));
			p.z = z;
			cloud->points.emplace_back(p);

			pcl::PointXYZRGB p_color;
			p_color.x = p.x;
			p_color.y = p.y;
			p_color.z = p.z;
			p_color.r = r;
			p_color.g = g;
			p_color.b = b;
			cloud_rgb->points.emplace_back(p_color);
			if (z < 0.0) {
				r -= 12;
				g += 12;
			}
			else {
				g -= 12;
				b += 12;
			}
		}
	}

	cloud->width = cloud->size();
	cloud->height = 1;

	cloud_rgb->width = cloud_rgb->size();
	cloud_rgb->height = 1;

	// 计算表面法向量
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> est_norm;
	est_norm.setInputCloud(cloud_rgb);   // 设置输入点云

	// 设置搜索数据结构
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	est_norm.setSearchMethod(tree);		// 设置搜索方法
	est_norm.setRadiusSearch(0.05);		// 设置搜索半径
	est_norm.compute(*cloud_normals_1);	// 计算法向量1

	est_norm.setRadiusSearch(0.1);
	est_norm.compute(*cloud_normals_2); // 计算法向量2
}

void  viewer_simple(pcl::PointCloud <pcl::PointXYZ>::ConstPtr cloud_ptr) {
	viewer->addPointCloud<pcl::PointXYZ>(cloud_ptr, "point_cloud");		 // 添加点云
}

void  viewer_rgb(pcl::PointCloud <pcl::PointXYZRGB>::ConstPtr cloud_ptr) {
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_ptr);	//  颜色
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_ptr, rgb, "point_cloud");
}

void  viewer_color(pcl::PointCloud <pcl::PointXYZ>::ConstPtr cloud_ptr) {
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_ptr, 255, 0, 0); 	// 颜色渲染
	viewer->addPointCloud<pcl::PointXYZ>(cloud_ptr, single_color, "point_cloud");
}

void viewer_depth(pcl::PointCloud <pcl::PointXYZ>::ConstPtr cloud_ptr) {
	// 按Z轴渲染
	pcl::visualization::PointCloudColorHandlerGenericField <pcl::PointXYZ> color(cloud_ptr, "x");
	viewer->addPointCloud<pcl::PointXYZ>(cloud_ptr, color, "point_cloud");
}


void viewer_normals(
	pcl::PointCloud <pcl::PointXYZRGB>::ConstPtr cloud_ptr, // 点云
	pcl::PointCloud<pcl::Normal>::ConstPtr normals_ptr) {
	// 彩色渲染
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_ptr);
	// 添加彩色点云
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_ptr, rgb, "point_cloud");
	// 添加法向量
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud_ptr, normals_ptr, 2, 0.05, "normals");
}

void viewer_shape(pcl::PointCloud <pcl::PointXYZRGB>::ConstPtr cloud_ptr) {
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_ptr);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_ptr, rgb, "point_cloud");

	// 01 添加直线
	viewer->addLine<pcl::PointXYZRGB>((*cloud_ptr)[0], (*cloud_ptr)[cloud_ptr->size() - 1], "line");
	// 02 添加球：第一个点为中心，半径为0.2。 r=1, g=0.0, b=0.0
	viewer->addSphere((*cloud_ptr)[0], 0.2, 1., 0.0, 0.0, "sphere");
	// 03 添加平面 ax+by+cz+d=0
	pcl::ModelCoefficients coeffs;
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	viewer->addPlane(coeffs, "plane");

	// 04 添加圆锥
	coeffs.values.clear();
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(5.0);
	viewer->addCone(coeffs, "cone");
}


void viewer_2(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_ptr,
	pcl::PointCloud<pcl::Normal>::ConstPtr normals1_ptr, pcl::PointCloud<pcl::Normal>::ConstPtr normals2_ptr) {
	int v1(0);
	// 指定左右两个角点的位置
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_ptr);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_ptr, rgb, "cloud1", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud_ptr, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_ptr, single_color, "cloud2", v2);

	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud_ptr, normals1_ptr, 2, 0.1, "normals1", v1);
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud_ptr, normals2_ptr, 2, 0.1, "normals2", v2);
}


void cloud_viewer() {
	// 03 设置其他渲染属性
	// 关闭警告，版本适配的问题
	viewer->getRenderWindow()->GlobalWarningDisplayOff();   // 关闭VTK9.1兼容警告
	viewer->setBackgroundColor(0, 0, 0);  // 背景颜色
	viewer->addCoordinateSystem(0.5);     // 坐标系大小
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud");
	
	// 显示点云
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		this_thread::sleep_for(100ms);
	}
	viewer->close();  // 必须关闭，否则会内存泄露
}


int main() {
        viewer.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
	// 01 生成示范点云
	make_data();

	// 02 接受键盘选择
	int s = 0;
	cout << "请选择选项（按enter确认）:" << endl;
	cout << "1: 简单显示" << endl;
	cout << "2: 彩色显示" << endl;
	cout << "3: 指定颜色" << endl;
	cout << "4: 深度渲染" << endl;
	cout << "5: 绘制法线" << endl;
	cout << "6: 绘制形状" << endl;
	cout << "7: 多个界面" << endl;
	cin >> s;
	switch (s)
	{
		// 简单显示
	case 1:
		viewer_simple(cloud);
		break;

		// 彩色显示
	case 2:
		viewer_rgb(cloud_rgb);
		break;

		// 指定颜色
	case 3:
		viewer_color(cloud);
		break;

		// 深度渲染
	case 4:
		viewer_depth(cloud);
		break;

		// 绘制法线
	case 5:
		viewer_normals(cloud_rgb, cloud_normals_1);
		break;

		// 绘制形状
	case 6:
		viewer_shape(cloud_rgb);
		break;
		// 多个窗口
	case 7:
		viewer_2(cloud_rgb, cloud_normals_1, cloud_normals_2);
		break;
	default:
		cout << "没有选项" << endl;
		break;
	}

	// x显示点云
	cloud_viewer();
	return 0;
}