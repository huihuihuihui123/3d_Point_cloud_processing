#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
	//读取文件
	PCDReader reader;
	PCDWriter writer;
	PointCloud<pcl::PointXYZ>::Ptr cloud(new PointCloud<pcl::PointXYZ>);
	PointCloud<pcl::PointXYZ>::Ptr cloud_f(new PointCloud<pcl::PointXYZ>);
	reader.read("G:/mm/2/FinalPCD.pcd", *cloud);

	//我们使用平面模型，其中ax + by + cz + d = 0，其中a = b = d = 0，并且c = 1，或者换句话说，XY平面。
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	//我们创建投影对象，并使用上面定义的模型作为投影模型
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_f);

	//可视化
	pcl::visualization::PCLVisualizer viewer("cloud_viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud_f, 255, 255, 255);
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud(cloud_f);
	//viewer.addCoordinateSystem();
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return(0);


}