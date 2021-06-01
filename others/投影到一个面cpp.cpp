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
	//��ȡ�ļ�
	PCDReader reader;
	PCDWriter writer;
	PointCloud<pcl::PointXYZ>::Ptr cloud(new PointCloud<pcl::PointXYZ>);
	PointCloud<pcl::PointXYZ>::Ptr cloud_f(new PointCloud<pcl::PointXYZ>);
	reader.read("G:/mm/2/FinalPCD.pcd", *cloud);

	//����ʹ��ƽ��ģ�ͣ�����ax + by + cz + d = 0������a = b = d = 0������c = 1�����߻��仰˵��XYƽ�档
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	//���Ǵ���ͶӰ���󣬲�ʹ�����涨���ģ����ΪͶӰģ��
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_f);

	//���ӻ�
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