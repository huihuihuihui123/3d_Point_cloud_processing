#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/filters/extract_indices.h>//提取索引
#include <iostream>
#include <vector>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr indices_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr out_indices_cloud(new pcl::PointCloud<pcl::PointXYZ>());
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

int num = 0;

void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
	std::vector< int > indices;
	if (event.getPointsIndices(indices) == -1)
		return;

	inliers->indices = indices;
	// 设置ExtractIndices的实际参数
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*out_indices_cloud);
	extract.setNegative(false);
	extract.filter(*indices_cloud);
	/*for (int i = 0; i < indices.size(); ++i)
	{
		clicked_points_3d->points.push_back(cloud->points.at(indices[i]));
	}*/


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(indices_cloud, 255, 0, 0);

	std::stringstream ss;
	std::string cloudName;
	ss << num++;
	ss >> cloudName;
	cloudName += "_cloudName";

	viewer->addPointCloud(indices_cloud, red, cloudName);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
}
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing) {
	if (event.getKeySym() == "space" && event.keyDown()&& out_indices_cloud->size()!=0) {
		*cloud = *out_indices_cloud;
		viewer->removeAllPointClouds();
		viewer->addPointCloud(cloud);
	}
}
void main()
{
	if (pcl::io::loadPCDFile("bunny.pcd", *cloud))
	{
		std::cerr << "ERROR: Cannot open file " << std::endl;
		return;
	}
	
	viewer->addPointCloud(cloud, "bunny");
	viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
	viewer->registerAreaPickingCallback(pp_callback, (void*)&cloud);
	viewer->registerKeyboardCallback(keyboardEventOccurred);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
