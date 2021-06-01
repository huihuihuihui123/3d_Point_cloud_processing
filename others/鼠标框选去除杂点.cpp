#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>//点云可视化
#include<algorithm>//sort函数排序

bool iteration_flag = false;//键盘按键全局变量

std::vector<std::string> cloudNameVector;
std::vector< int > indices;			//选中点云在原点云中的索引

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Radius_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Radius_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Statictics_filtered(new pcl::PointCloud<pcl::PointXYZ>);

int num = 0;
pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("filter Viewer"));

//点云框选事件函数
void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{

	if (event.getPointsIndices(indices) == -1)
		return;
	for (int i = 0; i < indices.size(); ++i)
	{
		clicked_points_3d->points.push_back(cloud->points.at(indices[i]));
		cout << clicked_points_3d->points[i].x << "," << clicked_points_3d->points[i].y << "," << clicked_points_3d->points[i].z << endl;
	}
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);
	std::stringstream ss;
	std::string cloudName;
	ss << num++;
	ss >> cloudName;
	cloudName += "_cloudName";
	cloudNameVector.push_back(cloudName);
	viewer->addPointCloud(clicked_points_3d, red, cloudName);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing) {
	if (event.getKeySym() == "space" && event.keyDown()) {
		iteration_flag = true;
		clicked_points_3d->clear();
		for (int i = 0; i < cloudNameVector.size(); i++)
		{
			viewer->removePointCloud(cloudNameVector[i]);
		}
		std::sort(indices.begin(), indices.end());
		
		for (int i = 0; i < indices.size(); i++)
		{
			//std::vector < pcl::PointXYZ >::iterator iter = cloud->points.begin();
			//cloud->points.erase(iter + indices[i] + 1 - i);
			cout << indices[i] << "  " << endl;
		}

	/*
		viewer->spinOnce(100);*/
	}
}





int
main(int argc, char** argv)
{

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read("水管拟合1.pcd", *cloud); // Remember to download the file first!

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ")." << endl;


	/**************************************************单视窗口******************************************************/
	viewer->setBackgroundColor(0, 0, 0); //创建窗口
	//设置点云颜色 
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud, 255, 255, 255); //投影前可以随便设一个颜色
	viewer->addPointCloud<pcl::PointXYZ>(cloud, source_color, "source");
	viewer->registerAreaPickingCallback(pp_callback, (void*)&cloud);									//框选回调函数，机制类似于中断优先级
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&cloud);					//删除点云鼠标回调函数
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}