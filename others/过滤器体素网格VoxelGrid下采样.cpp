#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>//点云可视化
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read("table_scene_lms400.pcd", *cloud); // Remember to download the file first!

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";

	/******************************************************************************
	  创建一个叶大小为1cm的pcl::VoxelGrid滤波器，
	**********************************************************************************/
	  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //创建滤波对象
	  sor.setInputCloud (cloud);            //设置需要过滤的点云给滤波对象
	  sor.setLeafSize (0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
	  sor.filter (*cloud_filtered);           //执行滤波处理，存储输出


	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

	

	/**************************************************单视窗口+两片点云******************************************************/
	//boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("project points using  aparameyric model"));
	//viewer->setBackgroundColor(0, 0, 0); //创建窗口 
	//int v1;
	//viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
	////设置点云颜色 
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud, 34, 25, 133); //投影前可以随便设一个颜色
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_filtered, 255, 255, 255);  //投影后的设置为白色
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, source_color, "source", v1);
	//viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, target_color, "projected", v1);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "projected");
	//viewer->spin();
	/**************************************************双视窗口*************************************************************/
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("filter Viewer"));
	viewer->initCameraParameters();
	int v1(0), v2(0);//视口编号在这里设置两个视口
	//5.1原始点云窗口
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("original", 10, 10, "v1 text", v1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud1", v1);
	viewer->addCoordinateSystem(1.0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud1");
	//5.2滤波窗口
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0, 0, 0, v2);
	viewer->addText("after filtered", 10, 10, "v2 text", v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "sample cloud2", v2);
	viewer->addCoordinateSystem(1.0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud2");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);  //刷新
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}