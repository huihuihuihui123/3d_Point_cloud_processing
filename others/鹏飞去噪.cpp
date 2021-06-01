#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>//点云可视化
#include <pcl/filters/voxel_grid.h>
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_VoxelGrid_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	// 定义读取对象
	pcl::PCDReader reader;
	// 读取点云文件
	reader.read<pcl::PointXYZRGB>("1.pcd", *cloud);

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << cloud->size() << std::endl;

	// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
	 //个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;   //创建滤波器对象
	sor.setInputCloud(cloud);                           //设置待滤波的点云
	sor.setMeanK(100);                               //设置在进行统计时考虑查询点临近点数
	sor.setStddevMulThresh(0.1);                      //设置判断是否为离群点的阀值
	sor.filter(*cloud_filtered);    //存储



	pcl::VoxelGrid<pcl::PointXYZRGB> VoxelGridsor;  //创建滤波对象
	VoxelGridsor.setInputCloud(cloud_filtered);            //设置需要过滤的点云给滤波对象
	VoxelGridsor.setLeafSize(0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
	VoxelGridsor.filter(*cloud_VoxelGrid_filtered);           //执行滤波处理，存储输出

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZRGB>("111.pcd", *cloud_VoxelGrid_filtered);
	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << cloud_VoxelGrid_filtered->size() << std::endl;


	/**************************************************双视窗口*************************************************************/
	pcl::visualization::PCLVisualizer viewer("filter Viewer");
	viewer.initCameraParameters();
	int v1(0), v2(0);//视口编号在这里设置两个视口
	//5.1原始点云窗口
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.setBackgroundColor(0, 0, 0, v1);
	viewer.addText("original", 10, 10, "v1 text", v1);
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud1", v1);
	viewer.addCoordinateSystem(1.0);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
	//5.2滤波窗口
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor(0, 0, 0, v2);
	viewer.addText("after filtered", 10, 10, "v2 text", v2);
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud_filtered, "sample cloud2", v2);
	viewer.addCoordinateSystem(1.0);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();  //刷新
	}
	//pcl::PCDWriter writer;
	//writer.write<pcl::PointXYZ>("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

	//sor.setNegative(true);
	//sor.filter(*cloud_filtered);
	//writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

	return (0);
}

//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/visualization/pcl_visualizer.h>//点云可视化
//int
//main(int argc, char** argv)
//{
//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
//
//	// Fill in the cloud data
//	pcl::PCDReader reader;
//	// Replace the path below with the path where you saved your file
//	reader.read("1.pcd", *cloud); // Remember to download the file first!
//
//	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
//		<< " data points (" << pcl::getFieldsList(*cloud) << ").";
//
//	/******************************************************************************
//	  创建一个叶大小为1cm的pcl::VoxelGrid滤波器，
//	**********************************************************************************/
//	pcl::VoxelGrid<pcl::PointXYZRGBA> sor;  //创建滤波对象
//	sor.setInputCloud(cloud);            //设置需要过滤的点云给滤波对象
//	sor.setLeafSize(0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
//	sor.filter(*cloud_filtered);           //执行滤波处理，存储输出
//
//
//	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
//		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";
//
//
//
//	/**************************************************单视窗口+两片点云******************************************************/
//	//boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("project points using  aparameyric model"));
//	//viewer->setBackgroundColor(0, 0, 0); //创建窗口 
//	//int v1;
//	//viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
//	////设置点云颜色 
//	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud, 34, 25, 133); //投影前可以随便设一个颜色
//	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_filtered, 255, 255, 255);  //投影后的设置为白色
//	//viewer->addPointCloud<pcl::PointXYZ>(cloud, source_color, "source", v1);
//	//viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, target_color, "projected", v1);
//	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
//	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "projected");
//	//viewer->spin();
//	/**************************************************双视窗口*************************************************************/
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("filter Viewer"));
//	viewer->initCameraParameters();
//	int v1(0), v2(0);//视口编号在这里设置两个视口
//	//5.1原始点云窗口
//	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//	viewer->setBackgroundColor(0, 0, 0, v1);
//	viewer->addText("original", 10, 10, "v1 text", v1);
//	viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, "sample cloud1", v1);
//	viewer->addCoordinateSystem(1.0);
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud1");
//	//5.2滤波窗口
//	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//	viewer->setBackgroundColor(0, 0, 0, v2);
//	viewer->addText("after filtered", 10, 10, "v2 text", v2);
//	viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_filtered, "sample cloud2", v2);
//	viewer->addCoordinateSystem(1.0);
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud2");
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce(100);  //刷新
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//}