#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>//点云可视化
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read("mls_points.pcd", *cloud); // Remember to download the file first!
	cout << cloud->size() << endl;

	/**************************************************单视窗口+两片点云******************************************************/
	boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("project points using  aparameyric model"));
	viewer->setBackgroundColor(0, 0, 0); //创建窗口 
	viewer->initCameraParameters();
	int v1;
	viewer->createViewPort(0.0, 0.0, 1, 1, v1);
	//设置点云颜色 
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud, 0, 255, 255); //投影前可以随便设一个颜色
	viewer->addPointCloud<pcl::PointXYZ>(cloud, source_color, "source", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();  //刷新;
	}
	return 0;
}