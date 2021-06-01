#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>//点云可视化
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// 定义读取对象
	pcl::PCDReader reader;
	// 读取点云文件
	reader.read<pcl::PointXYZRGBA>("1.pcd", *cloud);

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
	 //个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;   //创建滤波器对象
	sor.setInputCloud(cloud);                           //设置待滤波的点云
	sor.setMeanK(50);                               //设置在进行统计时考虑查询点临近点数
	sor.setStddevMulThresh(1.0);                      //设置判断是否为离群点的阀值
	sor.filter(*cloud_filtered);                    //存储

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;


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
	//pcl::PCDWriter writer;
	//writer.write<pcl::PointXYZ>("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

	//sor.setNegative(true);
	//sor.filter(*cloud_filtered);
	//writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

	return (0);
}