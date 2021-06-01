#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>//���ƿ��ӻ�

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read("turn_shuiguan360.pcd", *cloud); // Remember to download the file first!
	reader.read("boudaryvoxl0.3-neark.pcd", *cloud_filtered); // Remember to download the file first!
	std::cerr << "PointCloud before filtering: " << cloud->size() << endl;
	std::cerr << "PointCloud after filtering: " << cloud_filtered->size() << endl;


	/**************************************************˫�Ӵ���*************************************************************/
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("filter Viewer"));
	viewer->initCameraParameters();
	int v1(0), v2(0);//�ӿڱ�����������������ӿ�
	//5.1ԭʼ���ƴ���
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("original", 10, 10, "v1 text", v1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud1", v1);
	//viewer->addCoordinateSystem(1.0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud1");
	//5.2�˲�����
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0, 0, 0, v2);
	viewer->addText("after filtered", 10, 10, "v2 text", v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "sample cloud2", v2);
	//viewer->addCoordinateSystem(1.0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud2");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);  //ˢ��
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}