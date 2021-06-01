#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>//���ƿ��ӻ�
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read("mls_points.pcd", *cloud); // Remember to download the file first!
	cout << cloud->size() << endl;

	/**************************************************���Ӵ���+��Ƭ����******************************************************/
	boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("project points using  aparameyric model"));
	viewer->setBackgroundColor(0, 0, 0); //�������� 
	viewer->initCameraParameters();
	int v1;
	viewer->createViewPort(0.0, 0.0, 1, 1, v1);
	//���õ�����ɫ 
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud, 0, 255, 255); //ͶӰǰ���������һ����ɫ
	viewer->addPointCloud<pcl::PointXYZ>(cloud, source_color, "source", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();  //ˢ��;
	}
	return 0;
}