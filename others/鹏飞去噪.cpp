#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>//���ƿ��ӻ�
#include <pcl/filters/voxel_grid.h>
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_VoxelGrid_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	// �����ȡ����
	pcl::PCDReader reader;
	// ��ȡ�����ļ�
	reader.read<pcl::PointXYZRGB>("1.pcd", *cloud);

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << cloud->size() << std::endl;

	// �����˲�������ÿ����������ٽ���ĸ�������Ϊ50 ��������׼��ı�������Ϊ1  ����ζ�����һ
	 //����ľ��볬����ƽ������һ����׼�����ϣ���õ㱻���Ϊ��Ⱥ�㣬�������Ƴ����洢����
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;   //�����˲�������
	sor.setInputCloud(cloud);                           //���ô��˲��ĵ���
	sor.setMeanK(100);                               //�����ڽ���ͳ��ʱ���ǲ�ѯ���ٽ�����
	sor.setStddevMulThresh(0.1);                      //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ
	sor.filter(*cloud_filtered);    //�洢



	pcl::VoxelGrid<pcl::PointXYZRGB> VoxelGridsor;  //�����˲�����
	VoxelGridsor.setInputCloud(cloud_filtered);            //������Ҫ���˵ĵ��Ƹ��˲�����
	VoxelGridsor.setLeafSize(0.01f, 0.01f, 0.01f);  //�����˲�ʱ�������������Ϊ1cm��������
	VoxelGridsor.filter(*cloud_VoxelGrid_filtered);           //ִ���˲������洢���

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZRGB>("111.pcd", *cloud_VoxelGrid_filtered);
	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << cloud_VoxelGrid_filtered->size() << std::endl;


	/**************************************************˫�Ӵ���*************************************************************/
	pcl::visualization::PCLVisualizer viewer("filter Viewer");
	viewer.initCameraParameters();
	int v1(0), v2(0);//�ӿڱ�����������������ӿ�
	//5.1ԭʼ���ƴ���
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.setBackgroundColor(0, 0, 0, v1);
	viewer.addText("original", 10, 10, "v1 text", v1);
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud1", v1);
	viewer.addCoordinateSystem(1.0);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
	//5.2�˲�����
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor(0, 0, 0, v2);
	viewer.addText("after filtered", 10, 10, "v2 text", v2);
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud_filtered, "sample cloud2", v2);
	viewer.addCoordinateSystem(1.0);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();  //ˢ��
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
//#include <pcl/visualization/pcl_visualizer.h>//���ƿ��ӻ�
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
//	  ����һ��Ҷ��СΪ1cm��pcl::VoxelGrid�˲�����
//	**********************************************************************************/
//	pcl::VoxelGrid<pcl::PointXYZRGBA> sor;  //�����˲�����
//	sor.setInputCloud(cloud);            //������Ҫ���˵ĵ��Ƹ��˲�����
//	sor.setLeafSize(0.01f, 0.01f, 0.01f);  //�����˲�ʱ�������������Ϊ1cm��������
//	sor.filter(*cloud_filtered);           //ִ���˲������洢���
//
//
//	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
//		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";
//
//
//
//	/**************************************************���Ӵ���+��Ƭ����******************************************************/
//	//boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("project points using  aparameyric model"));
//	//viewer->setBackgroundColor(0, 0, 0); //�������� 
//	//int v1;
//	//viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
//	////���õ�����ɫ 
//	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud, 34, 25, 133); //ͶӰǰ���������һ����ɫ
//	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_filtered, 255, 255, 255);  //ͶӰ�������Ϊ��ɫ
//	//viewer->addPointCloud<pcl::PointXYZ>(cloud, source_color, "source", v1);
//	//viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, target_color, "projected", v1);
//	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
//	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "projected");
//	//viewer->spin();
//	/**************************************************˫�Ӵ���*************************************************************/
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("filter Viewer"));
//	viewer->initCameraParameters();
//	int v1(0), v2(0);//�ӿڱ�����������������ӿ�
//	//5.1ԭʼ���ƴ���
//	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//	viewer->setBackgroundColor(0, 0, 0, v1);
//	viewer->addText("original", 10, 10, "v1 text", v1);
//	viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, "sample cloud1", v1);
//	viewer->addCoordinateSystem(1.0);
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud1");
//	//5.2�˲�����
//	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//	viewer->setBackgroundColor(0, 0, 0, v2);
//	viewer->addText("after filtered", 10, 10, "v2 text", v2);
//	viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_filtered, "sample cloud2", v2);
//	viewer->addCoordinateSystem(1.0);
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud2");
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce(100);  //ˢ��
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//}