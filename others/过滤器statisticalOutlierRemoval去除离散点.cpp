#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>//���ƿ��ӻ�
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// �����ȡ����
	pcl::PCDReader reader;
	// ��ȡ�����ļ�
	reader.read<pcl::PointXYZRGBA>("1.pcd", *cloud);

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// �����˲�������ÿ����������ٽ���ĸ�������Ϊ50 ��������׼��ı�������Ϊ1  ����ζ�����һ
	 //����ľ��볬����ƽ������һ����׼�����ϣ���õ㱻���Ϊ��Ⱥ�㣬�������Ƴ����洢����
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;   //�����˲�������
	sor.setInputCloud(cloud);                           //���ô��˲��ĵ���
	sor.setMeanK(50);                               //�����ڽ���ͳ��ʱ���ǲ�ѯ���ٽ�����
	sor.setStddevMulThresh(1.0);                      //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ
	sor.filter(*cloud_filtered);                    //�洢

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;


	/**************************************************˫�Ӵ���*************************************************************/
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("filter Viewer"));
	viewer->initCameraParameters();
	int v1(0), v2(0);//�ӿڱ�����������������ӿ�
	//5.1ԭʼ���ƴ���
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("original", 10, 10, "v1 text", v1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud1", v1);
	viewer->addCoordinateSystem(1.0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud1");
	//5.2�˲�����
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0, 0, 0, v2);
	viewer->addText("after filtered", 10, 10, "v2 text", v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "sample cloud2", v2);
	viewer->addCoordinateSystem(1.0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud2");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);  //ˢ��
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	//pcl::PCDWriter writer;
	//writer.write<pcl::PointXYZ>("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

	//sor.setNegative(true);
	//sor.filter(*cloud_filtered);
	//writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

	return (0);
}