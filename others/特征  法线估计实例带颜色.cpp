#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>  //���߹�����ͷ�ļ�
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

//��ʵ�ֶ�����������ݼ��еĵ����һ����淨�ߣ�ִ�еĲ����ǣ�
//��Ӧ����P��ÿһ����p�õ�p�������Ԫ�أ�����p��ı���ķ���N�����N�ķ����Ƿ�ָ���ӵ����������ת��
int
main()
{
	//�򿪵��ƴ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("table_scene_lms400.pcd", *cloud);
	//�������߹��ƹ�������
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	//����һ���յ�KdTree���󣬲��������ݸ����߹�������
	//���ڸ������������ݼ���KdTree��������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	//�洢�������
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	//ʹ�ð뾶�ڲ�ѯ����Χ3���׷�Χ�ڵ������ٽ�Ԫ��
	//ne.setRadiusSearch(0.03);
	ne.setKSearch(10);//Ѱ�Ҿ��������10������Ϊ
	//��������ֵ
	ne.compute(*cloud_normals);

	// cloud_normals->points.size ()Ӧ����input cloud_downsampled->points.size ()����ͬ�ĳߴ�
	//���ӻ�
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud, 0, 0, 255);
	viewer->setBackgroundColor(0.0, 0.0, 0.0);
	viewer->addPointCloud(cloud, source_cloud_color_handler, "original_cloud");
	cout << cloud->points.size() << endl;
	cout << cloud_normals->points.size() << endl;
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals);


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	return 0;
}