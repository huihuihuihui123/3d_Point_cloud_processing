
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>//fpfh����������ͷ�ļ�
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

int main()
{
	//1.���ص���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("mls_points.pcd", *cloud);

	//2.���Ʒ�������
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	//����һ���յ�kdtree���󣬲��������ݸ����߹��ƶ���
	//���ڸ������������ݼ���kdtree��������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr Netree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(Netree);
	//������ݼ�
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	//ʹ�ð뾶�ڲ�ѯ����Χ3���׷�Χ�ڵ�������Ԫ��
	ne.setRadiusSearch(0.03);
	//��������ֵ
	ne.compute(*cloud_normals);
	//3.����PFH���ƶ���pfh����������������ݼ�cloud�ͷ���normals���ݸ���

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> Fpfh;

	Fpfh.setInputCloud(cloud);

	Fpfh.setInputNormals(cloud_normals);

	//�������������ΪPointNormal,��ִ��pfh.setInputNormals (cloud);

//4.����һ���յ�kd����ʾ�������������ݸ�PFH���ƶ���

	//�����Ѹ����������ݼ�������kdtree

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	Fpfh.setSearchMethod(tree);

	//������ݼ�

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

	//ʹ�ð뾶��5���׷�Χ�ڵ�������Ԫ�ء�

	//ע�⣺�˴�ʹ�õİ뾶����Ҫ���ڹ��Ʊ��淨��ʱʹ�õİ뾶!!!

	Fpfh.setRadiusSearch(0.05);

	//����pfh����ֵ

	Fpfh.compute(*pfhs);

	// pfhs->points.size ()Ӧ����input cloud->points.size ()����ͬ�Ĵ�С����ÿ���㶼��һ��pfh��������
	cout << pfhs->points.size() << endl;
	cout << cloud->points.size() << endl;
	getchar();
	return 0;
}