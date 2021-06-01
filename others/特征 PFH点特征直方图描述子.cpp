#include <pcl/point_types.h>                  //������ͷ�ļ�
#include <pcl/features/pfh.h>                 //pfh����������ͷ�ļ�
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

//struct PFHSignature125
//{
//	float histogram[125];
//};
//Ĭ�ϵ�PFHʵ��ʹ��5������ϸ�֣����磬�ĸ�����ֵ�е�ÿһ��������ֵ���ʹ�����������䣩��
//���Ҳ��������루�������� - ���� �û����Ե���computePairFeatures���������Ҫ��Ҳ��þ��룬�⵼��5 ^ 3����ֵ��125�ֽ����飨�������Ǵ洢��pcl::PFHSignature125�������С�
int main()
{
	//1.���ص���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("table_scene_lms400_downsampled.pcd", *cloud);

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

	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;

	pfh.setInputCloud(cloud);

	pfh.setInputNormals(cloud_normals);

		//�������������ΪPointNormal,��ִ��pfh.setInputNormals (cloud);

	//4.����һ���յ�kd����ʾ�������������ݸ�PFH���ƶ���

		//�����Ѹ����������ݼ�������kdtree

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pfh.setSearchMethod(tree);

	//������ݼ�

	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());

	//ʹ�ð뾶��5���׷�Χ�ڵ�������Ԫ�ء�

	//ע�⣺�˴�ʹ�õİ뾶����Ҫ���ڹ��Ʊ��淨��ʱʹ�õİ뾶!!!

	pfh.setRadiusSearch(0.05);

	//����pfh����ֵ

	pfh.compute(*pfhs);

	// pfhs->points.size ()Ӧ����input cloud->points.size ()����ͬ�Ĵ�С����ÿ���㶼��һ��pfh��������
	cout << pfhs->points.size() << endl;
	cout << cloud->points.size() << endl;
	getchar();
	return 0;
}
