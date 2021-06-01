#include <pcl/point_cloud.h>        //�����Ͷ���ͷ�ļ�
#include <pcl/kdtree/kdtree_flann.h> //kdtree�ඨ��ͷ�ļ�

#include <iostream>
#include <vector>
#include <ctime>

int
main(int argc, char** argv)
{
	srand(time(NULL));   //��ϵͳʱ���ʼ���������
	//����һ��PointCloud<pcl::PointXYZ>
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// �����������
	cloud->width = 1000;             //�˴���������
	cloud->height = 1;                //��ʾ����Ϊ�������
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)   //ѭ������������
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}
	//����KdTreeFLANN���󣬲��Ѵ����ĵ�������Ϊ����,����һ��searchPoint������Ϊ��ѯ��
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	//���������ռ�
	kdtree.setInputCloud(cloud);
	//���ò�ѯ�㲢�����ֵ
	pcl::PointXYZ searchPoint;
	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	// K �ٽ�����
	//����һ������������Ϊ10���������������洢��������K���ڣ����������У�һ���洢��������ѯ����ڵ���������һ���洢��Ӧ���ڵľ���ƽ��
	int K = 10;

	std::vector<int> pointIdxNKNSearch(K);      //�洢��ѯ���������
	std::vector<float> pointNKNSquaredDistance(K); //�洢���ڵ��Ӧ����ƽ��
	//��ӡ�����Ϣ
	std::cout << "K nearest neighbor search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with K=" << K << std::endl;

	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)  //ִ��K��������
	{
		//��ӡ���н�������
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
			<< " " << cloud->points[pointIdxNKNSearch[i]].y
			<< " " << cloud->points[pointIdxNKNSearch[i]].z
			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}
	/**********************************************************************************
	 ����Ĵ���չʾ���ҵ�������searchPoint��ĳһ�뾶����������������н��ڣ����¶�����������
	 pointIdxRadiusSearch  pointRadiusSquaredDistance���洢���ڽ��ڵ���Ϣ
	 ********************************************************************************/
	 // �뾶 R�ڽ�����������

	std::vector<int> pointIdxRadiusSearch;           //�洢��������
	std::vector<float> pointRadiusSquaredDistance;   //�洢���ڶ�Ӧ�����ƽ��

	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);   //���������ĳһ�뾶
	//��ӡ���
	std::cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << std::endl;


	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)  //ִ�а뾶R�ڽ�����������
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			<< " (squared distance: " <<pointRadiusSquaredDistance[i] << ")" << std::endl;
	}

	getchar();
	return 0;
}