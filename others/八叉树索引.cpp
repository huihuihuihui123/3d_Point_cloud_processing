#include <pcl/point_cloud.h>   //����ͷ�ļ�
#include <pcl/octree/octree.h>  //�˲���ͷ�ļ�

#include <iostream>
#include <vector>
#include <ctime>

//Octree��ؼ����˵��
//PCL octree����ṩ�˼���octree���ͣ����Ǹ��Ե�Ҷ�ڵ������������ǲ�ͬ�ģ�
//
//OctreePointCloudVector(����OctreePointCloud)����octree�ܹ�����ÿһ���ڵ��ϵĵ������С�
//
//OctreePointCloudSinglePoint : ��octree���������ÿһ���ڵ��ϵĵ���������������������������Ҷ�ڵ�ĵ�����
//
//	OctreePointCloudOccupancy : ��octree�಻�洢����Ҷ�ڵ��ϵ��κ���Ϣ���������ڿռ����������
//
//	OctreePointCloudDensity : �洢ÿһ��Ҷ�ڵ������е����Ŀ�������Խ��пռ�㼯�ܼ��̶ȵĲ�ѯ

int
main(int argc, char** argv)
{
	srand((unsigned int)time(NULL));   //��ϵͳʱ���ʼ����������� srand (time (NULL))������

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// ������������
	cloud->width = 1000;
	cloud->height = 1;               //����
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)   //���ѭ���������Ƶ�����ֵ
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}
	/****************************************************************************
	����һ��octreeʵ���������÷ֱ��ʽ��г�ʼ������octree������ҳ�ڵ��ŵ�����������
	�ֱ��ʲ����������һ��octree����С���صĳߴ磬���octree������Ƿֱ��ʺ͵��ƿռ�ά��
	�ĺ��������֪�����Ƶı߽��Ӧ����defineBoundingbox�������������octreeȻ��ͨ��
	����ָ������е����ӵ�ctree�С�
	*****************************************************************************/
	float resolution = 128.0f;//���������С

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);   //��ʼ��Octree

	octree.setInputCloud(cloud);         //�����������    ����������ؼ��Ľ���PointCloud��octree֮�����ϵ
	octree.addPointsFromInputCloud();   //����octree

	pcl::PointXYZ searchPoint;   //����searchPoint

	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	/*************************************************************************************
	һ��PointCloud��octree��ϵһ�𣬾��ܽ�����������������ʹ�õ��ǡ����ؽ������������Ѳ�ѯ������������
	 �������������Ϊ��ѯ������أ�����Ե�������������ʽ���棬�����������������֮��ľ���ȡ����octree�ķֱ��ʲ���
  *****************************************************************************************/

	std::vector<int> pointIdxVec;            //�洢���ؽ��������������

	if (octree.voxelSearch(searchPoint, pointIdxVec))    //ִ�����������ؽ����pointIdxVec
	{
		std::cout << "Neighbors within voxel search at (" << searchPoint.x
			<< " " << searchPoint.y
			<< " " << searchPoint.z << ")"
			<< std::endl;

		for (size_t i = 0; i < pointIdxVec.size(); ++i)                  //��ӡ���������
			std::cout << "    " << cloud->points[pointIdxVec[i]].x
			<< " " << cloud->points[pointIdxVec[i]].y
			<< " " << cloud->points[pointIdxVec[i]].z << std::endl;
	}

	/**********************************************************************************
	 K ������Ϊ10 ��K��������  �������������д�������ֿ�����������һ��pointIdxNKNSearch�����������
	  ��������������������  �ڶ�������pointNKNSquaredDistance�洢�����������֮��ľ����ƽ����
   *************************************************************************************/

   //K ��������
	int K = 10;

	std::vector<int> pointIdxNKNSearch;   //����������������
	std::vector<float> pointNKNSquaredDistance;            //�����������֮��ľ����ƽ��

	std::cout << "K nearest neighbor search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with K=" << K << std::endl;

	if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
			<< " " << cloud->points[pointIdxNKNSearch[i]].y
			<< " " << cloud->points[pointIdxNKNSearch[i]].z
			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}

	// �뾶�ڽ�������

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

	std::cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << std::endl;


	if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}
	getchar();

}