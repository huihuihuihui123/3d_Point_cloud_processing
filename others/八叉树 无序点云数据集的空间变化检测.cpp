#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#include <iostream>
#include <vector>
#include <ctime>

int
main(int argc, char** argv)
{
	srand((unsigned int)time(NULL));  //��ϵͳʱ���ʼ���������

	// �˲����ķֱ��ʣ������صĴ�С
	float resolution = 32.0f;

	// ��ʼ���ռ�仯������
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);  //��������ʵ��cloudA���ɵĵ����������ڽ����˲���octree����

	// ΪcloudA������������
	cloudA->width = 128;                      //���õ���cloudA�ĵ���
	cloudA->height = 1;                          //�����
	cloudA->points.resize(cloudA->width * cloudA->height);   //����

	for (size_t i = 0; i < cloudA->points.size(); ++i)         //ѭ�����
	{
		cloudA->points[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
		cloudA->points[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
		cloudA->points[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
	}

	// ��ӵ��Ƶ��˲����У������˲���
	octree.setInputCloud(cloudA);  //�����������
	octree.addPointsFromInputCloud();   //��������ƹ����˲���
	 /***********************************************************************************
	  ����cloudA�ǲο��������佨���İ˲��������������Ŀռ�ֲ���octreePointCloudChangeDetector
	  ��̳���Octree2BufBae�࣬Octree2BufBae������ͬʱ���ڴ��б���͹�������octree��������Ӧ�����ڴ��
	  �û����ܹ����������Ѿ������˵Ľڵ������˼����������ɵ��ư˲�������ʱ������ڴ������ͷŲ���
	  ͨ������ octree.switchBuffers ()���ð˲��� octree����Ļ�����������֮ǰ��octree������Ȼ�������ڴ���
	 ************************************************************************************/
	 // �����˲����Ļ��壬����CloudA��Ӧ�İ˲����ṹ��Ȼ���ڴ���
	octree.switchBuffers();
	//cloudB�������ڽ����µİ˲����ṹ����ǰһ��cloudA��Ӧ�İ˲�������octree����ͬʱ���ڴ���פ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);   //ʵ�������ƶ���cloudB

	// ΪcloudB�������� 
	cloudB->width = 128;
	cloudB->height = 1;
	cloudB->points.resize(cloudB->width * cloudB->height);

	for (size_t i = 0; i < cloudB->points.size(); ++i)
	{
		cloudB->points[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
		cloudB->points[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
		cloudB->points[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
	}

	// ���cloudB���˲�����
	octree.setInputCloud(cloudB);
	octree.addPointsFromInputCloud();

	/**************************************************************************************************************
	Ϊ�˼�����ȡ������couodB�ĵ㼯R����R��û��cloudA�е�Ԫ�أ����Ե���getPointIndicesFromNewVoxels������ͨ��̽�������˲���֮��
	���صĲ�ͬ��������cloudB ���¼ӵ��������������ͨ�������������Ի�ȡR�㼯  ������������̽����cloudB�����cloudA�仯�ĵ㼯������ֻ��̽��
	����cloudA�����ӵĵ㼯��������̽����ٵ�
  ****************************************************************************************************************/


	std::vector<int> newPointIdxVector;  //�洢����ӵ�����������

	// ��ȡǰһcloudA��Ӧ�˲�����cloudB��Ӧ�ڰ˲�����û�еĵ㼯
	octree.getPointIndicesFromNewVoxels(newPointIdxVector);

	// ��ӡ�㼯
	std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
	for (size_t i = 0; i < newPointIdxVector.size(); ++i)
		std::cout << i << "# Index:" << newPointIdxVector[i]
		<< "  Point:" << cloudB->points[newPointIdxVector[i]].x << " "
		<< cloudB->points[newPointIdxVector[i]].y << " "
		<< cloudB->points[newPointIdxVector[i]].z << std::endl;
	getchar();
}