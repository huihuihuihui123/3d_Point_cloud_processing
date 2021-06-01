#include <iostream>
#include <pcl/io/pcd_io.h>      //ioģ�� 
#include <pcl/point_types.h>   //��������

int
main(int argc, char** argv)
{
	//��������pcl::PointXYZ�����������ͣ��ֱ�Ϊ����cloud_a, cloud_b, ���cloud_c
	pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;

	// ������������
	//����cloud_a�ĸ���Ϊ5
	cloud_a.width = 5;
	cloud_a.height = cloud_b.height = 1; //���ö�Ϊ�������
	cloud_a.points.resize(cloud_a.width * cloud_a.height); //����
	cloud_b.width = 3;
	cloud_b.points.resize(cloud_b.width * cloud_b.height);
	
	//����ѭ�������������������涨����������͵ĵ�������
	for (size_t i = 0; i < cloud_a.points.size(); ++i)
	{  //cloud_a���������㣨ÿ���㶼��X Y Z �����������ֵ��
		cloud_a.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	for (size_t i = 0; i < cloud_b.points.size(); ++i)
	{   //�������a+b=c����cloud_b����������Ϊxyz������
		cloud_b.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_b.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_b.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	
	/*******************************************************************
    2�����루cloud_a cloud_b ��
	1�������cloud_c ��Ȼ�����Ϊ�����������cloud_a�� cloud_b

	********************************************************************/
	//���Cloud A
	std::cerr << "Cloud A: " << std::endl;
	for (size_t i = 0; i < cloud_a.points.size(); ++i)
		std::cerr << "    " << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << std::endl;
	//���Cloud B
	std::cerr << "Cloud B: " << std::endl;
	for (size_t i = 0; i < cloud_b.points.size(); ++i)
	{
		std::cerr << "    " << cloud_b.points[i].x << " " << cloud_b.points[i].y << " " << cloud_b.points[i].z << std::endl;
	}
	// Copy the point cloud data

	/***************************************�á�+�����ط�����Ƭ���Ƽ�����***********************************************************/
	cloud_c = cloud_a;
	cloud_c += cloud_b;//��cloud_a��cloud_b����һ�𴴽�cloud_c  �����
	std::cerr << "Cloud C: " << std::endl;
	for (size_t i = 0; i < cloud_c.points.size(); ++i)
	std::cerr << "    " << cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << " " << std::endl;

	getchar();
	return (0);
}