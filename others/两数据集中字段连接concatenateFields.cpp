#include <iostream>
#include <pcl/io/pcd_io.h>      //ioģ�� 
#include <pcl/point_types.h>   //��������

int
main(int argc, char** argv)
{
	//��������pcl::PointXYZ�����������ͣ��ֱ�Ϊcloud_a, cloud_b, cloud_c
	pcl::PointCloud<pcl::PointXYZ> cloud_a;
	//�洢��������ʱ��Ҫ��Normal����,Normal (float n_x, float n_y, float n_z)
	pcl::PointCloud<pcl::Normal> n_cloud_b; // ����n_x��n_y��n_z
	//�洢����XYZ��normal��ĵ���
	pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;

	// ������������
	//����cloud_a�ĸ���Ϊ5
	cloud_a.width = 5;
	cloud_a.height  = n_cloud_b.height = 1; //���ö�Ϊ�������
	cloud_a.points.resize(cloud_a.width * cloud_a.height); //����
	
	n_cloud_b.width = 5; //���������XYZ��normal������5�����ߣ��ֶμ����ӣ�
	n_cloud_b.points.resize(n_cloud_b.width * n_cloud_b.height);
	
	//����ѭ�������������������涨����������͵ĵ�������
	for (size_t i = 0; i < cloud_a.points.size(); ++i)
	{  //cloud_a���������㣨ÿ���㶼��X Y Z �����������ֵ��
		cloud_a.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	for (size_t i = 0; i < n_cloud_b.points.size(); ++i)
	{  //�������xyz+normal=xyznormal��n_cloud_b��5������Ϊnormal����
		n_cloud_b.points[i].normal[0] = 1024 * rand() / (RAND_MAX + 1.0f);
		n_cloud_b.points[i].normal[1] = 1024 * rand() / (RAND_MAX + 1.0f);
		n_cloud_b.points[i].normal[2] = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	/*******************************************************************
	���������ӵ��ƻ��õ���2�����ƶ���2�����루cloud_a ��n_cloud_b��
	1�������n_cloud_c��Ȼ�����Ϊ�����������cloud_a ��n_cloud_b�������

	********************************************************************/
	//���Cloud A
	std::cerr << "Cloud A: " << std::endl;
	for (size_t i = 0; i < cloud_a.points.size(); ++i)
		std::cerr << "    " << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << std::endl;
	//���Cloud B
	std::cerr << "Cloud B: " << std::endl;
		for (size_t i = 0; i < n_cloud_b.points.size(); ++i)
			std::cerr << "    " << n_cloud_b.points[i].normal[0] << " " << n_cloud_b.points[i].normal[1] << " " << n_cloud_b.points[i].normal[2] << std::endl;

	// Copy the point cloud data
	
	//�����ֶ�  ��cloud_a�� n_cloud_b�ֶ����� һ�𴴽� p_n_cloud_c)
	pcl::concatenateFields(cloud_a, n_cloud_b, p_n_cloud_c);
	std::cerr << "Cloud C: " << std::endl;
	for (size_t i = 0; i < p_n_cloud_c.points.size(); ++i)
		std::cerr << "    " <<
		p_n_cloud_c.points[i].x << " " << p_n_cloud_c.points[i].y << " " << p_n_cloud_c.points[i].z << " " <<
		p_n_cloud_c.points[i].normal[0] << " " << p_n_cloud_c.points[i].normal[1] << " " << p_n_cloud_c.points[i].normal[2] << std::endl;

	getchar();
	return (0);
}