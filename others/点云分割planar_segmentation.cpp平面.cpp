#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>   //����������Ʒ���ͷ�ļ�
#include <pcl/sample_consensus/model_types.h>   //ģ�Ͷ���ͷ�ļ�
#include <pcl/segmentation/sac_segmentation.h>   //���ڲ���һ���Էָ�����ͷ�ļ�

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// ������
	cloud->width = 15;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	// �������ݣ���������������Ƶ�x,y���꣬������zΪ1��ƽ����
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1.0;
	}

	// ���ü�������㣬���������ü������zֵ��ʹ��ƫ��zΪ1��ƽ��
	cloud->points[0].z = 2.0;
	cloud->points[3].z = -2.0;
	cloud->points[6].z = 4.0;

	std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl;  //��ӡ
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << std::endl;
	//�����ָ�ʱ����Ҫ��ģ��ϵ������coefficients���洢�ڵ�ĵ��������϶���inliers
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// �����ָ����
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// ��ѡ�����ã�����ģ��ϵ����Ҫ�Ż�
	seg.setOptimizeCoefficients(true);
	// ��Ҫ�����ã����÷ָ��ģ�����ͣ����õ�����������Ʒ��������뷧ֵ���������
	seg.setModelType(pcl::SACMODEL_PLANE);   //����ģ������
	seg.setMethodType(pcl::SAC_RANSAC);      //�����������һ���Է�������
	seg.setDistanceThreshold(0.01);    //�趨���뷧ֵ�����뷧ֵ�����˵㱻��Ϊ�Ǿ��ڵ��Ǳ������������
										 //��ʾ�㵽����ģ�͵ľ������ֵ��

	seg.setInputCloud(cloud);
	//�����ָ�ʵ�֣��洢�ָ������㼸��inliers���洢ƽ��ģ�͵�ϵ��coefficients
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return (-1);
	}
	//��ӡ��ƽ��ģ��
	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;

	std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
	for (size_t i = 0; i < inliers->indices.size(); ++i)
		std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
		<< cloud->points[inliers->indices[i]].y << " "
		<< cloud->points[inliers->indices[i]].z << std::endl;

	getchar();
	return (0);
}