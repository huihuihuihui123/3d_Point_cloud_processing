#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	//������
	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	/**********************************�˲���ConditionalRemoval************************************/
	//���������޶����µ��˲���
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new
		pcl::ConditionAnd<pcl::PointXYZ>());   //���������������
	//Ϊ�������������ӱȽ�����
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));   //�����Z�ֶ��ϴ���0�ıȽ�����
	/*
	EQ ���� EQUAL����
	<span style = "font-family: Arial, Helvetica, sans-serif;">GT ���� GREATER THAN����< / span>
	LT ���� LESS THANС��
	GE ���� GREATER THAN OR EQUAL ���ڵ���
	LE ���� LESS THAN OR EQUAL С�ڵ���
	*/
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));   //�����Z�ֶ���С��0.8�ıȽ�����
	  // �����˲�������������������ʼ��
	pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
	condrem.setCondition(range_cond);
	condrem.setInputCloud(cloud);                   //�������
	condrem.setKeepOrganized(true);               //���ñ��ֵ��ƵĽṹ
	// ִ���˲�
	condrem.filter(*cloud_filtered);  //����0.0С��0.8�������������ڽ����˲���
	
	std::cerr << "Cloud before filtering: " << std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << std::endl;
	// display pointcloud after filtering
	std::cerr << "Cloud after filtering: " << std::endl;
	for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
		std::cerr << "    " << cloud_filtered->points[i].x << " "
		<< cloud_filtered->points[i].y << " "
		<< cloud_filtered->points[i].z << std::endl;
	return (0);
}