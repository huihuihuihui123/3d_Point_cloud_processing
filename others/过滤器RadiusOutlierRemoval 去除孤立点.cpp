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
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f); //��Χ�� 0--1024
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	
	/*******************************������RadiusOutlierRemovalȥ��������******************************************/
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;  //�����˲���
	outrem.setInputCloud(cloud);    //�����������
	outrem.setRadiusSearch(800);     //���ð뾶Ϊ0.8�ķ�Χ�����ٽ���
	outrem.setMinNeighborsInRadius(2); //���ò�ѯ�������㼯��С��2��ɾ��
		// apply filter
	outrem.filter(*cloud_filtered);     //ִ�������˲�   �ڰ뾶Ϊ0.8 �ڴ˰뾶�ڱ���Ҫ�������ھӵ㣬�˵�Żᱣ��

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

	getchar();
	return (0);
}