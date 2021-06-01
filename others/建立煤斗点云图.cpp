
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
int main(int argc, char** argv)
{
	//��ʼ������,��������ѭ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_4(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sum(new pcl::PointCloud<pcl::PointXYZ>);

	//������
	cloud->height = 10;
	cloud->width = 10000;
	cloud_1->height = 10;
	cloud_1->width = 10000;
	cloud_2->height = 10;
	cloud_2->width = 10000;
	cloud_3->height = 10;
	cloud_3->width = 10000;
	cloud_4->height = 10;
	cloud_4->width = 10000;

	//cloud->is_dense = true;
	cloud->points.resize(cloud->height *cloud->width);//����
	cloud_1->points.resize(cloud_1->height *cloud_1->width);//����
	cloud_2->points.resize(cloud_2->height *cloud_2->width);//����
	cloud_3->points.resize(cloud_3->height *cloud_3->width);//����
	cloud_4->points.resize(cloud_4->height *cloud_4->width);//����

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		//������Ҫ����ú���������棬Ȼ����cloudcompare�кϲ�
		//����������
		cloud->points[i].z = 9 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = (cloud->points[i].z + 1.8)*0.5555;
		cloud->points[i].x = cloud->points[i].y  * rand() / (RAND_MAX + 1.0f);

	}

	for (size_t i = 0; i < cloud_1->points.size(); ++i)
	{

		//����������
		cloud_1->points[i].z = 9 * rand() / (RAND_MAX + 1.0f);
		cloud_1->points[i].x = (cloud_1->points[i].z + 1.8)*0.5555;
		cloud_1->points[i].y = cloud_1->points[i].x  * rand() / (RAND_MAX + 1.0f);

	}
	for (size_t i = 0; i < cloud_2->points.size(); ++i)
	{
		//����ú������
		cloud_2->points[i].x = 1 * rand() / (RAND_MAX + 1.0f);
		cloud_2->points[i].y = 1 * rand() / (RAND_MAX + 1.0f);
		cloud_2->points[i].z = 0;
	}

	for (size_t i = 0; i < cloud_3->points.size(); ++i)
	{
		//����ú�����棬��x=0
		cloud_3->points[i].z = 9 * rand() / (RAND_MAX + 1.0f);
		cloud_3->points[i].x = (cloud_3->points[i].z + 1.8)*0.5555* rand() / (RAND_MAX + 1.0f);
		cloud_3->points[i].y = 0;
	}

	for (size_t i = 0; i < cloud_4->points.size(); ++i)
	{
		//����ú�����棬��y=0
		cloud_4->points[i].z = 9 * rand() / (RAND_MAX + 1.0f);
		cloud_4->points[i].y = (cloud_4->points[i].z + 1.8)*0.5555* rand() / (RAND_MAX + 1.0f);
		cloud_4->points[i].x = 0;
	}

	//���ƺϲ�

	*cloud_sum = *cloud + *cloud_1;
	*cloud_sum = *cloud_sum + *cloud_2;
	*cloud_sum = *cloud_sum + *cloud_3;
	*cloud_sum = *cloud_sum + *cloud_4;

	//pcl::PCDWriter writer;
	//writer.write("F:/���ƴ���/model/model_s2.pcd", *cloud);

	pcl::PLYWriter t, t1, t2, t3, t4;
	t.write("./model/cloud.ply", *cloud);
	t1.write("./model/cloud_1.ply", *cloud_1);
	t2.write("./model/cloud_2.ply", *cloud_2);
	t3.write("./model/cloud_3.ply", *cloud_3);
	t4.write("./model/cloud_4.ply", *cloud_4);

	//���ӻ�
	pcl::visualization::PCLVisualizer viewer("cloud_viewer");

	//viewer.setBackgroundColor(255, 255, 255);
	viewer.addPointCloud(cloud_sum);
	viewer.addCoordinateSystem();
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return(0);


}