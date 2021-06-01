#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>//���ƿ��ӻ�

int
main(int argc, char** argv)
{

	/**********************************************************************************************************
	 �������.PCD �ļ��������ݺ󣬴���һ��VOxelGrid�˲��������ݽ����²���������������²�����Ϊ�˼��ٴ�����̣�
	 Խ�ٵĵ���ζ�ŷָ�ѭ���д�������Խ��
	 **********************************************************************************************************/

	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);//�����˲�ǰ��ĵ���
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_p(new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_f(new pcl::PointCloud<pcl::PointXYZRGBA>);

	// ��ȡPCD�ļ�
	pcl::PCDReader reader;
	reader.read("quicksave.pcd", *cloud_filtered);

	cout << "111" << endl;
	pcl::io::savePCDFileASCII("quicksavetxt.pcd", *cloud_filtered);
	////ͳ���˲�ǰ�ĵ��Ƹ���
	//std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

	//// ��������դ���²���: �²����Ĵ�СΪ1cm
	//pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //����դ���²�������
	//sor.setInputCloud(cloud_blob);             //ԭʼ����
	//sor.setLeafSize(0.5f, 0.5f, 0.5f);    // ���ò������ش�С
	//sor.filter(*cloud_filtered_blob);        //����

	//// ת��Ϊģ�����
	//pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

	//std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

	//// �����²�����ĵ���
	//pcl::PCDWriter writer;
	///*writer.write<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);*/

	//pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	//pcl::SACSegmentation<pcl::PointXYZ> seg;               //�����ָ����

	//seg.setOptimizeCoefficients(true);                    //���öԹ���ģ�Ͳ��������Ż�����

	//seg.setModelType(pcl::SACMODEL_PLANE);                //���÷ָ�ģ�����
	//seg.setMethodType(pcl::SAC_RANSAC);                   //�������ĸ�����������Ʒ���
	//seg.setMaxIterations(100);                            //��������������
	//seg.setDistanceThreshold(0.5);                      //�ж��Ƿ�Ϊģ���ڵ�ľ��뷧ֵ


	///*************************************ExtractIndices��ȡ����**********************************************/
	//// ����ExtractIndices��ʵ�ʲ���
	//pcl::ExtractIndices<pcl::PointXYZ> extract;        //����������ȡ����

	//int i = 0, nr_points = (int)cloud_filtered->points.size();
	//// While 30% of the original cloud is still there
	//std::vector<std::vector<float>> coefficient4s;
	//while (cloud_filtered->points.size() > 0.3 * nr_points)
	//{
	//	// Ϊ�˴�����ư����Ķ��ģ�ͣ���һ��ѭ����ִ�иù��̲���ÿ��ģ�ͱ���ȡ�󣬱���ʣ��ĵ���е���
	//	seg.setInputCloud(cloud_filtered);
	//	seg.segment(*inliers, *coefficients);
	//	if (inliers->indices.size() == 0)
	//	{
	//		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	//		break;
	//	}

	//	// Extract the inliers
	//	extract.setInputCloud(cloud_filtered);
	//	extract.setIndices(inliers);
	//	extract.setNegative(false);
	//	extract.filter(*cloud_p);
	//	std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

	//	std::stringstream ss;
	//	ss << "FinalPCD_plane" << i << ".pcd";
	//	writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);

	//	// Create the filtering object
	//	extract.setNegative(true);
	//	extract.filter(*cloud_f);
	//	cloud_filtered.swap(cloud_f);
	//	std::cerr << "Plan "<<i<<": " << coefficients->values[0]<<","<< coefficients->values[1] <<","<< coefficients->values[2]<<","<< coefficients->values[3] <<std::endl;
	//	i++;
	//	coefficient4s.push_back(coefficients->values);
	//}
	//std::cout << "plane num= " << i << std::endl;
	//std::vector<float> distances;
	//for (int m = 0; m < i/2; m++)
	//{
	//	float distance = abs(coefficient4s[m * 2][3] - coefficient4s[2 * m + 1][3])
	//		/sqrtf(pow((coefficient4s[m * 2][0]+ coefficient4s[m * 2+1][0])/2, 2)+ pow((coefficient4s[m * 2][1] + coefficient4s[m * 2 + 1][1])/2, 2)+ pow((coefficient4s[m * 2][2] + coefficient4s[m * 2 + 1][2])/2, 2));
	//	distances.push_back(distance);
	//}
	//for (std::vector<float>::iterator iter = distances.begin(); iter != distances.end(); iter++)
	//{

	//	std::cout << "plane distance= " << *iter << std::endl;
	//}
	std::cout << "Successfully! " << std::endl;
	getchar();
	return (0);
}