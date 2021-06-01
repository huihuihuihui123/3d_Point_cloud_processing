#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>//点云可视化

int
main(int argc, char** argv)
{

	/**********************************************************************************************************
	 从输入的.PCD 文件载入数据后，创建一个VOxelGrid滤波器对数据进行下采样，在这里进行下才样是为了加速处理过程，
	 越少的点意味着分割循环中处理起来越快
	 **********************************************************************************************************/

	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);//申明滤波前后的点云
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_p(new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_f(new pcl::PointCloud<pcl::PointXYZRGBA>);

	// 读取PCD文件
	pcl::PCDReader reader;
	reader.read("quicksave.pcd", *cloud_filtered);

	cout << "111" << endl;
	pcl::io::savePCDFileASCII("quicksavetxt.pcd", *cloud_filtered);
	////统计滤波前的点云个数
	//std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

	//// 创建体素栅格下采样: 下采样的大小为1cm
	//pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //体素栅格下采样对象
	//sor.setInputCloud(cloud_blob);             //原始点云
	//sor.setLeafSize(0.5f, 0.5f, 0.5f);    // 设置采样体素大小
	//sor.filter(*cloud_filtered_blob);        //保存

	//// 转换为模板点云
	//pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

	//std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

	//// 保存下采样后的点云
	//pcl::PCDWriter writer;
	///*writer.write<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);*/

	//pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	//pcl::SACSegmentation<pcl::PointXYZ> seg;               //创建分割对象

	//seg.setOptimizeCoefficients(true);                    //设置对估计模型参数进行优化处理

	//seg.setModelType(pcl::SACMODEL_PLANE);                //设置分割模型类别
	//seg.setMethodType(pcl::SAC_RANSAC);                   //设置用哪个随机参数估计方法
	//seg.setMaxIterations(100);                            //设置最大迭代次数
	//seg.setDistanceThreshold(0.5);                      //判断是否为模型内点的距离阀值


	///*************************************ExtractIndices提取点云**********************************************/
	//// 设置ExtractIndices的实际参数
	//pcl::ExtractIndices<pcl::PointXYZ> extract;        //创建点云提取对象

	//int i = 0, nr_points = (int)cloud_filtered->points.size();
	//// While 30% of the original cloud is still there
	//std::vector<std::vector<float>> coefficient4s;
	//while (cloud_filtered->points.size() > 0.3 * nr_points)
	//{
	//	// 为了处理点云包含的多个模型，在一个循环中执行该过程并在每次模型被提取后，保存剩余的点进行迭代
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