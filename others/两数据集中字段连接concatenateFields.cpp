#include <iostream>
#include <pcl/io/pcd_io.h>      //io模块 
#include <pcl/point_types.h>   //数据类型

int
main(int argc, char** argv)
{
	//申明三个pcl::PointXYZ点云数据类型，分别为cloud_a, cloud_b, cloud_c
	pcl::PointCloud<pcl::PointXYZ> cloud_a;
	//存储进行连接时需要的Normal点云,Normal (float n_x, float n_y, float n_z)
	pcl::PointCloud<pcl::Normal> n_cloud_b; // 向量n_x，n_y，n_z
	//存储连接XYZ与normal后的点云
	pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;

	// 创建点云数据
	//设置cloud_a的个数为5
	cloud_a.width = 5;
	cloud_a.height  = n_cloud_b.height = 1; //设置都为无序点云
	cloud_a.points.resize(cloud_a.width * cloud_a.height); //总数
	
	n_cloud_b.width = 5; //如果是连接XYZ与normal则生成5个法线（字段间连接）
	n_cloud_b.points.resize(n_cloud_b.width * n_cloud_b.height);
	
	//以下循环生成无序点云填充上面定义的两种类型的点云数据
	for (size_t i = 0; i < cloud_a.points.size(); ++i)
	{  //cloud_a产生三个点（每个点都有X Y Z 三个随机填充的值）
		cloud_a.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	for (size_t i = 0; i < n_cloud_b.points.size(); ++i)
	{  //如果连接xyz+normal=xyznormal则n_cloud_b用5个点作为normal数据
		n_cloud_b.points[i].normal[0] = 1024 * rand() / (RAND_MAX + 1.0f);
		n_cloud_b.points[i].normal[1] = 1024 * rand() / (RAND_MAX + 1.0f);
		n_cloud_b.points[i].normal[2] = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	/*******************************************************************
	定义了连接点云会用到的2个点云对象：2个输入（cloud_a 和n_cloud_b）
	1个输出（n_cloud_c）然后就是为两个输入点云cloud_a 和n_cloud_b填充数据

	********************************************************************/
	//输出Cloud A
	std::cerr << "Cloud A: " << std::endl;
	for (size_t i = 0; i < cloud_a.points.size(); ++i)
		std::cerr << "    " << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << std::endl;
	//输出Cloud B
	std::cerr << "Cloud B: " << std::endl;
		for (size_t i = 0; i < n_cloud_b.points.size(); ++i)
			std::cerr << "    " << n_cloud_b.points[i].normal[0] << " " << n_cloud_b.points[i].normal[1] << " " << n_cloud_b.points[i].normal[2] << std::endl;

	// Copy the point cloud data
	
	//连接字段  把cloud_a和 n_cloud_b字段连接 一起创建 p_n_cloud_c)
	pcl::concatenateFields(cloud_a, n_cloud_b, p_n_cloud_c);
	std::cerr << "Cloud C: " << std::endl;
	for (size_t i = 0; i < p_n_cloud_c.points.size(); ++i)
		std::cerr << "    " <<
		p_n_cloud_c.points[i].x << " " << p_n_cloud_c.points[i].y << " " << p_n_cloud_c.points[i].z << " " <<
		p_n_cloud_c.points[i].normal[0] << " " << p_n_cloud_c.points[i].normal[1] << " " << p_n_cloud_c.points[i].normal[2] << std::endl;

	getchar();
	return (0);
}