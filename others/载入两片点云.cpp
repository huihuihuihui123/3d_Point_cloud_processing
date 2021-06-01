#include <iostream>
#include <pcl/io/pcd_io.h>      //io模块 
#include <pcl/point_types.h>   //数据类型

int
main(int argc, char** argv)
{
	//申明三个pcl::PointXYZ点云数据类型，分别为输入cloud_a, cloud_b, 输出cloud_c
	pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;

	// 创建点云数据
	//设置cloud_a的个数为5
	cloud_a.width = 5;
	cloud_a.height = cloud_b.height = 1; //设置都为无序点云
	cloud_a.points.resize(cloud_a.width * cloud_a.height); //总数
	cloud_b.width = 3;
	cloud_b.points.resize(cloud_b.width * cloud_b.height);
	
	//以下循环生成无序点云填充上面定义的两种类型的点云数据
	for (size_t i = 0; i < cloud_a.points.size(); ++i)
	{  //cloud_a产生三个点（每个点都有X Y Z 三个随机填充的值）
		cloud_a.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	for (size_t i = 0; i < cloud_b.points.size(); ++i)
	{   //如果连接a+b=c，则cloud_b用三个点作为xyz的数据
		cloud_b.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_b.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_b.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	
	/*******************************************************************
    2个输入（cloud_a cloud_b ）
	1个输出（cloud_c ）然后就是为两个输入点云cloud_a和 cloud_b

	********************************************************************/
	//输出Cloud A
	std::cerr << "Cloud A: " << std::endl;
	for (size_t i = 0; i < cloud_a.points.size(); ++i)
		std::cerr << "    " << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << std::endl;
	//输出Cloud B
	std::cerr << "Cloud B: " << std::endl;
	for (size_t i = 0; i < cloud_b.points.size(); ++i)
	{
		std::cerr << "    " << cloud_b.points[i].x << " " << cloud_b.points[i].y << " " << cloud_b.points[i].z << std::endl;
	}
	// Copy the point cloud data

	/***************************************用“+”重载符将两片点云加起来***********************************************************/
	cloud_c = cloud_a;
	cloud_c += cloud_b;//把cloud_a和cloud_b连接一起创建cloud_c  后输出
	std::cerr << "Cloud C: " << std::endl;
	for (size_t i = 0; i < cloud_c.points.size(); ++i)
	std::cerr << "    " << cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << " " << std::endl;

	getchar();
	return (0);
}