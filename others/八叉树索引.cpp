#include <pcl/point_cloud.h>   //点云头文件
#include <pcl/octree/octree.h>  //八叉树头文件

#include <iostream>
#include <vector>
#include <ctime>

//Octree类关键点的说明
//PCL octree组件提供了几个octree类型，它们各自的叶节点特征基本上是不同的，
//
//OctreePointCloudVector(等于OctreePointCloud)：该octree能够保存每一个节点上的点索引列。
//
//OctreePointCloudSinglePoint : 该octree类仅仅保存每一个节点上的单个点的索引，仅仅保存最后分配给叶节点的点索引
//
//	OctreePointCloudOccupancy : 该octree类不存储它的叶节点上的任何信息，它能用于空间填充情况检查
//
//	OctreePointCloudDensity : 存储每一个叶节点体素中点的数目，它可以进行空间点集密集程度的查询

int
main(int argc, char** argv)
{
	srand((unsigned int)time(NULL));   //用系统时间初始化随机种子与 srand (time (NULL))的区别

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 创建点云数据
	cloud->width = 1000;
	cloud->height = 1;               //无序
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)   //随机循环产生点云的坐标值
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}
	/****************************************************************************
	创建一个octree实例，用设置分辨率进行初始化，该octree用它的页节点存放点索引向量，
	分辨率参数描述最低一级octree的最小体素的尺寸，因此octree的深度是分辨率和点云空间维度
	的函数，如果知道点云的边界框，应该用defineBoundingbox方法把它分配给octree然后通过
	点云指针把所有点增加到ctree中。
	*****************************************************************************/
	float resolution = 128.0f;//体素网格大小

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);   //初始化Octree

	octree.setInputCloud(cloud);         //设置输入点云    这两句是最关键的建立PointCloud和octree之间的联系
	octree.addPointsFromInputCloud();   //构建octree

	pcl::PointXYZ searchPoint;   //设置searchPoint

	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	/*************************************************************************************
	一旦PointCloud和octree联系一起，就能进行搜索操作，这里使用的是“体素近邻搜索”，把查询点所在体素中
	 其他点的索引作为查询结果返回，结果以点索引向量的形式保存，因此搜索点和搜索结果之间的距离取决于octree的分辨率参数
  *****************************************************************************************/

	std::vector<int> pointIdxVec;            //存储体素近邻搜索结果向量

	if (octree.voxelSearch(searchPoint, pointIdxVec))    //执行搜索，返回结果到pointIdxVec
	{
		std::cout << "Neighbors within voxel search at (" << searchPoint.x
			<< " " << searchPoint.y
			<< " " << searchPoint.z << ")"
			<< std::endl;

		for (size_t i = 0; i < pointIdxVec.size(); ++i)                  //打印结果点坐标
			std::cout << "    " << cloud->points[pointIdxVec[i]].x
			<< " " << cloud->points[pointIdxVec[i]].y
			<< " " << cloud->points[pointIdxVec[i]].z << std::endl;
	}

	/**********************************************************************************
	 K 被设置为10 ，K近邻搜索  方法把搜索结果写到两个分开的向量，第一个pointIdxNKNSearch包含搜索结果
	  （结果点的索引的向量）  第二个向量pointNKNSquaredDistance存储搜索点与近邻之间的距离的平方。
   *************************************************************************************/

   //K 近邻搜索
	int K = 10;

	std::vector<int> pointIdxNKNSearch;   //结果点的索引的向量
	std::vector<float> pointNKNSquaredDistance;            //搜索点与近邻之间的距离的平方

	std::cout << "K nearest neighbor search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with K=" << K << std::endl;

	if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
			<< " " << cloud->points[pointIdxNKNSearch[i]].y
			<< " " << cloud->points[pointIdxNKNSearch[i]].z
			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}

	// 半径内近邻搜索

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

	std::cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << std::endl;


	if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}
	getchar();

}