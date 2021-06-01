#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>//统计滤波
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>//体素下采样
#include <pcl/filters/passthrough.h>//条件滤波
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/features/normal_3d.h>//normal相关
#include<pcl/kdtree/kdtree_flann.h>//kd_tree
#include<pcl/kdtree/io.h>//kd_tree
#include <pcl/surface/gp3.h>//贪婪三角化  pcl::GreedyProjectionTriangulation
#include <pcl/segmentation/region_growing.h>// 平面分割 区域增长法
#include <pcl/common/centroid.h>//计算点云重心

#include <pcl/features/boundary.h>//边界提取

////点云边缘提取
//normEST.setRadiusSearch(reforn)。设置为分辨率的10倍时，效果较好，主要是对于法线估计。
//	邻域半径选择太小了，噪声较大，估计的法线就容易出错，而搜索邻域半径设置的太大估计速度就比较慢。
//boundEst.setRadiusSearch(re)，也设置10倍，太小则内部的很多点就都当成边界点了。
//  最后一个参数是边界判断时的角度阈值，默认值为PI / 2，此处设置为PI / 4，用户也可以根据需要进行更改。

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_VoxelGrid(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("./turn_shuibei360 - Cloud.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file sample.pcd \n");
		return (-1);
	}

	std::cout << "points sieze is:" << cloud->size() << std::endl;

	pcl::VoxelGrid<pcl::PointXYZ> sor;  //创建滤波对象
	sor.setInputCloud(cloud);            //设置需要过滤的点云给滤波对象
	sor.setLeafSize(0.1, 0.1, 0.1);  //设置滤波时创建的体素体积为1cm的立方体
	sor.filter(*cloud_VoxelGrid);           //执行滤波处理，存储输出
	std::cerr << "PointCloud after filtering: " << cloud_VoxelGrid->width * cloud_VoxelGrid->height
		<< " data points (" << pcl::getFieldsList(*cloud_VoxelGrid) << ").";


	//计算点云平均间距 后期用点云间距（点云分辨率a），后期用10*a 估计法线
	//创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_VoxelGrid);
	int k = 2;
	float everagedistance = 0;
	size_t npoints = 0; //有效点数
	for (int i = 0; i < cloud->size() / 2; i++)
	{
		//std::cout << "cloud->size()/2" << cloud->points[i] << std::endl;
		std::vector<int> nnh;
		std::vector<float> squaredistance;
		//  pcl::PointXYZ p;
		//   p = cloud->points[i];
		int nres = kdtree.nearestKSearch(cloud->points[i], k, nnh, squaredistance);
		if (nres == 2)
		{
			everagedistance += sqrt(squaredistance[1]);
			++npoints;
		}
		/*std::cout << "查询点位： " << cloud->points[i] << std::endl;
		std::cout << "近邻为： " << nnh[0] << "  " << nnh[1] << std::endl;
		std::cout << "近邻为： " << cloud->points[nnh[0]] << "  " << cloud->points[nnh[1]] << std::endl;
		*/

		//   cout<<everagedistance<<endl;

	}
	if (npoints != 0)
	{
		everagedistance = everagedistance / npoints;
	}
	cout << "average distance is : " << everagedistance << endl;

	/******************************估计法线*******************************************/
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;  //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
	normEst.setInputCloud(cloud_VoxelGrid);
	normEst.setSearchMethod(tree);
	//normEst.setRadiusSearch(10*everagedistance);  //法向估计的半径
	normEst.setKSearch(9);  //法向估计的点数
	normEst.compute(*normals);
	cout << "normal size is " << normals->size() << endl;
	//normal_est.setViewPoint(0,0,0); //这个应该会使法向一致

	/**********************************估计边界****************************************************/
	pcl::PointCloud<pcl::Boundary> boundaries;  //保存边界估计结果
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;//定义一个进行边界特征估计的对象
	est.setInputCloud(cloud_VoxelGrid);
	est.setInputNormals(normals);/*M_PI_2 */
	est.setAngleThreshold(M_PI_2);   //在这里 由于构造函数已经对其进行了初始化 为Π/2 ，必须这样 使用 M_PI/2  M_PI_2 //边界估计时的角度阈值
	est.setSearchMethod(tree);
	est.setKSearch(20);  //一般这里的数值越高，最终边界识别的精度越好 20
	//est.setRadiusSearch(10 * everagedistance);  //一般设为10倍的分辨率
									//  est.setRadiusSearch(everagedistance);  //搜索半径
	est.compute(boundaries);

	//  pcl::PointCloud<pcl::PointXYZ> boundPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> noBoundPoints;
	int countBoundaries = 0;
	for (int i = 0; i < cloud_VoxelGrid->size(); i++) {
		uint8_t x = (boundaries.points[i].boundary_point);
		int a = static_cast<int>(x); //该函数的功能是强制类型转换
		if (a == 1)
		{
			//  boundPoints.push_back(cloud->points[i]);
			(*boundPoints).push_back(cloud->points[i]);
			countBoundaries++;
		}
		else
			noBoundPoints.push_back(cloud->points[i]);

	}
	std::cout << "boudary size is：" << countBoundaries << std::endl;
	//  pcl::io::savePCDFileASCII("boudary.pcd",boundPoints);

	pcl::io::savePCDFile("boudary.pcd", *boundPoints);
	pcl::io::savePCDFile("NoBoundpoints.pcd", noBoundPoints);


	pcl::visualization::PCLVisualizer viewer("boundPoints viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 0, 255, 255);
	viewer.addPointCloud(boundPoints, rgb, "sample cloud");
	//viewer.addCoordinateSystem(1.0, centroid[0], centroid[1], centroid[2]);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	/*pcl::visualization::CloudViewer viewer("test");
	viewer.showCloud(boundPoints);
	while (!viewer.wasStopped())
	{
	}*/
	return 0;
}

/************************************计算点云分辨率(平均点间距)**********************************************/
//原文链接：https ://blog.csdn.net/qq_39707351/article/details/93470524

	//计算点云平均间距 后期用点云间距（点云分辨率a），后期用10*a 估计法线
	//创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
double computerCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size() / 2; ++i)
	{
		if (!std::isfinite((*cloud)[i].x) || !std::isfinite((*cloud)[i].y) || !std::isfinite((*cloud)[i].z))
		{
			continue;
		}
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	cout << "average distance is : " << res << endl;
	return res;
}