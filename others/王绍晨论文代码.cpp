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
#include"cseg.h"

#include <ctime>//程序运行时间测试
//第三章 体素下采样
int third1()
{
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_VoxelGrid(new pcl::PCLPointCloud2());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3_VoxelGrid(new pcl::PointCloud<pcl::PointXYZ>);
	//点云对象的读取
	pcl::PCDReader reader;
	reader.read("C:\\Users\\Administrator\\Desktop\\轮廓3\\1plyCloud.pcd", *cloud);
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";
	pcl::fromPCLPointCloud2(*cloud, *cloud1);		/******************************************************************************
	   创建一个pcl::VoxelGrid滤波器，
   **********************************************************************************/
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //创建滤波对象
	sor.setInputCloud(cloud);            //设置需要过滤的点云给滤波对象
	sor.setLeafSize(0.07, 0.07, 0.07);  //设置滤波时创建的体素体积为1cm的立方体
	sor.filter(*cloud_VoxelGrid);           //执行滤波处理，存储输出
	std::cerr << "PointCloud after filtering: " << cloud_VoxelGrid->width * cloud_VoxelGrid->height
		<< " data points (" << pcl::getFieldsList(*cloud_VoxelGrid) << ").";
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud1);
	pcl::PointXYZ searchPoint;
	pcl::fromPCLPointCloud2(*cloud_VoxelGrid, *cloud3_VoxelGrid);
	/*searchPoint.x = cloud1->points[0].x;
	searchPoint.y = cloud1->points[0].y;
	searchPoint.z= cloud1->points[0].z;*/
	// 这个是近邻个数
	int K = 1;

	// 创建两个向量，分别存放近邻的索引值、近邻的中心距
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	cloud2->width = cloud_VoxelGrid->width;
	cloud2->height = cloud_VoxelGrid->height;
	cloud2->points.resize(cloud2->width * cloud2->height);

	//寻找距离重心最近的点

	for (int j = 0; j < cloud_VoxelGrid->width*cloud_VoxelGrid->height; j++)
	{
		searchPoint.x = cloud3_VoxelGrid->points[j].x;
		searchPoint.y = cloud3_VoxelGrid->points[j].y;
		searchPoint.z = cloud3_VoxelGrid->points[j].z;
		//这是执行 K 近邻查找的成员函数，下面一行判断查找成功则执行输出结果
		if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			{
				cloud2->points[j].x = cloud1->points[pointIdxNKNSearch[i]].x;
				cloud2->points[j].y = cloud1->points[pointIdxNKNSearch[i]].y;
				cloud2->points[j].z = cloud1->points[pointIdxNKNSearch[i]].z;
			}
		}
	}
	cout << cloud2->points.size();
	system("pause");
	//保存数据
	pcl::io::savePCDFileASCII("C:\\Users\\Administrator\\Desktop\\轮廓3\\11table_scene_lms400_downsampled7.pcd", *cloud2);
	return (0);

	return 0;
}

//点云去噪

int third2()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliner(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliner(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read<pcl::PointXYZ>("C:\\Users\\Administrator\\Desktop\\ply\\1Cloud.pcd", *cloud_filtered);

	//下采样之后再去噪可以加快处理速度

	 //pcl::VoxelGrid<pcl::PointXYZ> sor;         //创建滤波对象
	 //sor.setInputCloud(cloud);       //设置需要过滤的点云给滤波对象
	 //sor.setLeafSize(0.07, 0.07, 0.07);         //设置滤波时创建的体素体积为7cm的立方体
	 //sor.filter(*cloud_filtered);               //执行滤波处理，存储输出
	pcl::io::savePCDFileASCII("C:\\Users\\Administrator\\Desktop\\test_pcdchanpin.pcd", *cloud_filtered);
	//std::cerr << "Cloud before filtering: " << std::endl;
	//std::cerr << *cloud_filtered << std::endl;
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;
	sor1.setInputCloud(cloud_filtered);
	sor1.setMeanK(100);//100个临近点
	sor1.setStddevMulThresh(1.5);//距离大于1.5倍标准方差
	sor1.filter(*cloud_inliner);
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_inliner << std::endl;
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("table_scene_lms400_inliers1.pcd", *cloud_inliner, false);
	/*sor1.setNegative(true);
	sor1.filter(*cloud_outliner);*/
	//writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd", *cloud_outliner, false);*/
	pcl::visualization::PCLVisualizer viewer("demo");
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);
	// The color we will be using
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_green(cloud_inliner, 20, 180, 20);
	viewer.addPointCloud(cloud_inliner, cloud_out_green, "cloud_out", v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setSize(1280, 1024);  // Visualiser window size
								 //viewer.showCloud(cloud_out);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return (0);
}


//论文第四章代码说明
//点云平面分割拟合代码
int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredy(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredxy(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster1(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster3(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster4(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster5(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::ModelCoefficients::Ptr coefficients[5];
	cseg *csegc = new cseg[5];


	time_t begin, endread;
	begin = clock();  //开始计时
	// Load PCD file
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("./turn_pointcloud360 - Cloud.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file sample.pcd \n");
		return (-1);
	}
	endread = clock();
	double readTimes = double(endread - begin) / CLOCKS_PER_SEC; //将clock()函数的结果转化为以秒为单位的量

	std::cout << "load pcd time: " << readTimes << "s" << std::endl;

	// 设置条件滤波器对象
	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setInputCloud(cloud);            //设置输入点云
	//pass.setFilterFieldName("y");         //设置过滤时所需要点云类型的y字段
	//pass.setFilterLimits(20.0, 24.5);
	//pass.filter(*cloud_filteredy);//设置在过滤字段的范围
	//pcl::PassThrough<pcl::PointXYZ> passx;
	//passx.setInputCloud(cloud_filteredy);
	//passx.setFilterFieldName("x");               //设置过滤时所需要点云类型的x字段
	//passx.setFilterLimits(0.8, 4.3);  			//pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
	//passx.filter(*cloud_filteredxy);            //执行滤波，保存过滤结果在cloud_filtered
	///*pcl::io::savePCDFileASCII("C:\\Users\\Administrator\\Desktop\\test_pcd2.pcd", *cloud_filteredxy);*/

	//设置下采样
	pcl::VoxelGrid<pcl::PointXYZ> sor;         //创建滤波对象
	sor.setInputCloud(cloud);       //设置需要过滤的点云给滤波对象
	sor.setLeafSize(0.5, 0.5, 0.5);         //设置滤波时创建的体素体积为7cm的立方体
	sor.filter(*cloud_filtered);               //执行滤波处理，存储输出

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";
	//pcl::io::savePCDFileASCII("C:\\Users\\Administrator\\Desktop\\test_pcd1111.pcd", *cloud_filtered);


	//发现计算
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud_filtered);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	////贪婪三角化 曲面重建
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	//pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	//tree2->setInputCloud(cloud_with_normals);
	//pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//pcl::PolygonMesh triangles;  //创建多边形网格，用于存储结果
	//gp3.setSearchRadius(5);//搜索半径的设定（这个参数必须由用户指定），它决定的重建后三角形的大小。
	//gp3.setMu(2.5);//mu是个加权因子，对于每个参考点，其映射所选球的半径由mu与离参考点最近点的距离乘积所决定，这样就很好解决了点云密度不均匀的问题，mu一般取值为2.5-3。
	//gp3.setMaximumNearestNeighbors(100);//)临近点阈值设定。一般为80-100。

	////两点的法向量角度差大于此值，这两点将不会连接成三角形，这个就恰恰和点云局部平滑的约束呼应，
	////如果一个点是尖锐点那么它将不会和周围的点组成三角形，其实这个也有效的滤掉了一些离群点。这个值一般为45度。
	//gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees 最大平面角
	//gp3.setMinimumAngle(M_PI / 18); // 10 degrees 三角形角度约束
	//gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	//gp3.setNormalConsistency(false);//输入的法向量是否连续变化的。这个一般来讲是false，除非输入的点云是全局光滑的（比如说一个球）。

	//// 设置搜索方法和输入点云
	//gp3.setInputCloud(cloud_with_normals);
	//gp3.setSearchMethod(tree2);

	////执行重构，结果保存在triangles中
	//gp3.reconstruct(triangles);

	////保存网格图
	//pcl::io::saveOBJFile("./mesh1.obj", triangles);

	time_t endreconstruction = 0;
	endreconstruction = clock();  //开始计时
	double reconstructTimes = double(endreconstruction - endread) / CLOCKS_PER_SEC; //将clock()函数的结果转化为以秒为单位的量

	std::cout << "reconstructTimes pcd time: " << reconstructTimes << "s" << std::endl;

	//区域生长算法 分割点云

	/*工作原理：首先需要明白，区域增长是从有最小曲率值(curvature value)的点开始的。因此，我们必须计算出所有曲率值，并对它们进行排序。这是因为曲率最小的点位于平坦区域，而从最平坦的区域增长可以减少区域的总数。现在我们来具体描述这个过程：

		1.点云中有未标记点，按照点的曲率值对点进行排序，找到最小曲率值点，并把它添加到种子点集；

		2.对于每个种子点，算法都会发现周边的所有近邻点。1）计算每个近邻点与当前种子点的法线角度差(reg.setSmoothnessThreshold)，如果差值小于设置的阈值，则该近邻点被重点考虑，进行第二步测试；2）该近邻点通过了法线角度差检验，如果它的曲率小于我们设定的阈值(reg.setCurvatureThreshold)，这个点就被添加到种子点集，即属于当前平面。

		3.通过两次检验的点，被从原始点云去除。

		4.设置最小点簇的点数min（reg.setMinClusterSize），最大点簇为max（reg.setMaxClusterSize）。

		4.重复1 - 3步，算法会生成点数在min和max的所有平面，并对不同平面标记不同颜色加以区分。

		5.直到算法在剩余点中生成的点簇不能满足min，算法停止工作。*/
		
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(1000);//设置平面包含的最少点数（这个参数非常重要，小于这个参数的平面会被忽略不计）
	reg.setMaxClusterSize(1000000);//设置最大的点数，原理同上，但是一般我们希望的是无穷大，所以可以设大一点，
	reg.setSearchMethod(tree);//设置搜索方法，之前我们声明了使用kd树的方法，所以直接用就可以了，这也是默认的方法。
	reg.setNumberOfNeighbours(30);//设置参考的邻域点数，也就是看看周边的多少个点来决定这是一个平面
	//（这个参数至关重要，决定了你的容错率，如果设置的很大，那么从全局角度看某一个点稍微有点歪也可以接受，如果设置的很小则通常检测到的平面都会很小）
	reg.setInputCloud(cloud_filtered);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(0.5 / 180.0 * M_PI);//设置判断的阈值，就是两个法线在多大的夹角内还可以当做是共面的，
	//计算每个近邻点与当前种子点的法线角度差，如果差值小于设置的阈值，则该近邻点被重点考虑

	reg.setCurvatureThreshold(1);


	//可以把结果输出到一个簇里面，这个簇会自动把每个平面分成一个vector，可以打印下来看看
	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;
	std::cout << std::endl;

	time_t endmesh = 0;
	endmesh = clock();  //开始计时
	double meshTimes = double(endmesh - endreconstruction) / CLOCKS_PER_SEC; //将clock()函数的结果转化为以秒为单位的量

	std::cout << "endmesh pcd time: " << meshTimes << "s" << std::endl;


	// 创建存储点云重心的对象
	Eigen::Vector4f centroid;

	pcl::compute3DCentroid(*cloud_filtered, centroid);

	std::cout << "The XYZ coordinates of the centroid are: ("
		<< centroid[0] << ", "
		<< centroid[1] << ", "
		<< centroid[2] << ")." << std::endl;

	

	//左面
	for (std::vector<int>::const_iterator pit = clusters[0].indices.begin(); pit != clusters[0].indices.end(); ++pit)
	{
		cloud_cluster1->points.push_back(cloud_filtered->points[*pit]);
	}


	//求平面参数
	coefficients[1] = csegc[0].comcof(cloud_cluster1);
	//前面
	for (std::vector<int>::const_iterator pit = clusters[2].indices.begin(); pit != clusters[2].indices.end(); ++pit)
	{
		cloud_cluster2->points.push_back(cloud_filtered->points[*pit]);
	}
	coefficients[2] = csegc[1].comcof(cloud_cluster2);
	//上面
	for (std::vector<int>::const_iterator pit = clusters[3].indices.begin(); pit != clusters[3].indices.end(); ++pit)
	{
		cloud_cluster3->points.push_back(cloud_filtered->points[*pit]);
	}
	coefficients[0] = csegc[2].comcof(cloud_cluster3);
	//右面
	for (std::vector<int>::const_iterator pit = clusters[1].indices.begin(); pit != clusters[1].indices.end(); ++pit)
	{
		cloud_cluster4->points.push_back(cloud_filtered->points[*pit]);
	}
	coefficients[3] = csegc[3].comcof(cloud_cluster4);
	//后面
	for (std::vector<int>::const_iterator pit = clusters[5].indices.begin(); pit != clusters[5].indices.end(); ++pit)
	{
		cloud_cluster5->points.push_back(cloud_filtered->points[*pit]);
	}
	coefficients[4] = csegc[4].comcof(cloud_cluster5);
	for (int i = 0; i < 5; i++)
	{
		cout << "面参数:" << coefficients[i]->values[0] << " " << coefficients[i]->values[1] << " " << coefficients[i]->values[2] << " " << coefficients[i]->values[3] << endl;
	}
	double A[5], B[5], C[5], D[5];
	for (int k = 0; k < 5; k++)
	{
		A[k] = coefficients[k]->values[0];
		B[k] = coefficients[k]->values[1];
		C[k] = coefficients[k]->values[2];
		D[k] = -(coefficients[k]->values[3]);
		cout << "A" << k << ":" << A[k] << " " << "B" << k << ":" << B[k] << " " << "C" << k << ":" << C[k] << " " << "D" << k << ":" << D[k] << endl;
	}
	//三个平面方程联立求交点
	double x[4], y[4], z[4];
	x[0] = (B[0] * C[1] * D[2] - 1 * B[0] * C[2] * D[1] - 1 * B[1] * C[0] * D[2] + B[2] * C[0] * D[1] + B[1] * C[2] * D[0] - B[2] * C[1] * D[0]) /
		(A[0] * B[1] * C[2] - A[0] * B[2] * C[1] - A[1] * B[0] * C[2] + A[1] * B[2] * C[0] + A[2] * B[0] * C[1] - A[2] * B[1] * C[0]);
	y[0] = -(A[0] * C[1] * D[2] - A[0] * C[2] * D[1] - A[1] * C[0] * D[2] + A[2] * C[0] * D[1] + A[1] * C[2] * D[0] - A[2] * C[1] * D[0]) /
		(A[0] * B[1] * C[2] - A[0] * B[2] * C[1] - A[1] * B[0] * C[2] + A[1] * B[2] * C[0] + A[2] * B[0] * C[1] - A[2] * B[1] * C[0]);
	z[0] = (A[0] * B[1] * D[2] - A[0] * B[2] * D[1] - A[1] * B[0] * D[2] + A[2] * B[0] * D[1] + A[1] * B[2] * D[0] - A[2] * B[1] * D[0]) /
		(A[0] * B[1] * C[2] - A[0] * B[2] * C[1] - A[1] * B[0] * C[2] + A[1] * B[2] * C[0] + A[2] * B[0] * C[1] - A[2] * B[1] * C[0]);


	x[1] = (B[0] * C[2] * D[3] - 1 * B[0] * C[3] * D[2] - 1 * B[2] * C[0] * D[3] + B[3] * C[0] * D[2] + B[2] * C[3] * D[0] - B[3] * C[2] * D[0]) /
		(A[0] * B[2] * C[3] - A[0] * B[3] * C[2] - A[2] * B[0] * C[3] + A[2] * B[3] * C[0] + A[3] * B[0] * C[2] - A[3] * B[2] * C[0]);
	y[1] = -(A[0] * C[2] * D[3] - A[0] * C[3] * D[2] - A[2] * C[0] * D[3] + A[3] * C[0] * D[2] + A[2] * C[3] * D[0] - A[3] * C[2] * D[0]) /
		(A[0] * B[2] * C[3] - A[0] * B[3] * C[2] - A[2] * B[0] * C[3] + A[2] * B[3] * C[0] + A[3] * B[0] * C[2] - A[3] * B[2] * C[0]);
	z[1] = (A[0] * B[2] * D[3] - A[0] * B[3] * D[2] - A[2] * B[0] * D[3] + A[3] * B[0] * D[2] + A[2] * B[3] * D[0] - A[3] * B[2] * D[0]) /
		(A[0] * B[2] * C[3] - A[0] * B[3] * C[2] - A[2] * B[0] * C[3] + A[2] * B[3] * C[0] + A[3] * B[0] * C[2] - A[3] * B[2] * C[0]);


	x[2] = (B[0] * C[3] * D[4] - 1 * B[0] * C[4] * D[3] - 1 * B[3] * C[0] * D[4] + B[4] * C[0] * D[3] + B[3] * C[4] * D[0] - B[4] * C[3] * D[0]) /
		(A[0] * B[3] * C[4] - A[0] * B[4] * C[3] - A[3] * B[0] * C[4] + A[3] * B[4] * C[0] + A[4] * B[0] * C[3] - A[4] * B[3] * C[0]);
	y[2] = -(A[0] * C[3] * D[4] - A[0] * C[4] * D[3] - A[3] * C[0] * D[4] + A[4] * C[0] * D[3] + A[3] * C[4] * D[0] - A[4] * C[3] * D[0]) /
		(A[0] * B[3] * C[4] - A[0] * B[4] * C[3] - A[3] * B[0] * C[4] + A[3] * B[4] * C[0] + A[4] * B[0] * C[3] - A[4] * B[3] * C[0]);
	z[2] = (A[0] * B[3] * D[4] - A[0] * B[4] * D[3] - A[3] * B[0] * D[4] + A[4] * B[0] * D[3] + A[3] * B[4] * D[0] - A[4] * B[3] * D[0]) /
		(A[0] * B[3] * C[4] - A[0] * B[4] * C[3] - A[3] * B[0] * C[4] + A[3] * B[4] * C[0] + A[4] * B[0] * C[3] - A[4] * B[3] * C[0]);


	x[3] = (B[0] * C[1] * D[4] - 1 * B[0] * C[4] * D[1] - 1 * B[1] * C[0] * D[4] + B[4] * C[0] * D[1] + B[1] * C[4] * D[0] - B[4] * C[1] * D[0]) /
		(A[0] * B[1] * C[4] - A[0] * B[4] * C[1] - A[1] * B[0] * C[4] + A[1] * B[4] * C[0] + A[4] * B[0] * C[1] - A[4] * B[1] * C[0]);
	y[3] = -(A[0] * C[1] * D[4] - A[0] * C[4] * D[1] - A[1] * C[0] * D[4] + A[4] * C[0] * D[1] + A[1] * C[4] * D[0] - A[4] * C[1] * D[0]) /
		(A[0] * B[1] * C[4] - A[0] * B[4] * C[1] - A[1] * B[0] * C[4] + A[1] * B[4] * C[0] + A[4] * B[0] * C[1] - A[4] * B[1] * C[0]);
	z[3] = (A[0] * B[1] * D[4] - A[0] * B[4] * D[1] - A[1] * B[0] * D[4] + A[4] * B[0] * D[1] + A[1] * B[4] * D[0] - A[4] * B[1] * D[0]) /
		(A[0] * B[1] * C[4] - A[0] * B[4] * C[1] - A[1] * B[0] * C[4] + A[1] * B[4] * C[0] + A[4] * B[0] * C[1] - A[4] * B[1] * C[0]);

	ofstream out("C:\\Users\\Administrator\\Desktop\\四对点", ios::app);
	for (int j = 0; j < 4; j++)
	{

		cout << x[j] << "  " << "  " << y[j] << "  " << z[j] << endl;
		out << x[j] << " " << y[j] << " " << z[j] << " " << std::endl;

	}
	out.close();

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::PCLVisualizer viewer("Cluster viewer");
	viewer.addPointCloud(colored_cloud, "color");
	//viewer.addCoordinateSystem(1.0, centroid[0], centroid[1], centroid[2]);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	//pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	//pcl::visualization::CloudViewer viewer("Cluster viewer");
	//viewer.showCloud(colored_cloud);
	//while (!viewer.wasStopped())
	//{

	//}

	return (0);
}

////第四章 旋转平移矩阵求取
//int getRT()
//{
//	//旋转平移矩阵求取
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2(new pcl::PointCloud<pcl::PointXYZ>());
//
//	cloud_in1->width = 3;
//	cloud_in1->height = 1;
//	cloud_in1->is_dense = false;
//	cloud_in1->resize(cloud_in1->width * cloud_in1->height);
//
//	cloud_out1->width = 3;
//	cloud_out1->height = 1;
//	cloud_out1->is_dense = false;
//	cloud_out1->resize(cloud_out1->width * cloud_out1->height);
//
//	//从文件中读取点对
//	FILE * fRead;
//	fRead = fopen("C:\\Users\\Administrator\\Desktop\\圆坐标\\3对圆心坐标.txt", "r");
//
//	cout << "cloud_out1 : " << endl;
//	for (int i = 0; i < cloud_out1->points.size(); i++)
//	{
//		double pt[3];
//		fscanf(fRead, "%lf %lf %lf", pt, pt + 1, pt + 2);
//		cloud_out1->points[i].x = pt[0];
//		cloud_out1->points[i].y = pt[1];
//		cloud_out1->points[i].z = pt[2];
//
//		cout << cloud_out1->points[i].x << "  \t" << cloud_out1->points[i].y << "  \t" << cloud_out1->points[i].z << endl;
//	}
//
//	cout << "cloud_in1 : " << endl;
//	for (int i = 0; i < cloud_in1->points.size(); i++)
//	{
//		double pt[3];
//		fscanf(fRead, "%lf %lf %lf", pt, pt + 1, pt + 2);
//		cloud_in1->points[i].x = pt[0];
//		cloud_in1->points[i].y = pt[1];
//		cloud_in1->points[i].z = pt[2];
//
//		cout << cloud_in1->points[i].x << "  \t" << cloud_in1->points[i].y << "  \t" << cloud_in1->points[i].z << endl;
//	}
//	fclose(fRead);
//	//利用SVD方法求解变换矩阵
//	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
//	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation2;
//	TESVD.estimateRigidTransformation(*cloud_in1, *cloud_out1, transformation2);
//	//输出变换矩阵信息
//	std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << std::endl;
//	printf("\n");
//	printf("    | %6.3f %6.3f %6.3f | \n", transformation2(0, 0), transformation2(0, 1), transformation2(0, 2));
//	printf("R = | %6.3f %6.3f %6.3f | \n", transformation2(1, 0), transformation2(1, 1), transformation2(1, 2));
//	printf("    | %6.3f %6.3f %6.3f | \n", transformation2(2, 0), transformation2(2, 1), transformation2(2, 2));
//	printf("\n");
//	printf("t = < %0.3f, %0.3f, %0.3f >\n", transformation2(0, 3), transformation2(1, 3), transformation2(2, 3));
//	return 0;
//}


/********************************第五章 ****************************************/

////点云边缘提取
//normEST.setRadiusSearch(reforn)。设置为分辨率的10倍时，效果较好，主要是对于法线估计。
//	邻域半径选择太小了，噪声较大，估计的法线就容易出错，而搜索邻域半径设置的太大估计速度就比较慢。
//boundEst.setRadiusSearch(re)，也设置10倍，太小则内部的很多点就都当成边界点了。
//  最后一个参数是边界判断时的角度阈值，默认值为PI / 2，此处设置为PI / 4，用户也可以根据需要进行更改。

int main()
{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("F:\\三维点云处理程序\\三维点云处理程序\\三维点云处理程序\\pcl程序\\pcl程序\\剔除离群点\\ConsoleApplication6\\table_scene_lms400_inliers.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file sample.pcd \n");
		return (-1);
	}

	std::cout << "points sieze is:" << cloud->size() << std::endl;
	



	//计算点云平均间距 后期用点云间距（点云分辨率a），后期用10*a 估计法线
	//创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
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
		int nres=kdtree.nearestKSearch(cloud->points[i], k, nnh, squaredistance);
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
	cout << "everage distance is : " << everagedistance << endl;

	/******************************估计法线*******************************************/
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;  //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
	normEst.setInputCloud(cloud);
	normEst.setSearchMethod(tree);
	// normEst.setRadiusSearch(2);  //法向估计的半径
	normEst.setKSearch(9);  //法向估计的点数
	normEst.compute(*normals);
	cout << "normal size is " << normals->size() << endl;
	//normal_est.setViewPoint(0,0,0); //这个应该会使法向一致

	/**********************************估计边界****************************************************/
	pcl::PointCloud<pcl::Boundary> boundaries;  //保存边界估计结果
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;//定义一个进行边界特征估计的对象
	est.setInputCloud(cloud);
	est.setInputNormals(normals);/*M_PI_2 */
	est.setAngleThreshold(M_PI_2);   //在这里 由于构造函数已经对其进行了初始化 为Π/2 ，必须这样 使用 M_PI/2  M_PI_2 //边界估计时的角度阈值
	est.setSearchMethod(tree);
	est.setKSearch(20);  //一般这里的数值越高，最终边界识别的精度越好 20
									//  est.setRadiusSearch(everagedistance);  //搜索半径
	est.compute(boundaries);

	//  pcl::PointCloud<pcl::PointXYZ> boundPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> noBoundPoints;
	int countBoundaries = 0;
	for (int i = 0; i < cloud->size(); i++) {
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

	pcl::io::savePCDFileASCII("boudary.pcd", *boundPoints);
	pcl::io::savePCDFileASCII("NoBoundpoints.pcd", noBoundPoints);
	pcl::visualization::CloudViewer viewer("test");
	viewer.showCloud(boundPoints);
	while (!viewer.wasStopped())
	{
	}
	return 0;
}

/************************************计算点云分辨率(平均点间距)**********************************************/
//原文链接：https ://blog.csdn.net/qq_39707351/article/details/93470524
double computerCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size()/2; ++i)
	{
		if (!std::isfinite((*cloud)[i].x)|| !std::isfinite((*cloud)[i].y)|| !std::isfinite((*cloud)[i].z))
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
	return res;
}
