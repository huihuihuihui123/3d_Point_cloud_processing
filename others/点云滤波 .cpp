#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>//点云可视化
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Radius_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Radius_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Statictics_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud(new pcl::PointCloud<pcl::PointXYZ>);

int num = 0;
pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("filter Viewer"));
int v1(0), v2(0);//视口编号在这里设置两个视口
//点云框选事件函数
void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
	std::vector< int > indices;
	if (event.getPointsIndices(indices) == -1)
		return;
	for (int i = 0; i < indices.size(); ++i)
	{
		clicked_points_3d->points.push_back(cloud_Statictics_filtered->points.at(indices[i]));
		cout << clicked_points_3d->points[i].x << "," << clicked_points_3d->points[i].y << "," << clicked_points_3d->points[i].z << endl;
	}
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);
	std::stringstream ss;
	std::string cloudName;
	ss << num++;
	ss >> cloudName;
	cloudName += "_cloudName";

	viewer->addPointCloud(clicked_points_3d, red, cloudName, v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
}


	

int
main(int argc, char** argv)
{

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read("水管1.pcd", *cloud); // Remember to download the file first!

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ")." << endl;
#if 0
	/******************************************************************************
	  创建一个叶大小为1cm的pcl::VoxelGrid滤波器，
	**********************************************************************************/
	pcl::VoxelGrid<pcl::PointXYZ> sor;  //创建滤波对象
	sor.setInputCloud(cloud);            //设置需要过滤的点云给滤波对象
	sor.setLeafSize(0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
	sor.filter(*cloud_filtered);           //执行滤波处理，存储输出


	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

#endif

#if 1
	/*******************************过滤器RadiusOutlierRemoval去除孤立点******************************************/
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(0.01);
	outrem.setMinNeighborsInRadius(50);//30
	outrem.filter(*cloud_Radius_filtered);
	//pcl::io::savePCDFileASCII("水管after_Radius_filter.pcd", *cloud_Radius_filtered);
	std::cout << "Pointcloud after filtering: " << cloud_Radius_filtered->width*cloud_Radius_filtered->height << "data points" << endl;
#endif
#if 0
	/*******************************过滤器RadiusOutlierRemoval去除孤立点******************************************/
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem2;
	outrem2.setInputCloud(cloud_Radius_filtered);
	outrem2.setRadiusSearch(0.05);
	outrem2.setMinNeighborsInRadius(300);//30
	outrem2.filter(*cloud_Radius_filtered2);

	std::cout << "Pointcloud after filtering: " << cloud_Radius_filtered2->width*cloud_Radius_filtered2->height << "data points" << endl;
#endif

#if 1
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> st;
	st.setInputCloud(cloud_Radius_filtered);
	st.setMeanK(100);
	st.setStddevMulThresh(1);
	st.filter(*cloud_Statictics_filtered);
	std::cout << "Pointcloud after filtering: " << cloud_Statictics_filtered->width*cloud_Statictics_filtered->height << "data points" << endl;
	pcl::io::savePCDFileASCII("水管after_filter.pcd", *cloud_Statictics_filtered);
#endif
#if 1
	reader.read("水管after_Radius_static_filter.pcd", *cloud_Statictics_filtered);
	std::cout << "Pointcloud after filtering: " << cloud_Statictics_filtered->width*cloud_Statictics_filtered->height << "data points" << endl;
#endif 
#if 0
	/**********************mls*****************************************************/
	//说此类放在了Surface下面，但是通过反复的研究与使用，我发现此类并不能输出拟合后的表面，不能生成Mesh或者Triangulations，
	//	只是将点云进行了MLS的映射，使得输出的点云更加平滑。
	//因此，在我看来此类应该放在Filter下。通过多次的实验与数据的处理，我发现此类主要适用于点云的光顺处理，
	//	当然输入的点云最好是滤过离群点之后的点集，否则将会牺牲表面拟合精度的代价来获得输出点云。
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_mls(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	mls.setInputCloud(cloud_Statictics_filtered);
	//建立搜索对象
	mls.setSearchMethod(tree_mls);
	mls.setSearchRadius(0.03);//这个值越大，输出的点越多

	mls.process(*mls_points);
	pcl::io::savePCDFileASCII("水管mls_points.pcd", *mls_points);
	reader.read("水管mls_points.pcd", *mls_cloud);
	cout <<"mls: " <<mls_cloud->size() << endl;
#endif
#if 0
	/*******************************将法向量与坐标点连接起来***************************************/
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_Statictics_filtered);
	n.setInputCloud(cloud_Statictics_filtered);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	cout << "normal" << normals->size() << endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud_Statictics_filtered, *normals, *cloud_with_normals);

	cout << "cloud_with_normals" << endl;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	/**********************poisson reconstruction***********************/
	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setConfidence(false);//是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
	poisson.setDegree(2); //设置参数degree[1, 5], 值越大越精细，耗时越久
	poisson.setDepth(8); //树的最大深度，求解2^d x 2 ^ d x 2 ^ d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度
	poisson.setIsoDivide(8);//用于提取ISO等值面的算法的深度
	poisson.setManifold(true); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
	poisson.setOutputPolygons(true);//是否输出多边形网格（而不是三角化移动立方体的结果）
	poisson.setSamplesPerNode(3.0); // 设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0 - 5.0], 有噪声[15. - 20.]平滑
	poisson.setScale(1.25);//设置用于重构的立方体直径和样本边界立方体直径的比率。
	poisson.setSolverDivide(8);//设置求解线性方程组的Gauss-Seidel迭代方法的深度
	poisson.setSearchMethod(tree2);
	poisson.setInputCloud(cloud_with_normals);
	pcl::PolygonMesh mesh;//创建多变形网格，用于存储结果
	poisson.performReconstruction(mesh);//执行重构
#endif 
	/**********************poisson2 reconstruction***********************/
	//pcl::Poisson<pcl::PointNormal> poissonMLS;
	//poissonMLS.setConfidence(false);//是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
	//poissonMLS.setDegree(2); //设置参数degree[1, 5], 值越大越精细，耗时越久
	//poissonMLS.setDepth(8); //树的最大深度，求解2^d x 2 ^ d x 2 ^ d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度
	//poissonMLS.setIsoDivide(8);//用于提取ISO等值面的算法的深度
	//poissonMLS.setManifold(true); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
	//poissonMLS.setOutputPolygons(true);//是否输出多边形网格（而不是三角化移动立方体的结果）
	//poissonMLS.setSamplesPerNode(3.0); // 设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0 - 5.0], 有噪声[15. - 20.]平滑
	//poissonMLS.setScale(1.25);//设置用于重构的立方体直径和样本边界立方体直径的比率。
	//poissonMLS.setSolverDivide(8);//设置求解线性方程组的Gauss-Seidel迭代方法的深度
	//poissonMLS.setSearchMethod(tree2);
	//poissonMLS.setInputCloud(mls_points);
	//pcl::PolygonMesh meshMLS;//创建多变形网格，用于存储结果
	//poisson.performReconstruction(meshMLS);//执行重构
	/**************************************************单视窗口+两片点云******************************************************/
	boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("project points using  aparameyric model"));
	viewer->setBackgroundColor(0, 0, 0); //创建窗口 
	int v1;
	viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
	//设置点云颜色 
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud, 34, 25, 133); //投影前可以随便设一个颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_Statictics_filtered, 255, 255, 255);  //投影后的设置为白色
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, source_color, "source", v1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_Statictics_filtered, target_color, "projected", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "projected");
	viewer->spin();
	/**************************************************双视窗口*************************************************************/

	//viewer->initCameraParameters();
	//
	////5.1原始点云窗口
	//viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	//viewer->setBackgroundColor(0, 0, 0, v1);
	//viewer->addText("original", 10, 10, "v1 text", v1);
	//viewer->addPointCloud<pcl::PointXYZ>(cloud_Statictics_filtered, "sample cloud1", v1);
	////viewer->addPolygonMesh(mesh, "meshMLS", v1);
	////viewer->setRepresentationToPointsForAllActors(); // 以点的形式显示
	//viewer->addCoordinateSystem(1.0);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud1");
	////5.2滤波窗口

	//viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	//viewer->setBackgroundColor(0, 0, 0, v2);
	//viewer->addText("after filtered", 10, 10, "v2 text", v2);
	//viewer->addPolygonMesh(mesh, "mesh",v2);
	//// viewer->setRepresentationToPointsForAllActors(); // 以点的形式显示
	////viewer->setRepresentationToSurfaceForAllActors();  //以面的形式显示
	////viewer->setRepresentationToWireframeForAllActors(); // 以网格的形式显示
	////viewer->addPointCloud<pcl::PointXYZ>(mls_cloud, "sample cloud2",v2);
	//viewer->addCoordinateSystem(1.0);
	//viewer->registerAreaPickingCallback(pp_callback, (void*)&cloud_Statictics_filtered);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud2");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);  //刷新
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}