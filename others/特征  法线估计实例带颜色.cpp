#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>  //法线估计类头文件
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

//（实现对输入点云数据集中的点估计一组表面法线）执行的操作是：
//对应点云P中每一个点p得到p点最近邻元素，计算p点的表面的法线N，检查N的方向是否指向视点如果不是则翻转。
int
main()
{
	//打开点云代码
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("table_scene_lms400.pcd", *cloud);
	//创建法线估计估计向量
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	//创建一个空的KdTree对象，并把它传递给法线估计向量
	//基于给出的输入数据集，KdTree将被建立
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	//存储输出数据
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	//使用半径在查询点周围3厘米范围内的所有临近元素
	//ne.setRadiusSearch(0.03);
	ne.setKSearch(10);//寻找距离最近的10个点作为
	//计算特征值
	ne.compute(*cloud_normals);

	// cloud_normals->points.size ()应该与input cloud_downsampled->points.size ()有相同的尺寸
	//可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud, 0, 0, 255);
	viewer->setBackgroundColor(0.0, 0.0, 0.0);
	viewer->addPointCloud(cloud, source_cloud_color_handler, "original_cloud");
	cout << cloud->points.size() << endl;
	cout << cloud_normals->points.size() << endl;
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals);


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	return 0;
}