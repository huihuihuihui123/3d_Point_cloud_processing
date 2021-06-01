#include <pcl/point_types.h>                  //点类型头文件
#include <pcl/features/pfh.h>                 //pfh特征估计类头文件
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

//struct PFHSignature125
//{
//	float histogram[125];
//};
//默认的PFH实现使用5个分箱细分（例如，四个特征值中的每一个将从其值间隔使用这个多个分箱），
//并且不包括距离（如上所述 - 尽管 用户可以调用computePairFeatures方法如果需要，也获得距离，这导致5 ^ 3浮点值的125字节数组（）。它们存储在pcl::PFHSignature125点类型中。
int main()
{
	//1.加载点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("table_scene_lms400_downsampled.pcd", *cloud);

	//2.估计法线向量
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
		//创建一个空的kdtree对象，并把它传递给法线估计对象
		//基于给出的输入数据集，kdtree将被建立
	pcl::search::KdTree<pcl::PointXYZ>::Ptr Netree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(Netree);
		//输出数据集
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		//使用半径在查询点周围3厘米范围内的所有邻元素
	ne.setRadiusSearch(0.03);
		//计算特征值
	ne.compute(*cloud_normals);
	//3.创建PFH估计对象pfh，并将输入点云数据集cloud和法线normals传递给它

	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;

	pfh.setInputCloud(cloud);

	pfh.setInputNormals(cloud_normals);

		//如果点云是类型为PointNormal,则执行pfh.setInputNormals (cloud);

	//4.创建一个空的kd树表示法，并把它传递给PFH估计对象。

		//基于已给的输入数据集，建立kdtree

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pfh.setSearchMethod(tree);

	//输出数据集

	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());

	//使用半径在5厘米范围内的所有邻元素。

	//注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!

	pfh.setRadiusSearch(0.05);

	//计算pfh特征值

	pfh.compute(*pfhs);

	// pfhs->points.size ()应该与input cloud->points.size ()有相同的大小，即每个点都有一个pfh特征向量
	cout << pfhs->points.size() << endl;
	cout << cloud->points.size() << endl;
	getchar();
	return 0;
}
