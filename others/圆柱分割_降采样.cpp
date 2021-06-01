#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <ctime>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>//计算重心

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <Eigen/Core>


#include <string>
#include <atlstr.h>//CString头文件



#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>


#include <pcl/segmentation/supervoxel_clustering.h>//超体素分割

#include <pcl/segmentation/lccp_segmentation.h>//LCCP分割

#include <pcl/segmentation/cpc_segmentation.h>
using namespace std;


typedef pcl::PointXYZ PointT;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;
//---------------读取txt点云文件,返回pcl点云类型-------------------
void CreateCloudFromTxt(const std::string& file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	std::ifstream file(file_path.c_str());//c_str()：生成一个const char*指针，指向以空字符终止的数组。
	std::string line;
	pcl::PointXYZ point;
	//float nx, ny, nz;
	while (getline(file, line)) {
		std::stringstream ss(line);
		ss >> point.x;
		ss >> point.y;
		ss >> point.z;
		cloud->push_back(point);
	}
	file.close();
}

/***************************点云降采样******************************/
void downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered,float grid_size)
{
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(grid_size, grid_size, grid_size);
	sor.filter(*cloud_filtered);

	std::cerr << "PointCloud after sample down: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";
}

/***************************滤除平面******************************/
void detectObjectsOnCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
{
	if (cloud->size() > 0)
	{
		//创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// 创建分割对象
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// 可选择配置，设置模型系数需要优化
		seg.setOptimizeCoefficients(true);
		// 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
		seg.setModelType(pcl::SACMODEL_PLANE);//设置模型类型
		seg.setMethodType(pcl::SAC_RANSAC);//设置随机采样一致性方法类型
		// you can modify the parameter below
		seg.setMaxIterations(1000);
		seg.setDistanceThreshold(0.001);//设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
		seg.setInputCloud(cloud);
		//引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			cout << "error! Could not found any inliers!" << endl;
		}
		// extract ground
		// 从点云中抽取分割的处在平面上的点集
		pcl::ExtractIndices<pcl::PointXYZ> extractor;//点提取对象
		extractor.setInputCloud(cloud);
		extractor.setIndices(inliers);
		extractor.setNegative(true);
		extractor.filter(*cloud_filtered);
		// vise-versa, remove the ground not just extract the ground
		// just setNegative to be true
		cout << "filter done." << endl;
	}
	else
	{
		cout << "no data!" << endl;
	}
}


void visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));

	// 添加需要显示的点云数据

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "example");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "example");

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

//

/***********************************区域生长法分割********************************************/
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
CloudSegmentWithRegionGrow(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	clock_t start_normal = clock();
	//设置搜索的方式或者说是结构
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	//求法线
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(30);
	normal_estimator.compute(*normals);


	clock_t end_normal = clock();
	cout << "Normal calculate： " << (double)(end_normal - start_normal) << " ms" << endl;
	//聚类对象<点，法线>
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(1000);  //最小的聚类的点数
	reg.setMaxClusterSize(5000);  //最大的
	reg.setSearchMethod(tree);    //搜索方式
	reg.setNumberOfNeighbours(30);    //设置搜索的邻域点的个数
	reg.setInputCloud(cloud);         //输入点
	//reg.setIndices (indices);
	reg.setInputNormals(normals);     //输入的法线
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);  //设置平滑度
	reg.setCurvatureThreshold(1.0);     //设置曲率的阀值

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);


	clock_t end_segment = clock();
	cout << "end_segment： " << (double)(end_segment - end_normal) << " ms" << endl;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Eucluextra; //用于储存欧式分割后的点云
	for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			cloud_cluster->points.push_back(cloud->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		Eucluextra.push_back(cloud_cluster);
	}
	return Eucluextra;
}

/********************************欧式聚类分割***********************************************/
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
CloudSegmentWithEuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	//设置搜索的方式或者说是结构
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	clock_t start_segment = clock();
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;//创建欧式聚类分割对象
	ec.setClusterTolerance(0.003); //设置近邻搜索的搜索半径
	ec.setMinClusterSize(800); //设置最小聚类尺寸
	ec.setMaxClusterSize(2000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	clock_t end_segment = clock();
	cout << "end_segment： " << (double)(end_segment - start_segment) << " ms" << endl;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Eucluextra; //用于储存欧式分割后的点云
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			cloud_cluster->points.push_back(cloud->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		Eucluextra.push_back(cloud_cluster);
	}
	return Eucluextra;
}

/*-------------------------主函数，首先进行平面分割，保存滤除平面后的点云cloud_down_sample.pcd-----------------------------------*/
#if 0
//*********************平面分割*********************************/
int main(int argc, char** argv) {


	clock_t start;
	start = clock();

	// -------------------加载点云----------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Ori(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down_sample(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_RemovePlane(new pcl::PointCloud<pcl::PointXYZ>);
	CreateCloudFromTxt("surface_pts_000.txt", cloud_Ori);
	clock_t end_read = clock();
	cout << "read_cloud： " << (double)(end_read - start) << " ms" << endl;
	// -----------------降采样---------------------
	downSample(cloud_Ori, cloud_down_sample,0.34/224.0);
	clock_t end_down = clock();
	cout << "cloud_down_sample： " << (double)(end_down - end_read) << " ms" << endl;
	// -----------------滤除平面---------------------
	detectObjectsOnCloud(cloud_down_sample, cloud_RemovePlane);
	cout <<"cloud_RemovePlane->size()=" <<cloud_RemovePlane->size() << endl;
	clock_t end_Remove_plane = clock();
	cout << "Remove_plane： " << (double)(end_Remove_plane - end_down) << " ms" << endl;

	// -----------------可视化点云---------------------
	pcl::io::savePCDFile("cloud_RemovePlane.pcd", *cloud_RemovePlane, true);
	pcl::io::savePCDFile("cloud_down_sample.pcd", *cloud_down_sample, true);
	visualization(cloud_RemovePlane);
	return 0;
}
#endif
/*-------------------------主函数，读取滤除平面后的点云cloud_down_sample.pcd，保存分割后的点云-----------------------------------*/
#if 1
int
main(int argc, char** argv)
{
	clock_t start;
	start = clock();

	// 读取点云数据
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read("cloud_RemovePlane.pcd", *cloud);

	//std::cout << "source cloud size=" << cloud->size() << endl;
	//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Eucluextra = CloudSegmentWithRegionGrow(cloud);//区域生长
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Eucluextra = CloudSegmentWithEuclideanCluster(cloud);//欧式聚类分割
	
	//计算最上面的一幅点云
	Eigen::Vector4f centroid;
	priority_queue<pair<float, int>> que;
	int max_NumOfPoints_Index = 0;
	for (int i = 0; i < Eucluextra.size(); i++)
	{
		pcl::compute3DCentroid(*(Eucluextra[i]), centroid);
		float center_z = centroid[2];
		que.push(make_pair(center_z, i));
	}
	//按照Z坐标从上到下排序
	for (int i = 0; i < Eucluextra.size(); i++)
	{
		int max_Top_Index = que.top().second;
		string str = "segment";
		//保存分割后的点云segment0.pcd、segment1.pcd、segment2.pcd....
		pcl::io::savePCDFile(str +to_string(i)+".pcd", *Eucluextra[max_Top_Index]);
		que.pop();
	}
	
	cout << "Cluster.size()=" << Eucluextra.size() << endl;
	//可视化
	pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
	viewer.initCameraParameters();

	int v1(0);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v1);
	viewer.addText("Cloud before segmenting", 10, 10, "v1 test", v1);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);

	int v2(0);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v2);
	viewer.addText("Cloud after segmenting", 10, 10, "v2 test", v2);
	for (int i = 0; i < Eucluextra.size(); i++)
	{
		string str_filename = "cloud";
		str_filename += std::to_string(i) + ".pcd";
		//显示分割得到的各片点云 
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(Eucluextra[i], 255 * (1 - i)*(2 - i) / 2, 255 * i*(2 - i), 255 * i*(i - 1) / 2);
		viewer.addPointCloud(Eucluextra[i], color, str_filename, v2);
	}

	cout << "分割圆柱数目： " << Eucluextra.size() << endl;
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return (0);
}
#endif



