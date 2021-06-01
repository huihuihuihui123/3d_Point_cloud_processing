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
#include <pcl/common/centroid.h>//��������

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <Eigen/Core>


#include <string>
#include <atlstr.h>//CStringͷ�ļ�



#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>


#include <pcl/segmentation/supervoxel_clustering.h>//�����طָ�

#include <pcl/segmentation/lccp_segmentation.h>//LCCP�ָ�

#include <pcl/segmentation/cpc_segmentation.h>
using namespace std;


typedef pcl::PointXYZ PointT;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;
//---------------��ȡtxt�����ļ�,����pcl��������-------------------
void CreateCloudFromTxt(const std::string& file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	std::ifstream file(file_path.c_str());//c_str()������һ��const char*ָ�룬ָ���Կ��ַ���ֹ�����顣
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

/***************************���ƽ�����******************************/
void downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered,float grid_size)
{
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(grid_size, grid_size, grid_size);
	sor.filter(*cloud_filtered);

	std::cerr << "PointCloud after sample down: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";
}

/***************************�˳�ƽ��******************************/
void detectObjectsOnCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
{
	if (cloud->size() > 0)
	{
		//�����ָ�ʱ����Ҫ��ģ��ϵ������coefficients���洢�ڵ�ĵ��������϶���inliers
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// �����ָ����
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// ��ѡ�����ã�����ģ��ϵ����Ҫ�Ż�
		seg.setOptimizeCoefficients(true);
		// ��Ҫ�����ã����÷ָ��ģ�����ͣ����õ�����������Ʒ��������뷧ֵ���������
		seg.setModelType(pcl::SACMODEL_PLANE);//����ģ������
		seg.setMethodType(pcl::SAC_RANSAC);//�����������һ���Է�������
		// you can modify the parameter below
		seg.setMaxIterations(1000);
		seg.setDistanceThreshold(0.001);//�趨���뷧ֵ�����뷧ֵ�����˵㱻��Ϊ�Ǿ��ڵ��Ǳ������������
		seg.setInputCloud(cloud);
		//�����ָ�ʵ�֣��洢�ָ������㼸��inliers���洢ƽ��ģ�͵�ϵ��coefficients
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			cout << "error! Could not found any inliers!" << endl;
		}
		// extract ground
		// �ӵ����г�ȡ�ָ�Ĵ���ƽ���ϵĵ㼯
		pcl::ExtractIndices<pcl::PointXYZ> extractor;//����ȡ����
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

	// �����Ҫ��ʾ�ĵ�������

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "example");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "example");

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

//

/***********************************�����������ָ�********************************************/
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
CloudSegmentWithRegionGrow(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	clock_t start_normal = clock();
	//���������ķ�ʽ����˵�ǽṹ
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	//����
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(30);
	normal_estimator.compute(*normals);


	clock_t end_normal = clock();
	cout << "Normal calculate�� " << (double)(end_normal - start_normal) << " ms" << endl;
	//�������<�㣬����>
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(1000);  //��С�ľ���ĵ���
	reg.setMaxClusterSize(5000);  //����
	reg.setSearchMethod(tree);    //������ʽ
	reg.setNumberOfNeighbours(30);    //���������������ĸ���
	reg.setInputCloud(cloud);         //�����
	//reg.setIndices (indices);
	reg.setInputNormals(normals);     //����ķ���
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);  //����ƽ����
	reg.setCurvatureThreshold(1.0);     //�������ʵķ�ֵ

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);


	clock_t end_segment = clock();
	cout << "end_segment�� " << (double)(end_segment - end_normal) << " ms" << endl;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Eucluextra; //���ڴ���ŷʽ�ָ��ĵ���
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

/********************************ŷʽ����ָ�***********************************************/
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
CloudSegmentWithEuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	//���������ķ�ʽ����˵�ǽṹ
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	clock_t start_segment = clock();
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;//����ŷʽ����ָ����
	ec.setClusterTolerance(0.003); //���ý��������������뾶
	ec.setMinClusterSize(800); //������С����ߴ�
	ec.setMaxClusterSize(2000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	clock_t end_segment = clock();
	cout << "end_segment�� " << (double)(end_segment - start_segment) << " ms" << endl;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Eucluextra; //���ڴ���ŷʽ�ָ��ĵ���
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

/*-------------------------�����������Ƚ���ƽ��ָ�����˳�ƽ���ĵ���cloud_down_sample.pcd-----------------------------------*/
#if 0
//*********************ƽ��ָ�*********************************/
int main(int argc, char** argv) {


	clock_t start;
	start = clock();

	// -------------------���ص���----------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Ori(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down_sample(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_RemovePlane(new pcl::PointCloud<pcl::PointXYZ>);
	CreateCloudFromTxt("surface_pts_000.txt", cloud_Ori);
	clock_t end_read = clock();
	cout << "read_cloud�� " << (double)(end_read - start) << " ms" << endl;
	// -----------------������---------------------
	downSample(cloud_Ori, cloud_down_sample,0.34/224.0);
	clock_t end_down = clock();
	cout << "cloud_down_sample�� " << (double)(end_down - end_read) << " ms" << endl;
	// -----------------�˳�ƽ��---------------------
	detectObjectsOnCloud(cloud_down_sample, cloud_RemovePlane);
	cout <<"cloud_RemovePlane->size()=" <<cloud_RemovePlane->size() << endl;
	clock_t end_Remove_plane = clock();
	cout << "Remove_plane�� " << (double)(end_Remove_plane - end_down) << " ms" << endl;

	// -----------------���ӻ�����---------------------
	pcl::io::savePCDFile("cloud_RemovePlane.pcd", *cloud_RemovePlane, true);
	pcl::io::savePCDFile("cloud_down_sample.pcd", *cloud_down_sample, true);
	visualization(cloud_RemovePlane);
	return 0;
}
#endif
/*-------------------------����������ȡ�˳�ƽ���ĵ���cloud_down_sample.pcd������ָ��ĵ���-----------------------------------*/
#if 1
int
main(int argc, char** argv)
{
	clock_t start;
	start = clock();

	// ��ȡ��������
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read("cloud_RemovePlane.pcd", *cloud);

	//std::cout << "source cloud size=" << cloud->size() << endl;
	//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Eucluextra = CloudSegmentWithRegionGrow(cloud);//��������
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Eucluextra = CloudSegmentWithEuclideanCluster(cloud);//ŷʽ����ָ�
	
	//�����������һ������
	Eigen::Vector4f centroid;
	priority_queue<pair<float, int>> que;
	int max_NumOfPoints_Index = 0;
	for (int i = 0; i < Eucluextra.size(); i++)
	{
		pcl::compute3DCentroid(*(Eucluextra[i]), centroid);
		float center_z = centroid[2];
		que.push(make_pair(center_z, i));
	}
	//����Z������ϵ�������
	for (int i = 0; i < Eucluextra.size(); i++)
	{
		int max_Top_Index = que.top().second;
		string str = "segment";
		//����ָ��ĵ���segment0.pcd��segment1.pcd��segment2.pcd....
		pcl::io::savePCDFile(str +to_string(i)+".pcd", *Eucluextra[max_Top_Index]);
		que.pop();
	}
	
	cout << "Cluster.size()=" << Eucluextra.size() << endl;
	//���ӻ�
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
		//��ʾ�ָ�õ��ĸ�Ƭ���� 
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(Eucluextra[i], 255 * (1 - i)*(2 - i) / 2, 255 * i*(2 - i), 255 * i*(i - 1) / 2);
		viewer.addPointCloud(Eucluextra[i], color, str_filename, v2);
	}

	cout << "�ָ�Բ����Ŀ�� " << Eucluextra.size() << endl;
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return (0);
}
#endif



