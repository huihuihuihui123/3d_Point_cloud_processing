#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <ctime>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


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
//---------------��ȡtxt�ļ�-------------------
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
		//ss >> nx;
		//ss >> ny;
		//ss >> nz;
		cloud->push_back(point);
	}
	file.close();
}


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
		seg.setMaxIterations(1000);//��ʾ�㵽����ģ�͵ľ������ֵ��
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


	//	std::vector<int> inliers;    //�洢���ڵ㼯�ϵĵ������������

	////����RANSACƽ�����
	//	pcl::SampleConsensusModelPlane<PointT>::Ptr    model_p(new pcl::SampleConsensusModelPlane<PointT>(cloud));        //���ƽ��ģ�͵Ķ���
	//	pcl::RandomSampleConsensus<PointT> ransacP(model_p);
	//	ransacP.setDistanceThreshold(0.0005);        //��ƽ�����С��1mm�ĵ���Ϊ���ڵ㿼��
	//	ransacP.computeModel();                    //ִ�������������
	//	ransacP.getInliers(inliers);                //�洢�������õľ��ڵ�
	//	pcl::copyPointCloud<PointT>(*cloud, inliers, *cloud_filtered);    //���ƹ���ģ�͵����о��ڵ㵽cloud_in��
	//	//pcl::io::savePCDFile("./data/seg_RAN/RANSAC_building_1.pcd", *cloud_in);
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
///*********************ƽ��ָ�*********************************/
//int main(int argc, char** argv) {
//
//
//	clock_t start;
//	start = clock();
//
//	// -------------------���ص���----------------------
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Ori(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_RemovePlane(new pcl::PointCloud<pcl::PointXYZ>);
//	CreateCloudFromTxt("surface_pts_010.txt", cloud_Ori);
//
//	clock_t end_read = clock();
//	cout << "read_cloud�� " << (double)(end_read - start) << " ms" << endl;
//
//	detectObjectsOnCloud(cloud_Ori, cloud_RemovePlane);
//	cout <<"cloud_RemovePlane->size()=" <<cloud_RemovePlane->size() << endl;
//	clock_t end_Remove_plane = clock();
//	cout << "Remove_plane�� " << (double)(end_Remove_plane - end_read) << " ms" << endl;
//
//	// -----------------���ӻ�����---------------------
//	visualization(cloud_RemovePlane);
//	pcl::io::savePCDFile("cloud_RemovePlane.pcd", *cloud_RemovePlane, true);
//
//	return 0;
//}


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
	ec.setClusterTolerance(0.001); //���ý��������������뾶
	ec.setMinClusterSize(1000); //������С����ߴ�
	ec.setMaxClusterSize(5000);
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


int CloudSegmentWithSuperSupervoxel(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	//�趨�ᾧ����
	float voxel_resolution = 0.008f;
	float seed_resolution = 0.1f;
	float color_importance = 0.2f;
	float spatial_importance = 0.4f;
	float normal_importance = 1.0f;

	//���ɽᾧ��
	pcl::SupervoxelClustering<pcl::PointXYZ> super(voxel_resolution, seed_resolution);
	super.setUseSingleCameraTransform(false);
	//������Ƽ��ᾧ����
	super.setInputCloud(cloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	//����ᾧ�ָ����������һ��ӳ���
	std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZ>::Ptr > supervoxel_clusters;
	super.extract(supervoxel_clusters);  //��þ�������  
	pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();  //��þ��� 
	pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	viewer->addPointCloud(voxel_centroid_cloud, "voxel centroids");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "voxel centroids");     //��Ⱦ����
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.95, "voxel centroids");

	viewer->addPointCloud(labeled_voxel_cloud, "labeled voxels");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "labeled voxels");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
	return (0);

}
int
main(int argc, char** argv)
{
	clock_t start;
	start = clock();

	//// -------------------���ص���----------------------
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Ori(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_RemovePlane(new pcl::PointCloud<pcl::PointXYZ>);
	CreateCloudFromTxt("surface_pts_020.txt", cloud_Ori);

	clock_t end_read = clock();
	cout << "read_cloud�� " << (double)(end_read - start) << " ms" << endl;

	detectObjectsOnCloud(cloud_Ori, cloud_RemovePlane);
	cout <<"cloud_RemovePlane->size()=" <<cloud_RemovePlane->size() << endl;
	clock_t end_Remove_plane = clock();
	cout << "Remove_plane�� " << (double)(end_Remove_plane - end_read) << " ms" << endl;


	pcl::io::savePCDFile("cloud_RemovePlane.pcd", *cloud_RemovePlane, true);*/

	// ��ȡ��������
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read("cloud_RemovePlane.pcd", *cloud);

	//std::cout << "source cloud size=" << cloud->size() << endl;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Eucluextra = CloudSegmentWithRegionGrow(cloud);//��������
	//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Eucluextra = CloudSegmentWithEuclideanCluster(cloud);//ŷʽ����ָ�
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

	cout <<"�ָ�Բ����Ŀ�� " <<Eucluextra.size() << endl;
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return (0);
}




