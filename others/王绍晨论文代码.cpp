#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>//ͳ���˲�
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>//�����²���
#include <pcl/filters/passthrough.h>//�����˲�
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/features/normal_3d.h>//normal���
#include<pcl/kdtree/kdtree_flann.h>//kd_tree
#include<pcl/kdtree/io.h>//kd_tree
#include <pcl/surface/gp3.h>//̰�����ǻ�  pcl::GreedyProjectionTriangulation
#include <pcl/segmentation/region_growing.h>// ƽ��ָ� ����������
#include <pcl/common/centroid.h>//�����������

#include <pcl/features/boundary.h>//�߽���ȡ
#include"cseg.h"

#include <ctime>//��������ʱ�����
//������ �����²���
int third1()
{
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_VoxelGrid(new pcl::PCLPointCloud2());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3_VoxelGrid(new pcl::PointCloud<pcl::PointXYZ>);
	//���ƶ���Ķ�ȡ
	pcl::PCDReader reader;
	reader.read("C:\\Users\\Administrator\\Desktop\\����3\\1plyCloud.pcd", *cloud);
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";
	pcl::fromPCLPointCloud2(*cloud, *cloud1);		/******************************************************************************
	   ����һ��pcl::VoxelGrid�˲�����
   **********************************************************************************/
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //�����˲�����
	sor.setInputCloud(cloud);            //������Ҫ���˵ĵ��Ƹ��˲�����
	sor.setLeafSize(0.07, 0.07, 0.07);  //�����˲�ʱ�������������Ϊ1cm��������
	sor.filter(*cloud_VoxelGrid);           //ִ���˲������洢���
	std::cerr << "PointCloud after filtering: " << cloud_VoxelGrid->width * cloud_VoxelGrid->height
		<< " data points (" << pcl::getFieldsList(*cloud_VoxelGrid) << ").";
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud1);
	pcl::PointXYZ searchPoint;
	pcl::fromPCLPointCloud2(*cloud_VoxelGrid, *cloud3_VoxelGrid);
	/*searchPoint.x = cloud1->points[0].x;
	searchPoint.y = cloud1->points[0].y;
	searchPoint.z= cloud1->points[0].z;*/
	// ����ǽ��ڸ���
	int K = 1;

	// ���������������ֱ��Ž��ڵ�����ֵ�����ڵ����ľ�
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	cloud2->width = cloud_VoxelGrid->width;
	cloud2->height = cloud_VoxelGrid->height;
	cloud2->points.resize(cloud2->width * cloud2->height);

	//Ѱ�Ҿ�����������ĵ�

	for (int j = 0; j < cloud_VoxelGrid->width*cloud_VoxelGrid->height; j++)
	{
		searchPoint.x = cloud3_VoxelGrid->points[j].x;
		searchPoint.y = cloud3_VoxelGrid->points[j].y;
		searchPoint.z = cloud3_VoxelGrid->points[j].z;
		//����ִ�� K ���ڲ��ҵĳ�Ա����������һ���жϲ��ҳɹ���ִ��������
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
	//��������
	pcl::io::savePCDFileASCII("C:\\Users\\Administrator\\Desktop\\����3\\11table_scene_lms400_downsampled7.pcd", *cloud2);
	return (0);

	return 0;
}

//����ȥ��

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

	//�²���֮����ȥ����Լӿ촦���ٶ�

	 //pcl::VoxelGrid<pcl::PointXYZ> sor;         //�����˲�����
	 //sor.setInputCloud(cloud);       //������Ҫ���˵ĵ��Ƹ��˲�����
	 //sor.setLeafSize(0.07, 0.07, 0.07);         //�����˲�ʱ�������������Ϊ7cm��������
	 //sor.filter(*cloud_filtered);               //ִ���˲������洢���
	pcl::io::savePCDFileASCII("C:\\Users\\Administrator\\Desktop\\test_pcdchanpin.pcd", *cloud_filtered);
	//std::cerr << "Cloud before filtering: " << std::endl;
	//std::cerr << *cloud_filtered << std::endl;
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;
	sor1.setInputCloud(cloud_filtered);
	sor1.setMeanK(100);//100���ٽ���
	sor1.setStddevMulThresh(1.5);//�������1.5����׼����
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


//���ĵ����´���˵��
//����ƽ��ָ���ϴ���
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
	begin = clock();  //��ʼ��ʱ
	// Load PCD file
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("./turn_pointcloud360 - Cloud.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file sample.pcd \n");
		return (-1);
	}
	endread = clock();
	double readTimes = double(endread - begin) / CLOCKS_PER_SEC; //��clock()�����Ľ��ת��Ϊ����Ϊ��λ����

	std::cout << "load pcd time: " << readTimes << "s" << std::endl;

	// ���������˲�������
	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setInputCloud(cloud);            //�����������
	//pass.setFilterFieldName("y");         //���ù���ʱ����Ҫ�������͵�y�ֶ�
	//pass.setFilterLimits(20.0, 24.5);
	//pass.filter(*cloud_filteredy);//�����ڹ����ֶεķ�Χ
	//pcl::PassThrough<pcl::PointXYZ> passx;
	//passx.setInputCloud(cloud_filteredy);
	//passx.setFilterFieldName("x");               //���ù���ʱ����Ҫ�������͵�x�ֶ�
	//passx.setFilterLimits(0.8, 4.3);  			//pass.setFilterLimitsNegative (true);   //���ñ�����Χ�ڻ��ǹ��˵���Χ��
	//passx.filter(*cloud_filteredxy);            //ִ���˲���������˽����cloud_filtered
	///*pcl::io::savePCDFileASCII("C:\\Users\\Administrator\\Desktop\\test_pcd2.pcd", *cloud_filteredxy);*/

	//�����²���
	pcl::VoxelGrid<pcl::PointXYZ> sor;         //�����˲�����
	sor.setInputCloud(cloud);       //������Ҫ���˵ĵ��Ƹ��˲�����
	sor.setLeafSize(0.5, 0.5, 0.5);         //�����˲�ʱ�������������Ϊ7cm��������
	sor.filter(*cloud_filtered);               //ִ���˲������洢���

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";
	//pcl::io::savePCDFileASCII("C:\\Users\\Administrator\\Desktop\\test_pcd1111.pcd", *cloud_filtered);


	//���ּ���
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud_filtered);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	////̰�����ǻ� �����ؽ�
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	//pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	//tree2->setInputCloud(cloud_with_normals);
	//pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//pcl::PolygonMesh triangles;  //����������������ڴ洢���
	//gp3.setSearchRadius(5);//�����뾶���趨����������������û�ָ���������������ؽ��������εĴ�С��
	//gp3.setMu(2.5);//mu�Ǹ���Ȩ���ӣ�����ÿ���ο��㣬��ӳ����ѡ��İ뾶��mu����ο��������ľ���˻��������������ͺܺý���˵����ܶȲ����ȵ����⣬muһ��ȡֵΪ2.5-3��
	//gp3.setMaximumNearestNeighbors(100);//)�ٽ�����ֵ�趨��һ��Ϊ80-100��

	////����ķ������ǶȲ���ڴ�ֵ�������㽫�������ӳ������Σ������ǡǡ�͵��ƾֲ�ƽ����Լ����Ӧ��
	////���һ�����Ǽ������ô�����������Χ�ĵ���������Σ���ʵ���Ҳ��Ч���˵���һЩ��Ⱥ�㡣���ֵһ��Ϊ45�ȡ�
	//gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees ���ƽ���
	//gp3.setMinimumAngle(M_PI / 18); // 10 degrees �����νǶ�Լ��
	//gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	//gp3.setNormalConsistency(false);//����ķ������Ƿ������仯�ġ����һ��������false����������ĵ�����ȫ�ֹ⻬�ģ�����˵һ���򣩡�

	//// ���������������������
	//gp3.setInputCloud(cloud_with_normals);
	//gp3.setSearchMethod(tree2);

	////ִ���ع������������triangles��
	//gp3.reconstruct(triangles);

	////��������ͼ
	//pcl::io::saveOBJFile("./mesh1.obj", triangles);

	time_t endreconstruction = 0;
	endreconstruction = clock();  //��ʼ��ʱ
	double reconstructTimes = double(endreconstruction - endread) / CLOCKS_PER_SEC; //��clock()�����Ľ��ת��Ϊ����Ϊ��λ����

	std::cout << "reconstructTimes pcd time: " << reconstructTimes << "s" << std::endl;

	//���������㷨 �ָ����

	/*����ԭ��������Ҫ���ף����������Ǵ�����С����ֵ(curvature value)�ĵ㿪ʼ�ġ���ˣ����Ǳ���������������ֵ���������ǽ�������������Ϊ������С�ĵ�λ��ƽ̹���򣬶�����ƽ̹�������������Լ��������������������������������������̣�

		1.��������δ��ǵ㣬���յ������ֵ�Ե���������ҵ���С����ֵ�㣬��������ӵ����ӵ㼯��

		2.����ÿ�����ӵ㣬�㷨���ᷢ���ܱߵ����н��ڵ㡣1������ÿ�����ڵ��뵱ǰ���ӵ�ķ��߽ǶȲ�(reg.setSmoothnessThreshold)�������ֵС�����õ���ֵ����ý��ڵ㱻�ص㿼�ǣ����еڶ������ԣ�2���ý��ڵ�ͨ���˷��߽ǶȲ���飬�����������С�������趨����ֵ(reg.setCurvatureThreshold)�������ͱ���ӵ����ӵ㼯�������ڵ�ǰƽ�档

		3.ͨ�����μ���ĵ㣬����ԭʼ����ȥ����

		4.������С��صĵ���min��reg.setMinClusterSize���������Ϊmax��reg.setMaxClusterSize����

		4.�ظ�1 - 3�����㷨�����ɵ�����min��max������ƽ�棬���Բ�ͬƽ���ǲ�ͬ��ɫ�������֡�

		5.ֱ���㷨��ʣ��������ɵĵ�ز�������min���㷨ֹͣ������*/
		
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(1000);//����ƽ����������ٵ�������������ǳ���Ҫ��С�����������ƽ��ᱻ���Բ��ƣ�
	reg.setMaxClusterSize(1000000);//�������ĵ�����ԭ��ͬ�ϣ�����һ������ϣ��������������Կ������һ�㣬
	reg.setSearchMethod(tree);//��������������֮ǰ����������ʹ��kd���ķ���������ֱ���þͿ����ˣ���Ҳ��Ĭ�ϵķ�����
	reg.setNumberOfNeighbours(30);//���òο������������Ҳ���ǿ����ܱߵĶ��ٸ�������������һ��ƽ��
	//���������������Ҫ������������ݴ��ʣ�������õĺܴ���ô��ȫ�ֽǶȿ�ĳһ������΢�е���Ҳ���Խ��ܣ�������õĺ�С��ͨ����⵽��ƽ�涼���С��
	reg.setInputCloud(cloud_filtered);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(0.5 / 180.0 * M_PI);//�����жϵ���ֵ���������������ڶ��ļн��ڻ����Ե����ǹ���ģ�
	//����ÿ�����ڵ��뵱ǰ���ӵ�ķ��߽ǶȲ�����ֵС�����õ���ֵ����ý��ڵ㱻�ص㿼��

	reg.setCurvatureThreshold(1);


	//���԰ѽ�������һ�������棬����ػ��Զ���ÿ��ƽ��ֳ�һ��vector�����Դ�ӡ��������
	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;
	std::cout << std::endl;

	time_t endmesh = 0;
	endmesh = clock();  //��ʼ��ʱ
	double meshTimes = double(endmesh - endreconstruction) / CLOCKS_PER_SEC; //��clock()�����Ľ��ת��Ϊ����Ϊ��λ����

	std::cout << "endmesh pcd time: " << meshTimes << "s" << std::endl;


	// �����洢�������ĵĶ���
	Eigen::Vector4f centroid;

	pcl::compute3DCentroid(*cloud_filtered, centroid);

	std::cout << "The XYZ coordinates of the centroid are: ("
		<< centroid[0] << ", "
		<< centroid[1] << ", "
		<< centroid[2] << ")." << std::endl;

	

	//����
	for (std::vector<int>::const_iterator pit = clusters[0].indices.begin(); pit != clusters[0].indices.end(); ++pit)
	{
		cloud_cluster1->points.push_back(cloud_filtered->points[*pit]);
	}


	//��ƽ�����
	coefficients[1] = csegc[0].comcof(cloud_cluster1);
	//ǰ��
	for (std::vector<int>::const_iterator pit = clusters[2].indices.begin(); pit != clusters[2].indices.end(); ++pit)
	{
		cloud_cluster2->points.push_back(cloud_filtered->points[*pit]);
	}
	coefficients[2] = csegc[1].comcof(cloud_cluster2);
	//����
	for (std::vector<int>::const_iterator pit = clusters[3].indices.begin(); pit != clusters[3].indices.end(); ++pit)
	{
		cloud_cluster3->points.push_back(cloud_filtered->points[*pit]);
	}
	coefficients[0] = csegc[2].comcof(cloud_cluster3);
	//����
	for (std::vector<int>::const_iterator pit = clusters[1].indices.begin(); pit != clusters[1].indices.end(); ++pit)
	{
		cloud_cluster4->points.push_back(cloud_filtered->points[*pit]);
	}
	coefficients[3] = csegc[3].comcof(cloud_cluster4);
	//����
	for (std::vector<int>::const_iterator pit = clusters[5].indices.begin(); pit != clusters[5].indices.end(); ++pit)
	{
		cloud_cluster5->points.push_back(cloud_filtered->points[*pit]);
	}
	coefficients[4] = csegc[4].comcof(cloud_cluster5);
	for (int i = 0; i < 5; i++)
	{
		cout << "�����:" << coefficients[i]->values[0] << " " << coefficients[i]->values[1] << " " << coefficients[i]->values[2] << " " << coefficients[i]->values[3] << endl;
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
	//����ƽ�淽�������󽻵�
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

	ofstream out("C:\\Users\\Administrator\\Desktop\\�ĶԵ�", ios::app);
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

////������ ��תƽ�ƾ�����ȡ
//int getRT()
//{
//	//��תƽ�ƾ�����ȡ
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
//	//���ļ��ж�ȡ���
//	FILE * fRead;
//	fRead = fopen("C:\\Users\\Administrator\\Desktop\\Բ����\\3��Բ������.txt", "r");
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
//	//����SVD�������任����
//	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
//	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation2;
//	TESVD.estimateRigidTransformation(*cloud_in1, *cloud_out1, transformation2);
//	//����任������Ϣ
//	std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << std::endl;
//	printf("\n");
//	printf("    | %6.3f %6.3f %6.3f | \n", transformation2(0, 0), transformation2(0, 1), transformation2(0, 2));
//	printf("R = | %6.3f %6.3f %6.3f | \n", transformation2(1, 0), transformation2(1, 1), transformation2(1, 2));
//	printf("    | %6.3f %6.3f %6.3f | \n", transformation2(2, 0), transformation2(2, 1), transformation2(2, 2));
//	printf("\n");
//	printf("t = < %0.3f, %0.3f, %0.3f >\n", transformation2(0, 3), transformation2(1, 3), transformation2(2, 3));
//	return 0;
//}


/********************************������ ****************************************/

////���Ʊ�Ե��ȡ
//normEST.setRadiusSearch(reforn)������Ϊ�ֱ��ʵ�10��ʱ��Ч���Ϻã���Ҫ�Ƕ��ڷ��߹��ơ�
//	����뾶ѡ��̫С�ˣ������ϴ󣬹��Ƶķ��߾����׳�������������뾶���õ�̫������ٶȾͱȽ�����
//boundEst.setRadiusSearch(re)��Ҳ����10����̫С���ڲ��ĺܶ��Ͷ����ɱ߽���ˡ�
//  ���һ�������Ǳ߽��ж�ʱ�ĽǶ���ֵ��Ĭ��ֵΪPI / 2���˴�����ΪPI / 4���û�Ҳ���Ը�����Ҫ���и��ġ�

int main()
{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("F:\\��ά���ƴ������\\��ά���ƴ������\\��ά���ƴ������\\pcl����\\pcl����\\�޳���Ⱥ��\\ConsoleApplication6\\table_scene_lms400_inliers.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file sample.pcd \n");
		return (-1);
	}

	std::cout << "points sieze is:" << cloud->size() << std::endl;
	



	//�������ƽ����� �����õ��Ƽ�ࣨ���Ʒֱ���a����������10*a ���Ʒ���
	//����һ������k���ڲ�ѯ,��ѯ��ʱ�����õ��ڵ����У����һ�����ڵ����䱾��
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	int k = 2;
	float everagedistance = 0;
	size_t npoints = 0; //��Ч����
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
		/*std::cout << "��ѯ��λ�� " << cloud->points[i] << std::endl;
		std::cout << "����Ϊ�� " << nnh[0] << "  " << nnh[1] << std::endl;
		std::cout << "����Ϊ�� " << cloud->points[nnh[0]] << "  " << cloud->points[nnh[1]] << std::endl;
		*/
		
		//   cout<<everagedistance<<endl;

	}
	if (npoints != 0)
	{
		everagedistance = everagedistance / npoints;
	}
	cout << "everage distance is : " << everagedistance << endl;

	/******************************���Ʒ���*******************************************/
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;  //����pcl::PointXYZ��ʾ�����������ݣ�pcl::Normal��ʾ�������,��pcl::Normalǰ�����Ƿ������һ��������
	normEst.setInputCloud(cloud);
	normEst.setSearchMethod(tree);
	// normEst.setRadiusSearch(2);  //������Ƶİ뾶
	normEst.setKSearch(9);  //������Ƶĵ���
	normEst.compute(*normals);
	cout << "normal size is " << normals->size() << endl;
	//normal_est.setViewPoint(0,0,0); //���Ӧ�û�ʹ����һ��

	/**********************************���Ʊ߽�****************************************************/
	pcl::PointCloud<pcl::Boundary> boundaries;  //����߽���ƽ��
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;//����һ�����б߽��������ƵĶ���
	est.setInputCloud(cloud);
	est.setInputNormals(normals);/*M_PI_2 */
	est.setAngleThreshold(M_PI_2);   //������ ���ڹ��캯���Ѿ���������˳�ʼ�� Ϊ��/2 ���������� ʹ�� M_PI/2  M_PI_2 //�߽����ʱ�ĽǶ���ֵ
	est.setSearchMethod(tree);
	est.setKSearch(20);  //һ���������ֵԽ�ߣ����ձ߽�ʶ��ľ���Խ�� 20
									//  est.setRadiusSearch(everagedistance);  //�����뾶
	est.compute(boundaries);

	//  pcl::PointCloud<pcl::PointXYZ> boundPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> noBoundPoints;
	int countBoundaries = 0;
	for (int i = 0; i < cloud->size(); i++) {
		uint8_t x = (boundaries.points[i].boundary_point);
		int a = static_cast<int>(x); //�ú����Ĺ�����ǿ������ת��
		if (a == 1)
		{
			//  boundPoints.push_back(cloud->points[i]);
			(*boundPoints).push_back(cloud->points[i]);
			countBoundaries++;
		}
		else
			noBoundPoints.push_back(cloud->points[i]);

	}
	std::cout << "boudary size is��" << countBoundaries << std::endl;
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

/************************************������Ʒֱ���(ƽ������)**********************************************/
//ԭ�����ӣ�https ://blog.csdn.net/qq_39707351/article/details/93470524
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
