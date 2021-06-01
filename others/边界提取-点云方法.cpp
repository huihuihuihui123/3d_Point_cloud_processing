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

////���Ʊ�Ե��ȡ
//normEST.setRadiusSearch(reforn)������Ϊ�ֱ��ʵ�10��ʱ��Ч���Ϻã���Ҫ�Ƕ��ڷ��߹��ơ�
//	����뾶ѡ��̫С�ˣ������ϴ󣬹��Ƶķ��߾����׳�������������뾶���õ�̫������ٶȾͱȽ�����
//boundEst.setRadiusSearch(re)��Ҳ����10����̫С���ڲ��ĺܶ��Ͷ����ɱ߽���ˡ�
//  ���һ�������Ǳ߽��ж�ʱ�ĽǶ���ֵ��Ĭ��ֵΪPI / 2���˴�����ΪPI / 4���û�Ҳ���Ը�����Ҫ���и��ġ�

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

	pcl::VoxelGrid<pcl::PointXYZ> sor;  //�����˲�����
	sor.setInputCloud(cloud);            //������Ҫ���˵ĵ��Ƹ��˲�����
	sor.setLeafSize(0.1, 0.1, 0.1);  //�����˲�ʱ�������������Ϊ1cm��������
	sor.filter(*cloud_VoxelGrid);           //ִ���˲������洢���
	std::cerr << "PointCloud after filtering: " << cloud_VoxelGrid->width * cloud_VoxelGrid->height
		<< " data points (" << pcl::getFieldsList(*cloud_VoxelGrid) << ").";


	//�������ƽ����� �����õ��Ƽ�ࣨ���Ʒֱ���a����������10*a ���Ʒ���
	//����һ������k���ڲ�ѯ,��ѯ��ʱ�����õ��ڵ����У����һ�����ڵ����䱾��
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_VoxelGrid);
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
		int nres = kdtree.nearestKSearch(cloud->points[i], k, nnh, squaredistance);
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
	cout << "average distance is : " << everagedistance << endl;

	/******************************���Ʒ���*******************************************/
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;  //����pcl::PointXYZ��ʾ�����������ݣ�pcl::Normal��ʾ�������,��pcl::Normalǰ�����Ƿ������һ��������
	normEst.setInputCloud(cloud_VoxelGrid);
	normEst.setSearchMethod(tree);
	//normEst.setRadiusSearch(10*everagedistance);  //������Ƶİ뾶
	normEst.setKSearch(9);  //������Ƶĵ���
	normEst.compute(*normals);
	cout << "normal size is " << normals->size() << endl;
	//normal_est.setViewPoint(0,0,0); //���Ӧ�û�ʹ����һ��

	/**********************************���Ʊ߽�****************************************************/
	pcl::PointCloud<pcl::Boundary> boundaries;  //����߽���ƽ��
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;//����һ�����б߽��������ƵĶ���
	est.setInputCloud(cloud_VoxelGrid);
	est.setInputNormals(normals);/*M_PI_2 */
	est.setAngleThreshold(M_PI_2);   //������ ���ڹ��캯���Ѿ���������˳�ʼ�� Ϊ��/2 ���������� ʹ�� M_PI/2  M_PI_2 //�߽����ʱ�ĽǶ���ֵ
	est.setSearchMethod(tree);
	est.setKSearch(20);  //һ���������ֵԽ�ߣ����ձ߽�ʶ��ľ���Խ�� 20
	//est.setRadiusSearch(10 * everagedistance);  //һ����Ϊ10���ķֱ���
									//  est.setRadiusSearch(everagedistance);  //�����뾶
	est.compute(boundaries);

	//  pcl::PointCloud<pcl::PointXYZ> boundPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> noBoundPoints;
	int countBoundaries = 0;
	for (int i = 0; i < cloud_VoxelGrid->size(); i++) {
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

/************************************������Ʒֱ���(ƽ������)**********************************************/
//ԭ�����ӣ�https ://blog.csdn.net/qq_39707351/article/details/93470524

	//�������ƽ����� �����õ��Ƽ�ࣨ���Ʒֱ���a����������10*a ���Ʒ���
	//����һ������k���ڲ�ѯ,��ѯ��ʱ�����õ��ڵ����У����һ�����ڵ����䱾��
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