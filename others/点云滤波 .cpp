#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>//���ƿ��ӻ�
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
int v1(0), v2(0);//�ӿڱ�����������������ӿ�
//���ƿ�ѡ�¼�����
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
	reader.read("ˮ��1.pcd", *cloud); // Remember to download the file first!

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ")." << endl;
#if 0
	/******************************************************************************
	  ����һ��Ҷ��СΪ1cm��pcl::VoxelGrid�˲�����
	**********************************************************************************/
	pcl::VoxelGrid<pcl::PointXYZ> sor;  //�����˲�����
	sor.setInputCloud(cloud);            //������Ҫ���˵ĵ��Ƹ��˲�����
	sor.setLeafSize(0.01f, 0.01f, 0.01f);  //�����˲�ʱ�������������Ϊ1cm��������
	sor.filter(*cloud_filtered);           //ִ���˲������洢���


	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

#endif

#if 1
	/*******************************������RadiusOutlierRemovalȥ��������******************************************/
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(0.01);
	outrem.setMinNeighborsInRadius(50);//30
	outrem.filter(*cloud_Radius_filtered);
	//pcl::io::savePCDFileASCII("ˮ��after_Radius_filter.pcd", *cloud_Radius_filtered);
	std::cout << "Pointcloud after filtering: " << cloud_Radius_filtered->width*cloud_Radius_filtered->height << "data points" << endl;
#endif
#if 0
	/*******************************������RadiusOutlierRemovalȥ��������******************************************/
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
	pcl::io::savePCDFileASCII("ˮ��after_filter.pcd", *cloud_Statictics_filtered);
#endif
#if 1
	reader.read("ˮ��after_Radius_static_filter.pcd", *cloud_Statictics_filtered);
	std::cout << "Pointcloud after filtering: " << cloud_Statictics_filtered->width*cloud_Statictics_filtered->height << "data points" << endl;
#endif 
#if 0
	/**********************mls*****************************************************/
	//˵���������Surface���棬����ͨ���������о���ʹ�ã��ҷ��ִ��ಢ���������Ϻ�ı��棬��������Mesh����Triangulations��
	//	ֻ�ǽ����ƽ�����MLS��ӳ�䣬ʹ������ĵ��Ƹ���ƽ����
	//��ˣ����ҿ�������Ӧ�÷���Filter�¡�ͨ����ε�ʵ�������ݵĴ����ҷ��ִ�����Ҫ�����ڵ��ƵĹ�˳����
	//	��Ȼ����ĵ���������˹���Ⱥ��֮��ĵ㼯�����򽫻�����������Ͼ��ȵĴ��������������ơ�
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_mls(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	mls.setInputCloud(cloud_Statictics_filtered);
	//������������
	mls.setSearchMethod(tree_mls);
	mls.setSearchRadius(0.03);//���ֵԽ������ĵ�Խ��

	mls.process(*mls_points);
	pcl::io::savePCDFileASCII("ˮ��mls_points.pcd", *mls_points);
	reader.read("ˮ��mls_points.pcd", *mls_cloud);
	cout <<"mls: " <<mls_cloud->size() << endl;
#endif
#if 0
	/*******************************�����������������������***************************************/
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
	poisson.setConfidence(false);//�Ƿ�ʹ�÷������Ĵ�С��Ϊ������Ϣ�����false�����з���������һ����
	poisson.setDegree(2); //���ò���degree[1, 5], ֵԽ��Խ��ϸ����ʱԽ��
	poisson.setDepth(8); //���������ȣ����2^d x 2 ^ d x 2 ^ d������Ԫ�����ڰ˲�������Ӧ�����ܶȣ�ָ��ֵ��Ϊ������
	poisson.setIsoDivide(8);//������ȡISO��ֵ����㷨�����
	poisson.setManifold(true); //�Ƿ���Ӷ���ε����ģ�����������ǻ�ʱ�� �������б�־���������Ϊtrue����Զ���ν���ϸ�����ǻ�ʱ������ģ�����false�����
	poisson.setOutputPolygons(true);//�Ƿ������������񣨶��������ǻ��ƶ�������Ľ����
	poisson.setSamplesPerNode(3.0); // ��������һ���˲�������е����������С��������������[1.0 - 5.0], ������[15. - 20.]ƽ��
	poisson.setScale(1.25);//���������ع���������ֱ���������߽�������ֱ���ı��ʡ�
	poisson.setSolverDivide(8);//����������Է������Gauss-Seidel�������������
	poisson.setSearchMethod(tree2);
	poisson.setInputCloud(cloud_with_normals);
	pcl::PolygonMesh mesh;//����������������ڴ洢���
	poisson.performReconstruction(mesh);//ִ���ع�
#endif 
	/**********************poisson2 reconstruction***********************/
	//pcl::Poisson<pcl::PointNormal> poissonMLS;
	//poissonMLS.setConfidence(false);//�Ƿ�ʹ�÷������Ĵ�С��Ϊ������Ϣ�����false�����з���������һ����
	//poissonMLS.setDegree(2); //���ò���degree[1, 5], ֵԽ��Խ��ϸ����ʱԽ��
	//poissonMLS.setDepth(8); //���������ȣ����2^d x 2 ^ d x 2 ^ d������Ԫ�����ڰ˲�������Ӧ�����ܶȣ�ָ��ֵ��Ϊ������
	//poissonMLS.setIsoDivide(8);//������ȡISO��ֵ����㷨�����
	//poissonMLS.setManifold(true); //�Ƿ���Ӷ���ε����ģ�����������ǻ�ʱ�� �������б�־���������Ϊtrue����Զ���ν���ϸ�����ǻ�ʱ������ģ�����false�����
	//poissonMLS.setOutputPolygons(true);//�Ƿ������������񣨶��������ǻ��ƶ�������Ľ����
	//poissonMLS.setSamplesPerNode(3.0); // ��������һ���˲�������е����������С��������������[1.0 - 5.0], ������[15. - 20.]ƽ��
	//poissonMLS.setScale(1.25);//���������ع���������ֱ���������߽�������ֱ���ı��ʡ�
	//poissonMLS.setSolverDivide(8);//����������Է������Gauss-Seidel�������������
	//poissonMLS.setSearchMethod(tree2);
	//poissonMLS.setInputCloud(mls_points);
	//pcl::PolygonMesh meshMLS;//����������������ڴ洢���
	//poisson.performReconstruction(meshMLS);//ִ���ع�
	/**************************************************���Ӵ���+��Ƭ����******************************************************/
	boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("project points using  aparameyric model"));
	viewer->setBackgroundColor(0, 0, 0); //�������� 
	int v1;
	viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
	//���õ�����ɫ 
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud, 34, 25, 133); //ͶӰǰ���������һ����ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_Statictics_filtered, 255, 255, 255);  //ͶӰ�������Ϊ��ɫ
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, source_color, "source", v1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_Statictics_filtered, target_color, "projected", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "projected");
	viewer->spin();
	/**************************************************˫�Ӵ���*************************************************************/

	//viewer->initCameraParameters();
	//
	////5.1ԭʼ���ƴ���
	//viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	//viewer->setBackgroundColor(0, 0, 0, v1);
	//viewer->addText("original", 10, 10, "v1 text", v1);
	//viewer->addPointCloud<pcl::PointXYZ>(cloud_Statictics_filtered, "sample cloud1", v1);
	////viewer->addPolygonMesh(mesh, "meshMLS", v1);
	////viewer->setRepresentationToPointsForAllActors(); // �Ե����ʽ��ʾ
	//viewer->addCoordinateSystem(1.0);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud1");
	////5.2�˲�����

	//viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	//viewer->setBackgroundColor(0, 0, 0, v2);
	//viewer->addText("after filtered", 10, 10, "v2 text", v2);
	//viewer->addPolygonMesh(mesh, "mesh",v2);
	//// viewer->setRepresentationToPointsForAllActors(); // �Ե����ʽ��ʾ
	////viewer->setRepresentationToSurfaceForAllActors();  //�������ʽ��ʾ
	////viewer->setRepresentationToWireframeForAllActors(); // ���������ʽ��ʾ
	////viewer->addPointCloud<pcl::PointXYZ>(mls_cloud, "sample cloud2",v2);
	//viewer->addCoordinateSystem(1.0);
	//viewer->registerAreaPickingCallback(pp_callback, (void*)&cloud_Statictics_filtered);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud2");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);  //ˢ��
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}