#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>             //ģ��ϵ��ͷ�ļ�
#include <pcl/filters/project_inliers.h>          //ͶӰ�˲���ͷ�ļ�
#include <pcl/visualization/pcl_visualizer.h>//���ƿ��ӻ�
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

	//�������Ʋ���ӡ����
	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	std::cerr << "Cloud before projection: " << std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << std::endl;

	// ���ModelCoefficients��ֵ,ʹ��ax+by+cz+d=0ƽ��ģ�ͣ����� a=b=d=0,c=1 Ҳ����X����Yƽ��
	//����ģ��ϵ�����󣬲�����Ӧ������
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	// ����ProjectInliers����ʹ��ModelCoefficients��ΪͶӰ�����ģ�Ͳ���
	pcl::ProjectInliers<pcl::PointXYZ> proj;     //����ͶӰ�˲�����
	proj.setModelType(pcl::SACMODEL_PLANE);      //���ö����Ӧ��ͶӰģ��
	proj.setInputCloud(cloud);                   //�����������
	proj.setModelCoefficients(coefficients);       //����ģ�Ͷ�Ӧ��ϵ��
	proj.filter(*cloud_projected);                 //ͶӰ����洢

	std::cerr << "Cloud after projection: " << std::endl;
	for (size_t i = 0; i < cloud_projected->points.size(); ++i)
		std::cerr << "    " << cloud_projected->points[i].x << " "
		<< cloud_projected->points[i].y << " "
		<< cloud_projected->points[i].z << std::endl;



	pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

	// Define R,G,B colors for the point cloud
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud, 255, 255, 255);
	// We add the point cloud to the viewer and pass the color handler
	viewer.addPointCloud(cloud, source_cloud_color_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(cloud_projected, 230, 20, 20); // Red
	viewer.addPointCloud(cloud_projected, transformed_cloud_color_handler, "transformed_cloud");

	viewer.addCoordinateSystem(1000, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "transformed_cloud");

	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}
	return (0);
}