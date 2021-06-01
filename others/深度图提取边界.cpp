/* \author Bastian Steder */
#include <iostream>
#include <boost/thread/thread.hpp>//�߳�ͷ�ļ�
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>//���������в���ͷ�ļ�
#include <pcl/visualization/common/float_image_utils.h>//�������ͼ���ͷ�ļ�
#include <pcl/io/png_io.h>//������ͼƬ��ͷ�ļ�
#include <pcl/filters/statistical_outlier_removal.h>//statiacl�˲�
typedef pcl::PointXYZ PointType;
// --------------------
// -----����-----
// --------------------
float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;//���ò��ɼ���Ϊ�����룿

std::string filename = "./turn_shuibei360 - Cloud.pcd";
int
main(int argc, char** argv)
{
	
	// ------------------------------------------------------------------
	// -----��ȡpcd�ļ������û�и���pcd�ļ��򴴽�һ��ʾ������-----
	// ------------------------------------------------------------------
	pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
	pcl::io::loadPCDFile(filename, *point_cloud_ptr);
	pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;//�������Ϊpoint_cloud
	pcl::PointCloud<PointType>::Ptr point_filterd(new pcl::PointCloud<PointType>);//statical֮��ĵ��ƴ��������

	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());//������λ��


	//������ĵ���point_cloud����statical_filter�õ�point_filterd
	pcl::StatisticalOutlierRemoval<PointType> sor;
	sor.setInputCloud(point_cloud_ptr);
	sor.setMeanK(10);
	sor.setStddevMulThresh(1.0);
	sor.filter(*point_filterd);
	pcl::io::savePCDFileASCII("point_filterd.pcd", *point_filterd);//�����˲�֮��ĵ���
	cout << "point_filterd.pcd" << point_filterd->size() << endl;
	std::cerr << "point_filterd.pcd has been saved!" << std::endl;
	// -----------------------------------------------
	// -----�ӵ��ƴ������ͼ��-----
	// -----------------------------------------------
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	range_image.createFromPointCloud(*point_filterd, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);//�������ͼ��range_image


  //save depth picture
	float *ranges = range_image.getRangesArray();
	unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, range_image.width, range_image.height);
	pcl::io::saveRgbPNGFile("saveRangeImageRGB.png", rgb_image, range_image.width, range_image.height);
	std::cerr << "Picture Saved!" << std::endl;

	//������������������������������8��10����22:13
	range_image.integrateFarRanges(far_ranges);//�����ݲ��뵽���ͼ����
	if (setUnseenToMaxRange)
		range_image.setUnseenToMaxRange();
	// --------------------------------------------
	// -----����ά���������ӵ���-----
	// --------------------------------------------
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	//�˲�֮���Ҵ��б߽���ȡ��
	int v1(0);
	viewer.createViewPort(0.5, 0, 1, 1, v1);
	viewer.setBackgroundColor(1, 1, 1);
	//viewer.addCoordinateSystem ();
	pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler(point_filterd, 0, 0, 0);
	viewer.addPointCloud(point_filterd, point_cloud_color_handler, "original point cloud", v1);
	//PointCloudColorHandlerCustom<pcl::PointWithRange>   range_image_color_handler (range_image_ptr, 150, 150, 150);
	//viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
	//viewer.setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "range image");
  //������ԭʼ���Ƶ�
	int v2(0);
	viewer.createViewPort(0, 0, 0.5, 1, v2);
	viewer.setBackgroundColor(1, 200, 200);
	pcl::visualization::PointCloudColorHandlerCustom<PointType> color2(point_cloud_ptr, 0, 0, 0);
	viewer.addPointCloud(point_cloud_ptr, color2, "cloud_in", v2);
	// -------------------------
	// -----��ȡ�߽�-----
	// -------------------------
	pcl::RangeImageBorderExtractor border_extractor(&range_image);//����߽���ȡ�Զ����β�Ϊһ��RangeImage���͵�ָ��
	pcl::PointCloud<pcl::BorderDescription> border_descriptions;
	border_extractor.compute(border_descriptions);//���߽���Ϣ��ŵ�board_description


	// ----------------------------------
	// -----����ά���������ʾ����߽�㼯-----
	// ----------------------------------
	pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>), veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>), shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
	pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr, &veil_points = *veil_points_ptr, &shadow_points = *shadow_points_ptr;//����ָ�룬��ģ����PointCloud<pcl::PointCloud>��
	for (int y = 0; y < (int)range_image.height; ++y)
	{
		for (int x = 0; x < (int)range_image.width; ++x)//�������ͼ���е�ÿһ����
		{
			if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
				border_points.points.push_back(range_image.points[y*range_image.width + x]);
			if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
				veil_points.points.push_back(range_image.points[y*range_image.width + x]);
			if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
				shadow_points.points.push_back(range_image.points[y*range_image.width + x]);
		}
	}
	//���ӻ�����߽絽statical�˲�֮��ĵ���
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler(border_points_ptr, 0, 255, 0);
	viewer.addPointCloud<pcl::PointWithRange>(border_points_ptr, border_points_color_handler, "border points", v1);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler(veil_points_ptr, 255, 0, 0);
	viewer.addPointCloud<pcl::PointWithRange>(veil_points_ptr, veil_points_color_handler, "veil points", v1);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler(shadow_points_ptr, 0, 0, 255);
	viewer.addPointCloud<pcl::PointWithRange>(shadow_points_ptr, shadow_points_color_handler, "shadow points", v1);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");


	//-------------------------------------
	// -----�����ͼ������ʾ�㼯-----
	// ------------------------------------
	pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
	range_image_borders_widget =
		pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(range_image, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), false, border_descriptions, "Range image with borders");
	// -------------------------------------
	//--------------------
	// -----��ѭ��-----
	//--------------------
	while (!viewer.wasStopped())
	{
		range_image_borders_widget->spinOnce();
		viewer.spinOnce();
		pcl_sleep(0.01);
	}

	return 0;
}
