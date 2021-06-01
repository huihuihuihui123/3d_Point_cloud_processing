/* \author Bastian Steder */
#include <iostream>
#include <boost/thread/thread.hpp>//线程头文件
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>//解析命令行参数头文件
#include <pcl/visualization/common/float_image_utils.h>//保存深度图相关头文件
#include <pcl/io/png_io.h>//保存至图片的头文件
#include <pcl/filters/statistical_outlier_removal.h>//statiacl滤波
typedef pcl::PointXYZ PointType;
// --------------------
// -----参数-----
// --------------------
float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;//设置不可见点为最大距离？

std::string filename = "./turn_shuibei360 - Cloud.pcd";
int
main(int argc, char** argv)
{
	
	// ------------------------------------------------------------------
	// -----读取pcd文件，如果没有给出pcd文件则创建一个示例点云-----
	// ------------------------------------------------------------------
	pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
	pcl::io::loadPCDFile(filename, *point_cloud_ptr);
	pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;//输入点云为point_cloud
	pcl::PointCloud<PointType>::Ptr point_filterd(new pcl::PointCloud<PointType>);//statical之后的点云存放在这里

	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());//传感器位姿


	//对输入的点云point_cloud进行statical_filter得到point_filterd
	pcl::StatisticalOutlierRemoval<PointType> sor;
	sor.setInputCloud(point_cloud_ptr);
	sor.setMeanK(10);
	sor.setStddevMulThresh(1.0);
	sor.filter(*point_filterd);
	pcl::io::savePCDFileASCII("point_filterd.pcd", *point_filterd);//保存滤波之后的点云
	cout << "point_filterd.pcd" << point_filterd->size() << endl;
	std::cerr << "point_filterd.pcd has been saved!" << std::endl;
	// -----------------------------------------------
	// -----从点云创建深度图像-----
	// -----------------------------------------------
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	range_image.createFromPointCloud(*point_filterd, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);//生成深度图像range_image


  //save depth picture
	float *ranges = range_image.getRangesArray();
	unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, range_image.width, range_image.height);
	pcl::io::saveRgbPNGFile("saveRangeImageRGB.png", rgb_image, range_image.width, range_image.height);
	std::cerr << "Picture Saved!" << std::endl;

	//、、、、、、、、、、、、、、、8月10号晚22:13
	range_image.integrateFarRanges(far_ranges);//将数据并入到深度图像中
	if (setUnseenToMaxRange)
		range_image.setUnseenToMaxRange();
	// --------------------------------------------
	// -----打开三维浏览器并添加点云-----
	// --------------------------------------------
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	//滤波之后且带有边界提取的
	int v1(0);
	viewer.createViewPort(0.5, 0, 1, 1, v1);
	viewer.setBackgroundColor(1, 1, 1);
	//viewer.addCoordinateSystem ();
	pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler(point_filterd, 0, 0, 0);
	viewer.addPointCloud(point_filterd, point_cloud_color_handler, "original point cloud", v1);
	//PointCloudColorHandlerCustom<pcl::PointWithRange>   range_image_color_handler (range_image_ptr, 150, 150, 150);
	//viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
	//viewer.setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "range image");
  //纯输入原始点云的
	int v2(0);
	viewer.createViewPort(0, 0, 0.5, 1, v2);
	viewer.setBackgroundColor(1, 200, 200);
	pcl::visualization::PointCloudColorHandlerCustom<PointType> color2(point_cloud_ptr, 0, 0, 0);
	viewer.addPointCloud(point_cloud_ptr, color2, "cloud_in", v2);
	// -------------------------
	// -----提取边界-----
	// -------------------------
	pcl::RangeImageBorderExtractor border_extractor(&range_image);//定义边界提取对对象，形参为一个RangeImage类型的指针
	pcl::PointCloud<pcl::BorderDescription> border_descriptions;
	border_extractor.compute(border_descriptions);//将边界信息存放到board_description


	// ----------------------------------
	// -----在三维浏览器中显示三组边界点集-----
	// ----------------------------------
	pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>), veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>), shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
	pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr, &veil_points = *veil_points_ptr, &shadow_points = *shadow_points_ptr;//创建指针，类模板是PointCloud<pcl::PointCloud>类
	for (int y = 0; y < (int)range_image.height; ++y)
	{
		for (int x = 0; x < (int)range_image.width; ++x)//遍历深度图像中的每一个点
		{
			if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
				border_points.points.push_back(range_image.points[y*range_image.width + x]);
			if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
				veil_points.points.push_back(range_image.points[y*range_image.width + x]);
			if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
				shadow_points.points.push_back(range_image.points[y*range_image.width + x]);
		}
	}
	//可视化三组边界到statical滤波之后的点云
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
	// -----在深度图像中显示点集-----
	// ------------------------------------
	pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
	range_image_borders_widget =
		pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(range_image, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), false, border_descriptions, "Range image with borders");
	// -------------------------------------
	//--------------------
	// -----主循环-----
	//--------------------
	while (!viewer.wasStopped())
	{
		range_image_borders_widget->spinOnce();
		viewer.spinOnce();
		pcl_sleep(0.01);
	}

	return 0;
}
