#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/print.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/console/time.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>
#include <pcl/surface/impl/organized_fast_mesh.hpp> 
#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>

#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
using namespace pcl::console;
int main (int argc, char** argv) {


	
	std::string filename ="turn_shuibei360 - Cloud.pcd";

	int width=640,height=480,size=2,type=0;
	float fx=525,fy=525,cx=320,cy=240;

	
	//convert unorignized point cloud to orginized point cloud begin
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile (filename, *cloud);
	print_info ("Read pcd file successfully\n");
	Eigen::Affine3f sensorPose;
	sensorPose.setIdentity(); 
	//1 0 0 0
	//0 1 0 0
	//0 0 1 0
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel=0.00;
	float minRange = 0.0f;

	pcl::RangeImagePlanar::Ptr rangeImage(new pcl::RangeImagePlanar);
	//设置深度图像的宽度/高度 /光心坐标/焦距等，将从一个视点获取的点云转换为深度图。
	//从已存在的点云中创建图像，其中， point_cloud 为指向创建深度图像所需点云的对象引用，
	//di_width 是视差图像的宽度， di_height 是视差图像的高度，
	//di_center_ x是照相机投影中心的 x 坐标， di _ center_y 是照相机投影中心的 y 坐标 ，
	//di_focal_length_x 是照相机水平方向上的焦距， di_ fo cal_ length_ y 是照相机垂直方向上的焦距，
	//sensor_pose 为模拟深度照相机的位姿， coordinate_frame 为点云所使用的坐标系统， 
	//noise_level 为传感器的噪声水平，用于 z缓冲区求取深度时，如果噪声越大，查询点周围的点深度就会影响查询点的深度，
	//如果无噪声，则直接取查询点 z 缓冲区中最小的距离为深度，
	//min_range 为可见点的最小深度，小于该距离的点视为盲区，即不可见点。
	
	rangeImage->createFromPointCloudWithFixedSize(*cloud,width,height,cx,cy,fx,fy,sensorPose,coordinate_frame);
	std::cout << rangeImage << "\n";
	//convert unorignized point cloud to orginized point cloud end


	//viusalization of range image
	pcl::visualization::RangeImageVisualizer range_image_widget ("点云库PCL从入门到精通");
	range_image_widget.showRangeImage (*rangeImage);
	range_image_widget.setWindowTitle("点云库PCL从入门到精通");
	/****************************************从深度图进行三角剖分*********************************************************/
	//triangulation based on range image
	pcl::OrganizedFastMesh<pcl::PointWithRange>::Ptr tri(new pcl::OrganizedFastMesh<pcl::PointWithRange>);
	//range包含从所获得的视点到采样点的距离测量值 x y z range
	pcl::search::KdTree<pcl::PointWithRange>::Ptr tree (new pcl::search::KdTree<pcl::PointWithRange>);
	tree->setInputCloud(rangeImage);
	pcl::PolygonMesh triangles;
	tri->setTrianglePixelSize(size);
	tri->setInputCloud(rangeImage);
	tri->setSearchMethod(tree);
	tri->setTriangulationType((pcl::OrganizedFastMesh<pcl::PointWithRange>::TriangulationType)type);
	tri->reconstruct(triangles);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("点云库PCL从入门到精通"));
	viewer->setBackgroundColor(0.5,0.5,0.5);
	viewer->addPolygonMesh(triangles,"tin");
	viewer->addCoordinateSystem();
	while (!range_image_widget.wasStopped ()&&!viewer->wasStopped())
	{
		range_image_widget.spinOnce ();

		pcl_sleep (0.01);
		viewer->spinOnce ();

	}
}