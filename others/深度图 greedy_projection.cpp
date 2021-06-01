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
	//�������ͼ��Ŀ��/�߶� /��������/����ȣ�����һ���ӵ��ȡ�ĵ���ת��Ϊ���ͼ��
	//���Ѵ��ڵĵ����д���ͼ�����У� point_cloud Ϊָ�򴴽����ͼ��������ƵĶ������ã�
	//di_width ���Ӳ�ͼ��Ŀ�ȣ� di_height ���Ӳ�ͼ��ĸ߶ȣ�
	//di_center_ x�������ͶӰ���ĵ� x ���꣬ di _ center_y �������ͶӰ���ĵ� y ���� ��
	//di_focal_length_x �������ˮƽ�����ϵĽ��࣬ di_ fo cal_ length_ y ���������ֱ�����ϵĽ��࣬
	//sensor_pose Ϊģ������������λ�ˣ� coordinate_frame Ϊ������ʹ�õ�����ϵͳ�� 
	//noise_level Ϊ������������ˮƽ������ z��������ȡ���ʱ���������Խ�󣬲�ѯ����Χ�ĵ���Ⱦͻ�Ӱ���ѯ�����ȣ�
	//�������������ֱ��ȡ��ѯ�� z ����������С�ľ���Ϊ��ȣ�
	//min_range Ϊ�ɼ������С��ȣ�С�ڸþ���ĵ���Ϊä���������ɼ��㡣
	
	rangeImage->createFromPointCloudWithFixedSize(*cloud,width,height,cx,cy,fx,fy,sensorPose,coordinate_frame);
	std::cout << rangeImage << "\n";
	//convert unorignized point cloud to orginized point cloud end


	//viusalization of range image
	pcl::visualization::RangeImageVisualizer range_image_widget ("���ƿ�PCL�����ŵ���ͨ");
	range_image_widget.showRangeImage (*rangeImage);
	range_image_widget.setWindowTitle("���ƿ�PCL�����ŵ���ͨ");
	/****************************************�����ͼ���������ʷ�*********************************************************/
	//triangulation based on range image
	pcl::OrganizedFastMesh<pcl::PointWithRange>::Ptr tri(new pcl::OrganizedFastMesh<pcl::PointWithRange>);
	//range����������õ��ӵ㵽������ľ������ֵ x y z range
	pcl::search::KdTree<pcl::PointWithRange>::Ptr tree (new pcl::search::KdTree<pcl::PointWithRange>);
	tree->setInputCloud(rangeImage);
	pcl::PolygonMesh triangles;
	tri->setTrianglePixelSize(size);
	tri->setInputCloud(rangeImage);
	tri->setSearchMethod(tree);
	tri->setTriangulationType((pcl::OrganizedFastMesh<pcl::PointWithRange>::TriangulationType)type);
	tri->reconstruct(triangles);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("���ƿ�PCL�����ŵ���ͨ"));
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