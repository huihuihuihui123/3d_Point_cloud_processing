#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>    //�������ͼ���ͷ�ļ�
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>   //���ͼ���ӻ���ͷ�ļ�
#include <pcl/visualization/pcl_visualizer.h>      //PCL���ӻ���ͷ�ļ�
#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointType;
//����
float angular_resolution_x = 0.5f,//angular_resolutionΪģ�����ȴ������ĽǶȷֱ��ʣ������ͼ����һ�����ض�Ӧ�ĽǶȴ�С
angular_resolution_y = angular_resolution_x;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;//���ͼ����ѭ����ϵͳ
bool live_update = false;
//���������ʾ
void
printUsage(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-rx <float>  angular resolution in degrees (default " << angular_resolution_x << ")\n"
		<< "-ry <float>  angular resolution in degrees (default " << angular_resolution_y << ")\n"
		<< "-c <int>     coordinate frame (default " << (int)coordinate_frame << ")\n"
		<< "-l           live update - update the range image according to the selected view in the 3D viewer.\n"
		<< "-h           this help\n"
		<< "\n\n";
}

void
setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);//Vector3f(0, 0, 0��1)--ԭ�����ꣻpos_vector--������ꣻT*��0��0��0��
	Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;//R*(0,0,1)+t
	Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
	viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
		look_at_vector[0], look_at_vector[1], look_at_vector[2],
		up_vector[0], up_vector[1], up_vector[2]);
}

//������
int
main(int argc, char** argv)
{
	//�����������
	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return 0;
	}
	if (pcl::console::find_argument(argc, argv, "-l") >= 0)
	{
		live_update = true;
		std::cout << "Live update is on.\n";
	}
	if (pcl::console::parse(argc, argv, "-rx", angular_resolution_x) >= 0)
		std::cout << "Setting angular resolution in x-direction to " << angular_resolution_x << "deg.\n";
	if (pcl::console::parse(argc, argv, "-ry", angular_resolution_y) >= 0)
		std::cout << "Setting angular resolution in y-direction to " << angular_resolution_y << "deg.\n";
	int tmp_coordinate_frame;
	if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
	{
		coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
		std::cout << "Using coordinate frame " << (int)coordinate_frame << ".\n";
	}
	angular_resolution_x = pcl::deg2rad(angular_resolution_x);
	angular_resolution_y = pcl::deg2rad(angular_resolution_y);

	//��ȡ����PCD�ļ�  ���û������PCD�ļ�������һ������
	pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());   //������������λ����һ��4*4�ķ���任
	/*std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");*/
	std::string filename = "D:/FinalPCD.pcd";
	if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
	{
		std::cout << "Was not able to open file \"" << filename << "\".\n";
		printUsage(argv[0]);
		return 0;
	}
	//����������λ�˸�ֵ  ���ǻ�ȡ���ƵĴ������ĵ�ƽ��*��ת������
	scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],//vector4f �������ɼ����ƣ�ԭ��/ƽ�ƣ�
		point_cloud.sensor_origin_[1],
		point_cloud.sensor_origin_[2])) *
		Eigen::Affine3f(point_cloud.sensor_orientation_);//��ת��Ԫ��

	// -----�Ӵ����ĵ����л�ȡ���ͼ--//
	//���û�������
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	/*
	 ����range_image.createFromPointCloud���������Ľ��� ���漰�ĽǶȶ�Ϊ����Ϊ��λ�� ��
	   point_cloudΪ�������ͼ������Ҫ�ĵ���
	  angular_resolution_x��ȴ�����X����ĽǶȷֱ���
	  angular_resolution_y��ȴ�����Y����ĽǶȷֱ���
	   pcl::deg2rad (360.0f)��ȴ�������ˮƽ�������Ƕ�
	   pcl::deg2rad (180.0f)��ֱ�������Ƕ�
	   scene_sensor_pose���õ�ģ�⴫������λ����һ������任����Ĭ��Ϊ4*4�ĵ�λ����任
	   coordinate_frame���尴����������ϵͳ��ϰ��  Ĭ��ΪCAMERA_FRAME
	   noise_level  ��ȡ���ͼ�����ʱ���ڽ���Բ�ѯ�����ֵ��Ӱ��ˮƽ
	   min_range ������С�Ļ�ȡ���룬С����С�Ļ�ȡ�����λ��Ϊ��������ä��
	   border_size  ���û�ȡ���ͼ���Ե�Ŀ�� Ĭ��Ϊ0
	*/
	range_image.createFromPointCloud(point_cloud, angular_resolution_x, angular_resolution_y, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

	//���ӻ�����
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
	viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
	//viewer.addCoordinateSystem (1.0f, "global");
	//PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
	//viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
	viewer.initCameraParameters();
	//range_image.getTransformationToWorldSystem ()�������ǻ�ȡ�����ͼ������ϵͳ��Ӧ�þ��Ǵ����������꣩ת��Ϊ��������ϵͳ��ת������

	const Eigen::Affine3f TransformationToWorldSystem = range_image.getTransformationToWorldSystem();
	cout << TransformationToWorldSystem.data() << endl;
	setViewerPose(viewer, range_image.getTransformationToWorldSystem());  //�����ӵ��λ��
	
	//���ӻ����ͼ
	pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(range_image);

	while (!viewer.wasStopped())
	{
		range_image_widget.spinOnce();
		viewer.spinOnce();
		pcl_sleep(0.01);

		if (1)
		{
			//���ѡ����ǡ���l�Ĳ���˵������Ҫ�����Լ�ѡ����ӵ����������ͼ��
		   // live update - update the range image according to the selected view in the 3D viewer.
			scene_sensor_pose = viewer.getViewerPose();
			range_image.createFromPointCloud(point_cloud, angular_resolution_x, angular_resolution_y,
				pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
				scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
			range_image_widget.showRangeImage(range_image);
		}
	}
}