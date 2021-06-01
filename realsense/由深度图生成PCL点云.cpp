// include the librealsense C++ header file
/*************************************************

Copyright:bupt

Author:�ν���

Date:2010-11-20

Description:����realsense������ʾ��ɼ�

0.��������ǰ����Ҫ�ֶ���Ϊ�޸�ȫ�ֱ�������

1.���г��򣬲鿴RGB�����ͼ����ESC�˳�

2.���¡��ո񡱡����س�������q�������� ���ͼ�����Ƶ�ת��

3.�ڵ�����ʾ���水�¡��ո���������ƽ������ڵ�ǰĿ¼��

4.�رյ�����ʾ�Ĵ��ں󣬻ص����ͼ��RGBͼ��ʾ��ѭ�����ٰ�ESC��������
**************************************************/

#include <librealsense2/rs.hpp>
#include <fstream>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp> 
using namespace std;
using namespace cv;

/***************************��Ҫ��Ϊ�޸ĵĲ���**********************************/
static float cx = 314.499;//����ڲ�
static float cy = 231.307;
static float fx = 619.554;
static float fy = 619.28;
static float backgroud = 3;//����3m��ȵĵ��ƽ������˵���backgroud*lengthscale ������ĵ��Ʊ��˳�
static float lengthscale = 1000;//���ȷֱ��� 1000-mm��1--m��

bool ispointCloudXYZ = false;// true--XYZ���ƣ�����ɫ��Ϣ��false--RGB���ƣ�����ɫ��Ϣ


//���ͼת����
pcl::PointCloud<pcl::PointXYZ>::Ptr depthToPcdXYZ(cv::Mat &depth, float depth_scale, float fx, float fy, float cx, float cy);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthToPcdXYZRGB(cv::Mat &RGB, cv::Mat &depth, float depth_scale, float fx, float fy, float cx, float cy);
float get_depth_scale(rs2::device dev);//��ȡ��ȶ�Ӧ�ĵ�λ��m
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
//�����Ʊ���Ϊtxt
void savetxt(const Mat &depth, const float depthscale);
//��ʾ����
void showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void showPointCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
//�������
void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *nothing);
bool isSavePcd = false;


int main()
{
	//Contruct a pipeline which abstracts the device
	rs2::pipeline pipe;
	rs2::colorizer color_map;
	//Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg;

	//Add desired streams to configuration
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	//cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

	//Instruct pipeline to start streaming with the requested configuration
	// pipe.start(cfg);
	//    const auto color_win = "color Image";
	//    namedWindow(color_win, WINDOW_AUTOSIZE);
	//    const auto depth_win = "depth Image";
	//    namedWindow(depth_win, WINDOW_AUTOSIZE);
	rs2::pipeline_profile profile = pipe.start(cfg);
	float depth_scale = get_depth_scale(profile.get_device());
	cout << "depth_scale" << depth_scale << endl;

	rs2_stream align_to = RS2_STREAM_COLOR;//���ͼ���뵽����

	rs2::align align(align_to);
	float depth_clipping_distance = 1.f;


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB;
	while (1)//realsense��ѭ����ѭ����ʾ���ͼ���ɫͼ
	{


		//Wait for all configured streams to produce a frame
		rs2::frameset frames = pipe.wait_for_frames();


		if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
		{
			//If the profile was changed, update the align object, and also get the new device's depth scale
			//���profile�����ı䣬�����align�������»�ȡ���ͼ�����ص����ȵ�λ��ת������
			profile = pipe.get_active_profile();
			align = rs2::align(align_to);
			depth_scale = get_depth_scale(profile.get_device());
		}

		//Get processed aligned frame
		//��ȡ������֡
		auto processed = align.process(frames);

		// Trying to get both other and aligned depth frames
		//���Ի�ȡ���������ͼ��֡������֡
		rs2::frame aligned_color_frame = processed.get_color_frame();//processed.first(align_to);
																	 //        rs2::frame aligned_depth_frame = processed.get_depth_frame().apply_filter(color_map);;
																	 //        rs2::frame depth_frame=frames.get_depth_frame().apply_filter(color_map);
		rs2::frame aligned_depth_frame = processed.get_depth_frame();//�Ѷ���

		rs2::frame depth_frame = frames.get_depth_frame().apply_filter(color_map);//δ����depth


		// Creating OpenCV Matrix from a color image
		Mat color(Size(640, 480), CV_8UC3, (void*)aligned_color_frame.get_data(), Mat::AUTO_STEP);
		/*Mat ir(Size(640, 480), CV_8UC1, (void*)ir_frame.get_data(), Mat::AUTO_STEP);
				Mat aligned_depth(Size(640, 480), CV_8UC3, (void*)aligned_depth_frame.get_data(), Mat::AUTO_STEP);
				Mat depth(Size(640, 480), CV_8UC3, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
*/
		Mat aligned_depth(Size(640, 480), CV_16UC1, (void*)aligned_depth_frame.get_data(), Mat::AUTO_STEP);
		Mat aligned_depth_color(Size(640, 480), CV_8UC3, (void*)aligned_depth_frame.apply_filter(color_map).get_data(), Mat::AUTO_STEP);
		//Mat depth(Size(640, 480), CV_8UC3, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

		// Display in a GUI
		imshow("aligned_depth_color", aligned_depth_color);
		//imshow("aligned_depth", aligned_depth*15);
		imshow("color", color);

		char key = waitKey(1);
		if (key == 32 || key == 13 || key == 'q')//�ո񡢻س���q
		{	
			if (ispointCloudXYZ == true)
			{
				//���ͼתXYZ
				cloud = depthToPcdXYZ(aligned_depth, depth_scale, fx, fy, cx, cy);
				showPointCloud(cloud);
			}
			else
			{
				//���ͼתRGB
				cloudRGB = depthToPcdXYZRGB(color, aligned_depth, depth_scale, fx, fy, cx, cy);
				showPointCloudRGB(cloudRGB);
			}
		}
		else if (key == 27)
		{
			break;
		}
	}
	return 0;
}

//��ȡ���ͼ�ֱ��ʣ����(m)=depth(i,j)*get_depth_scale;
float get_depth_scale(rs2::device dev)
{
	// Go over the device's sensors
	for (rs2::sensor& sensor : dev.query_sensors())
	{
		// Check if the sensor if a depth sensor
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
		{
			return dpt.get_depth_scale();
		}
	}
	throw std::runtime_error("Device does not have a depth sensor");
}

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
	for (auto&& sp : prev)
	{
		//If previous profile is in current (maybe just added another)
		auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
		if (itr == std::end(current)) //If it previous stream wasn't found in current
		{
			return true;
		}
	}
	return false;
}


//���ͼתXYZ���ƣ�����ɫ��Ϣ
pcl::PointCloud<pcl::PointXYZ>::Ptr depthToPcdXYZ(cv::Mat &depth,float depth_scale,float fx,float fy,float cx,float cy)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < depth.rows; i++)
	{
		pcl::PointXYZ pts;
		for (int j = 0; j < depth.cols; j++)
		{
			ushort d = depth.at<ushort>(i, j);
			float dist = d * depth_scale * lengthscale;//mm
			if (dist <= 0|| dist >backgroud*lengthscale) continue;
			pts.x = (j - cx)*dist / fx;
			pts.y = (i - cy)*dist / fy;
			pts.z = dist;
			cloud->push_back(pts);
		}
	}
	cloud->height = 1;
	cloud->width = cloud->points.size();
	return cloud;
}

//���ͼתRGBXYZ����
pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthToPcdXYZRGB(cv::Mat &RGB, cv::Mat &depth, float depth_scale, float fx, float fy, float cx, float cy)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < depth.rows; i++)
	{
		pcl::PointXYZRGB pts;
		for (int j = 0; j < depth.cols; j++)
		{
			ushort d = depth.at<ushort>(i, j);
			float dist = d * depth_scale * lengthscale;//mm
			if (d <= 0 || d > backgroud*lengthscale) continue;
			pts.x = (j - cx)*dist / fx;
			pts.y = (i - cy)*dist / fy;
			pts.z = dist;
			pts.r = RGB.at<cv::Vec3b>(i, j)[2];
			pts.g = RGB.at<cv::Vec3b>(i, j)[1];
			pts.b = RGB.at<cv::Vec3b>(i, j)[0];
			cloud->push_back(pts);
		}
	}
	cloud->height = 1;
	cloud->width = cloud->points.size();
	return cloud;
}

//���ӻ�XYZ����
void showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	/*******���ӻ��������ƣ�Ӧ��PCLVisualizer���ӻ�����ʾ��������XYZ��Ϣ�ĵ���***********/
		// �����洢�������ĵĶ���
	Eigen::Vector4f centroid;

	pcl::compute3DCentroid(*cloud, centroid);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("Pointcloud"));
	view->setBackgroundColor(0,0,0);
	view->addCoordinateSystem(1.0);
	view->setCameraPosition(centroid[0], centroid[1], centroid[2], 1.0, 1.0, 1.0);
	view->addPointCloud(cloud, "cloud");
	
	view->registerKeyboardCallback(&keyboardEvent, (void*)NULL);
	while (!view->wasStopped())
	{
		view->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		if (isSavePcd == true)
		{
			pcl::io::savePCDFileBinary("PointcloudXYZ.pcd", *cloud);
			isSavePcd = false;
		}
	}
}


//���ӻ�XYZRGB����
void showPointCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	/*******���ӻ��������ƣ�Ӧ��PCLVisualizer���ӻ�����ʾ��������XYZ��Ϣ�ĵ���***********/
		// �����洢�������ĵĶ���
	Eigen::Vector4f centroid;

	pcl::compute3DCentroid(*cloud, centroid);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("Pointcloud"));
	view->setBackgroundColor(0, 0, 0);
	view->addCoordinateSystem(1.0);
	view->setCameraPosition(centroid[0], centroid[1], centroid[2], 1.0, 1.0, 1.0);
	view->addPointCloud(cloud, "cloud");

	view->registerKeyboardCallback(&keyboardEvent, (void*)NULL);
	while (!view->wasStopped())
	{
		if (isSavePcd == true)
		{
			pcl::io::savePCDFileBinary("PointcloudXYZRGB.pcd", *cloud);
			isSavePcd = false;
		}
		view->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		
	}
}


void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *nothing)
{
	if (event.getKeySym() == "space"&&event.keyDown())
	{
		isSavePcd = true;
	}
}



