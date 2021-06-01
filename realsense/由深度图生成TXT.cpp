// include the librealsense C++ header file
/*************************************************

Copyright:bupt

Author:�ν���

Date:2010-11-20

Description:
����realsense���Ʋɼ� ����TXT---�ֶ� X\Y\Z\R\G\B  Or X\Y\Z
��������OpenCV��
���������¿ո�����ڱ�cppͬ��Ŀ¼�±���output.txt����


**************************************************/

#include <librealsense2/rs.hpp>
#include <fstream>
#include <opencv2/opencv.hpp> 
using namespace std;
using namespace cv;

/***************************��Ҫ��Ϊ�޸ĵĲ���**********************************/
static float cx = 314.499;//����ڲ�
static float cy = 231.307;
static float fx = 619.554;
static float fy = 619.28;
static float backgroud = 1;//����1m��ȵĵ��ƽ������˵���backgroud*lengthscale ������ĵ��Ʊ��˳�
static float lengthscale = 1000;//���ȷֱ��� 1000-mm��1--m��

float get_depth_scale(rs2::device dev);//��ȡ��ȶ�Ӧ�ĵ�λ��m
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
//�����Ʊ���Ϊtxt isRGBPointCloud==true---RGBXYZ;false---XYZ
void savetxt(const Mat &depth, const Mat &RGBImg, float fx, float fy, float cx, float cy,
	const float depthscale, const char* filename, bool isRGBPointCloud);


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
	rs2::pipeline_profile profile = pipe.start(cfg);
	float depth_scale = get_depth_scale(profile.get_device());//��ȡ��ȷֱ���
	cout << "depth_scale" << depth_scale << endl;

	rs2_stream align_to = RS2_STREAM_COLOR;

	rs2::align align(align_to);
	float depth_clipping_distance = 1.f;


	while (1)
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

		Mat aligned_depth(Size(640, 480), CV_16UC1, (void*)aligned_depth_frame.get_data(), Mat::AUTO_STEP);//��ʵ���ͼ(�����)
		Mat aligned_depth_color(Size(640, 480), CV_8UC3, (void*)aligned_depth_frame.apply_filter(color_map).get_data(), Mat::AUTO_STEP);//�����ͼ��ɫ(����ÿ������ͼ����ʾRGBɫ)
		

		// Display in a GUI
		imshow("aligned_depth_color", aligned_depth_color);
		//imshow("aligned_depth", aligned_depth*15);
		imshow("color", color);

		char key = waitKey(1);
		if (key == 32 || key == 13 || key == 'q')//�ո񡢻س���q
		{
			savetxt(aligned_depth,color,fx,fy,cx,cy,depth_scale,"outpu.txt",true);
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


void savetxt(const Mat &depth, const Mat &RGBImg, float fx, float fy, float cx, float cy,
			const float depthscale,const char* filename, bool isRGBPointCloud = true)
{


	ofstream fout(filename);
	for (int i = 0; i < depth.rows; i++)
	{
		for (int j = 0; j < depth.cols; j++)
		{
			Point3f tempts;
			float RGB[3];
			ushort d = depth.at<ushort>(i, j);
			float dist = d * depthscale * lengthscale;//mm
			if (d <= 0||d>backgroud*lengthscale)
			{
				continue;
			}
			else
			{
				tempts.x = (j - cx)*dist / fx;
				tempts.y = (i - cy)*dist / fy;
				tempts.z = dist;
				fout << tempts.x << "  " << tempts.y << "  "  << tempts.z ;
				if (isRGBPointCloud)
				{
					RGB[0] = RGBImg.at<Vec3b>(i, j)[2];
					RGB[1] = RGBImg.at<Vec3b>(i, j)[1];
					RGB[2] = RGBImg.at<Vec3b>(i, j)[0];
					fout << "  "<<RGB[0] << "  " << RGB[1] << "  " <<RGB[2] ;
				}
				fout << std::endl;
			}
		}
	}
	fout.close();

}





