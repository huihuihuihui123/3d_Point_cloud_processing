#include <opencv2/opencv.hpp> 
#include <fstream>


int main()
{
	//1.填入自己需要的程序
	//2.get3DPointsFromRGBDepth()
	//3.save txt
	return 0;
}
/**
 1. Function:       计算RGB图中像素对应的3D坐标,单位：m
 2. Description:    计算RGB图中像素对应的3D坐标(相对于相机坐标系),调用此函数前一定要进行深度图与彩色图对齐
 3. Calls:
 4. include.h:      #include<opencv2/opencv.hpp>
					#include<iostream>
 5. Input:			pixels--输入彩色图中的2D像素点--Point2f(x,y)
					Depth--深度图
					depth_scale--深度分辨率，由realsense获取，D415下此值为0.001
					fx\fy\cx\cy--深度相机内参

 6. Output:			3D点数组（以相机坐标系为原点）
 */

std::vector<cv::Point3f> get3DPointsFromRGBDepth(std::vector<cv::Point2f> pixels, cv::Mat Depth, float depth_scale, float fx, float fy, float cx, float cy)
{
	std::vector<cv::Point3f> res;//返回结果
	int backgroud = 5;//5米以外的为无效点
	for (int i = 0; i < pixels.size(); i++)
	{
		cv::Point2f Point = pixels[i];
		cv::Point3f Pointxyz;
		//深度单位：米
		ushort d = Depth.at<ushort>(Point.y, Point.x); float dist = d * depth_scale; //如果深度图的mat是ushort类型的，则需要乘上深度分辨率depth_scale；
		//float dist=Depth.at<float>(Point.y, Point.x);								//如果深度图的mat是float类型，则直接读取。
		if (dist <= 0 || dist > backgroud) res.push_back(cv::Point3f(0., 0., 0.));//如果求出的深度值小于0或大于5米，则认为是无效点，
		Pointxyz.x = (Point.x - cx)*dist / fx;
		Pointxyz.y = (Point.y - cy)*dist / fy;
		Pointxyz.z = dist;
		res.push_back(Pointxyz);
	}
	return res;
}

/**
 1. Function:       将vector<cv::Point3f> res保存成txt文件
 2. Description:    将vector<cv::Point3f> res保存成txt文件
 3. Calls:
 4. include.h:      #include<opencv2/opencv.hpp>
					#include<iostream>
 5. Input:			res--3D点数组
					filename--常量字符串，如"output.txt"等

 6. Output:			3D点数组（以相机坐标系为原点）
 */
void savetxt(std::vector<cv::Point3f> res, const char* filename)
{
	std::ofstream fout(filename);
	for (int i = 0; i < res.size(); i++)
	{
		cv::Point3f tempts = res[i];
		if (i != res.size() - 1)
		{
			fout << tempts.x << "  " << tempts.y << "  " << tempts.z;
			fout << std::endl;
		}
		else
		{
			fout << tempts.x << "  " << tempts.y << "  " << tempts.z;
		}

	}
	fout.close();

}