#include <opencv2/opencv.hpp> 
#include <fstream>


int main()
{
	//1.�����Լ���Ҫ�ĳ���
	//2.get3DPointsFromRGBDepth()
	//3.save txt
	return 0;
}
/**
 1. Function:       ����RGBͼ�����ض�Ӧ��3D����,��λ��m
 2. Description:    ����RGBͼ�����ض�Ӧ��3D����(������������ϵ),���ô˺���ǰһ��Ҫ�������ͼ���ɫͼ����
 3. Calls:
 4. include.h:      #include<opencv2/opencv.hpp>
					#include<iostream>
 5. Input:			pixels--�����ɫͼ�е�2D���ص�--Point2f(x,y)
					Depth--���ͼ
					depth_scale--��ȷֱ��ʣ���realsense��ȡ��D415�´�ֵΪ0.001
					fx\fy\cx\cy--�������ڲ�

 6. Output:			3D�����飨���������ϵΪԭ�㣩
 */

std::vector<cv::Point3f> get3DPointsFromRGBDepth(std::vector<cv::Point2f> pixels, cv::Mat Depth, float depth_scale, float fx, float fy, float cx, float cy)
{
	std::vector<cv::Point3f> res;//���ؽ��
	int backgroud = 5;//5�������Ϊ��Ч��
	for (int i = 0; i < pixels.size(); i++)
	{
		cv::Point2f Point = pixels[i];
		cv::Point3f Pointxyz;
		//��ȵ�λ����
		ushort d = Depth.at<ushort>(Point.y, Point.x); float dist = d * depth_scale; //������ͼ��mat��ushort���͵ģ�����Ҫ������ȷֱ���depth_scale��
		//float dist=Depth.at<float>(Point.y, Point.x);								//������ͼ��mat��float���ͣ���ֱ�Ӷ�ȡ��
		if (dist <= 0 || dist > backgroud) res.push_back(cv::Point3f(0., 0., 0.));//�����������ֵС��0�����5�ף�����Ϊ����Ч�㣬
		Pointxyz.x = (Point.x - cx)*dist / fx;
		Pointxyz.y = (Point.y - cy)*dist / fy;
		Pointxyz.z = dist;
		res.push_back(Pointxyz);
	}
	return res;
}

/**
 1. Function:       ��vector<cv::Point3f> res�����txt�ļ�
 2. Description:    ��vector<cv::Point3f> res�����txt�ļ�
 3. Calls:
 4. include.h:      #include<opencv2/opencv.hpp>
					#include<iostream>
 5. Input:			res--3D������
					filename--�����ַ�������"output.txt"��

 6. Output:			3D�����飨���������ϵΪԭ�㣩
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