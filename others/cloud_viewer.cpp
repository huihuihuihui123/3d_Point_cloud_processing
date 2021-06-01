#include <pcl/visualization/cloud_viewer.h>   //��cloud_viewerͷ�ļ�����
#include <pcl/visualization/pcl_visualizer.h>   //��cloud_viewerͷ�ļ�����
#include <iostream>                           //��׼�������ͷ�ļ�����
#include <pcl/io/io.h>                        //I/O���ͷ�ļ�����
#include <pcl/io/pcd_io.h>                    //PCD�ļ���ȡ


/**********************************************************************************
  ��������Ϊ�ص�����������������ֻע��һ�� ������ʵ�ֶԿ��ӻ����󱳾���ɫ�����ã����һ��Բ�򼸺���
*********************************************************************************/
int user_data;

void
viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);       //���ñ�����ɫ
	pcl::PointXYZ o;                                  //�洢���Բ��λ��
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);                  //���Բ�򼸺ζ���
	std::cout << "i only run once" << std::endl;

}
/***********************************************************************************
��Ϊ�ص�����������������ע���ÿ֡��ʾ��ִ��һ�Σ���������ʵ���ڿ��ӻ����������һ��ˢ����ʾ�ַ���
*************************************************************************************/
void
viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	user_data++;
}
/**************************************************************
���ȼ��ص����ļ������ƶ��󣬲���ʼ�����ӻ�����viewer��ע������Ļ�
 ��������ִ��ѭ��ֱ���յ��ر�viewer����Ϣ�˳�����
 *************************************************************/
int
main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);    //����cloud 
	pcl::io::loadPCDFile("maize.pcd", *cloud);         //���ص����ļ�

	pcl::visualization::CloudViewer viewer("Cloud Viewer");      //����viewer����

	//showCloud������ͬ���ģ��ڴ˴��ȴ�ֱ����Ⱦ��ʾΪֹ
	viewer.showCloud(cloud);

	//��ע�ắ���ڿ��ӻ���ʱ��ִֻ��һ��
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	//��ע�ắ������Ⱦ���ʱÿ�ζ�����
	viewer.runOnVisualizationThread(viewerPsycho);
	while (!viewer.wasStopped())
	{
		//�˴����������������
		//FIXME: Note that this is running in a separate thread from viewerPsycho
		//and you should guard against race conditions yourself...
		user_data++;
	}
	return 0;
}