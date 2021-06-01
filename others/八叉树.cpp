#include <pcl/point_cloud.h>                         // ��������
#include <pcl/point_types.h>                          //����������
#include <pcl/io/openni2_grabber.h>                    //���ƻ�ȡ�ӿ���
#include <pcl/visualization/cloud_viewer.h>            //���ƿ��ӻ���

#include <pcl/compression/octree_pointcloud_compression.h>   //����ѹ����

#include <stdio.h>
#include <sstream>
#include <stdlib.h>

#ifdef WIN32
# define sleep(x) Sleep((x)*1000)
#endif

class SimpleOpenNIViewer
{
public:
	SimpleOpenNIViewer() :
		viewer(" Point Cloud Compression Example")
	{
	}
	/************************************************************************************************
	  ��OpenNIGrabber�ɼ�ѭ��ִ�еĻص�����cloud_cb_�У����Ȱѻ�ȡ�ĵ���ѹ����stringstream����������һ�����ǽ�ѹ����
	  ����ѹ���˵Ķ��������ݽ��н��룬�洢���µĵ����н����˵��Ʊ����͵����ƿ��ӻ������н���ʵʱ���ӻ�
	*************************************************************************************************/

	void  cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		if (!viewer.wasStopped())
		{
			// �洢ѹ�����Ƶ��ֽ�������
			std::stringstream compressedData;
			// �洢�������
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());

			// ѹ������
			PointCloudEncoder->encodePointCloud(cloud, compressedData);

			// ��ѹ������
			PointCloudDecoder->decodePointCloud(compressedData, cloudOut);


			// ���ӻ���ѹ���ĵ���
			viewer.showCloud(cloudOut);
		}
	}
	/**************************************************************************************************************
	 �ں����д���PointCloudCompression��Ķ���������ͽ��룬��Щ�����ѹ�������ļ���Ϊ����ѹ���㷨�Ĳ���
	 ���ṩ��ѹ�������ļ�ΪOpenNI�����豸�ɼ����ĵ���Ԥ��ȷ����ͨ�ò�������������ʹ��MED_RES_ONLINE_COMPRESSION_WITH_COLOR
	 ���ò����������ڿ������ߵ�ѹ����ѹ�����÷����������ļ�/io/include/pcl/compression/compression_profiles.h���ҵ���
	  ��PointCloudCompression���캯����ʹ��MANUAL����CONFIGURATION���ԾͿ����ֶ�������ѹ���㷨��ȫ������
	******************************************************************************************************************/
	void run()
	{

		bool showStatistics = true;  //�����ڱ�׼�豸�������ӡ��ѹ�������Ϣ

		// ѹ��ѡ��������: /io/include/pcl/compression/compression_profiles.h
		pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

		// ��ʼ��ѹ���ͽ�ѹ������  ����ѹ��������Ҫ�趨ѹ������ѡ���ѹ����������Դ�����ж�
		PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
		PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();
		/***********************************************************************************************************
		����Ĵ���ΪOpenNI�����豸ʵ����һ���µĲ���������������ѭ���ص��ӿڣ�ÿ���豸��ȡһ֡���ݾͻص�����һ�Σ��������
		�ص���������ʵ������ѹ���Ϳ��ӻ���ѹ�������
	   ************************************************************************************************************/
	   //������OpenNI��ȡ���Ƶ�ץȡ����
		pcl::Grabber* interface =  new pcl::io::OpenNI2Grabber();

		// �����ص�����
		boost::function<void
		(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

		//�����ص������ͻص���Ϣ�İ�
		boost::signals2::connection c = interface->registerCallback(f);

		// ��ʼ���ܵ��Ƶ�������
		interface->start();

		while (!viewer.wasStopped())
		{
			sleep(1);
		}

		interface->stop();

		// ɾ��ѹ�����ѹ����ʵ��
		delete (PointCloudEncoder);
		delete (PointCloudDecoder);

	}

	pcl::visualization::CloudViewer viewer;

	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudEncoder;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudDecoder;

};

int
main(int argc, char **argv)
{
	SimpleOpenNIViewer v;  //����һ���µ�SimpleOpenNIViewer  ʵ������������run����
	v.run();

	return (0);
}