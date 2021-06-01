#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

//����ͼֻ�������������
int
main()
{
	//�򿪵���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("table_scene_mug_stereo_textured.pcd", *cloud);
	//�������߹�������
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	/****************************************************************************************
	���ַ��߹��Ʒ���
	 COVARIANCE_MATRIX ģʽ�Ӿ���ĳ����ľֲ������Э������󴴽�9�����֣������������ķ���
	AVERAGE_3D_GRADIENT   ģʽ����6������ͼ������ˮƽ����ʹ�ֱ�����ƽ�������ά�ݶȲ�ʹ�������ݶȼ������
						   �����㷨��
	AVERAGE_DEPTH����CHANGE  ģʽֻ������һ����һ�Ļ���ͼ���Ӷ�ƽ����ȱ仯���㷨��
	********************************************************************************************/
	ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);  //���÷��߹��Ƶķ�ʽAVERAGE_3D_GRADIENT

	ne.setMaxDepthChangeFactor(0.02f);   //������ȱ仯ϵ��
	ne.setNormalSmoothingSize(10.0f);   //���÷����Ż�ʱ���ǵ�����Ĵ�С
	ne.setInputCloud(cloud);               //����ĵ���
	ne.compute(*normals);                    //���㷨��
	//���ӻ�
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");   //�ӿڵ�����
	viewer.setBackgroundColor(0.0, 0.0, 0.5);    //������ɫ������
	viewer.addPointCloud(cloud);
	//viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);  //�����߼��뵽������
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return 0;
}