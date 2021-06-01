// correspondence_points_registraction.cpp: �������̨Ӧ�ó������ڵ㡣
//

//pcl::correspondences��������query��match������Ա���ֱ���sourc��target�����϶�Ӧ��Ե�������
//��ͨ��source[query].x�ȵ������ʾ��������ֵ��
#include<pcl/registration/correspondence_estimation.h>
#include<pcl/io/pcd_io.h>
#include<pcl/common/time.h>
#include<pcl/kdtree/io.h>
#include<vector>
#include<fstream>
#include <pcl/filters/extract_indices.h>
using namespace std;


pcl::PointCloud<pcl::PointXYZ>::Ptr FuseTwoPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_2);
int main()
{
	/*ofstream oa, ob;
	oa.open("overlap_a.txt");
	ob.open("overlap_b.txt");*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);

	//��ȡPCD�����ļ�
	pcl::io::loadPCDFile<pcl::PointXYZ>("turn_pointcloud0mls.pcd", *cloudA);
	pcl::io::loadPCDFile<pcl::PointXYZ>("turn_pointcloud60mls.pcd", *cloudB);


	pcl::StopWatch time;
	//�����ص���������
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
	core.setInputSource(cloudA);
	core.setInputTarget(cloudB);
	boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);   //��������Ȩ������ָ�룬��kdtree������
	core.determineReciprocalCorrespondences(*cor, 0.5);   //��֮���������,cor��Ӧ����


	//������ȡ
	pcl::PointIndices::Ptr cloudAinliers(new pcl::PointIndices());
	for (size_t i = 0; i < cor->size(); i++)
	{
		cloudAinliers->indices.push_back(cor->at(i).index_query);
	}
	
	

	//����������ص�����ɾ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAwithoutCover(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;        //����������ȡ����
		extract.setInputCloud(cloudA);
		extract.setIndices(cloudAinliers);
		extract.setNegative(true);//���·��ص�����
		extract.filter(*cloudAwithoutCover);

		std::cout << "fuse Time(s): " << time.getTimeSeconds() << std::endl;
		std::cout << "cover point size :" << cor->size() << std::endl;
		std::cout << "finish fuse , total size:" << cloudAwithoutCover->size() << std::endl;

		pcl::io::savePCDFileASCII("cloudAwithoutCover.pcd", *cloudAwithoutCover);
		////�����ص����Ƶ�PCD��ʽ�ļ�
		//pcl::PointCloud<pcl::PointXYZ>overlapA;
		//pcl::PointCloud<pcl::PointXYZ>overlapB;


		////pcl::PointCloud<pcl::PointXYZ>::Ptr totalAB = FuseTwoPointCloud(cloudA, cloudB);
		////pcl::io::savePCDFileASCII("totalAB.pcd",*totalAB);
		//overlapA.width = cor->size();
		//overlapA.height = 1;
		//overlapA.is_dense = false;
		//overlapA.points.resize(overlapA.width*overlapA.height);

		//overlapB.width = cor->size();
		//overlapB.height = 1;
		//overlapB.is_dense = false;
		//overlapB.points.resize(overlapB.width*overlapB.height);
	////����ȡ�����ص�����д��PCD
	//for (size_t i = 0; i < cor->size(); i++) {

	//	//overlapAд��pcd�ļ�
	//	overlapA.points[i].x = cloudA->points[cor->at(i).index_query].x;
	//	overlapA.points[i].y = cloudA->points[cor->at(i).index_query].y;
	//	overlapA.points[i].z = cloudA->points[cor->at(i).index_query].z;

	//	//overlapBд��pcd�ļ�
	//	overlapB.points[i].x = cloudB->points[cor->at(i).index_match].x;
	//	overlapB.points[i].y = cloudB->points[cor->at(i).index_match].y;
	//	overlapB.points[i].z = cloudB->points[cor->at(i).index_match].z;
	//}
	////�洢pcd�ļ�
	//pcl::io::savePCDFileASCII("overlapA.pcd", overlapA);
	//pcl::io::savePCDFileASCII("overlapB.pcd", overlapB);

	//oa, ob.close();	
		return 0;
}


//ȥ���ص����� ����view2�ӽǵĵ��ƴ���view1
pcl::PointCloud<pcl::PointXYZ>::Ptr FuseTwoPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_2)
{
	pcl::StopWatch time;
	std::cout << "start erase overlap part...." << std::endl;
	// ... read or fill in source and target
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> cor_est;
	cor_est.setInputSource(cloud_1);
	cor_est.setInputTarget(cloud_2);
	pcl::Correspondences all_correspondences;
	// Determine all reciprocal correspondences
	cor_est.determineReciprocalCorrespondences(all_correspondences,0.5);
	if (all_correspondences.size() == 0)
	{
		std::cout << "no valid points to match and fuse,return cloud_1 ! " << std::endl;
		return cloud_1;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr totalPointCloud(new pcl::PointCloud<pcl::PointXYZ>());
	totalPointCloud->resize(cloud_1->size() + cloud_2->size() - all_correspondences.size());

	int* mask_1 = new int[cloud_1->size()];  //2�ĵ������µĵ��ƣ������ص��Ĳ�����ʾ���ģ�����ʹ������Ӧ��1�е�index
	for (int i = 0; i < cloud_1->size(); i++)
	{
		mask_1[i] = 0;  //init mask 1
	}

	for (int i = 0; i < all_correspondences.size(); i++)
	{
		int idxInCloud1 = all_correspondences[i].index_query;
		mask_1[idxInCloud1] = -1;
	}

	int cloud2Cnt = cloud_2->size();
	int idxInTotalCnt = 0;  //��totalCloud�е�index
	for (int i = 0; i < cloud2Cnt + cloud_1->size(); i++)
	{
		if (i < cloud_2->size())
		{
			totalPointCloud->points[idxInTotalCnt] = cloud_2->points[idxInTotalCnt];
			idxInTotalCnt++;
		}
		else
		{
			if (mask_1[i - cloud2Cnt] == -1)  //�Ѿ���2��������ֵ��
			{
				continue;
			}
			else
			{
				totalPointCloud->points[idxInTotalCnt] = cloud_1->points[i - cloud2Cnt];
				idxInTotalCnt++;
			}
		}
	}

	std::cout << "fuse Time(s): " << time.getTimeSeconds() << std::endl;
	std::cout << "finish fuse , total size:" << totalPointCloud->size() << std::endl;

	totalPointCloud->is_dense = true;
	return totalPointCloud;
}
