// correspondence_points_registraction.cpp: 定义控制台应用程序的入口点。
//

//pcl::correspondences类里面有query和match两个成员，分别是sourc和target点云上对应点对的索引。
//以通过source[query].x等等来访问具体的坐标值。
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

	//读取PCD点云文件
	pcl::io::loadPCDFile<pcl::PointXYZ>("turn_pointcloud0mls.pcd", *cloudA);
	pcl::io::loadPCDFile<pcl::PointXYZ>("turn_pointcloud60mls.pcd", *cloudB);


	pcl::StopWatch time;
	//点云重叠索引计算
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
	core.setInputSource(cloudA);
	core.setInputTarget(cloudB);
	boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);   //共享所有权的智能指针，以kdtree做索引
	core.determineReciprocalCorrespondences(*cor, 0.5);   //点之间的最大距离,cor对应索引


	//索引提取
	pcl::PointIndices::Ptr cloudAinliers(new pcl::PointIndices());
	for (size_t i = 0; i < cor->size(); i++)
	{
		cloudAinliers->indices.push_back(cor->at(i).index_query);
	}
	
	

	//将输入点云重叠部分删除
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAwithoutCover(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;        //创建点云提取对象
		extract.setInputCloud(cloudA);
		extract.setIndices(cloudAinliers);
		extract.setNegative(true);//留下非重叠部分
		extract.filter(*cloudAwithoutCover);

		std::cout << "fuse Time(s): " << time.getTimeSeconds() << std::endl;
		std::cout << "cover point size :" << cor->size() << std::endl;
		std::cout << "finish fuse , total size:" << cloudAwithoutCover->size() << std::endl;

		pcl::io::savePCDFileASCII("cloudAwithoutCover.pcd", *cloudAwithoutCover);
		////构造重叠点云的PCD格式文件
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
	////将提取出的重叠点云写入PCD
	//for (size_t i = 0; i < cor->size(); i++) {

	//	//overlapA写入pcd文件
	//	overlapA.points[i].x = cloudA->points[cor->at(i).index_query].x;
	//	overlapA.points[i].y = cloudA->points[cor->at(i).index_query].y;
	//	overlapA.points[i].z = cloudA->points[cor->at(i).index_query].z;

	//	//overlapB写入pcd文件
	//	overlapB.points[i].x = cloudB->points[cor->at(i).index_match].x;
	//	overlapB.points[i].y = cloudB->points[cor->at(i).index_match].y;
	//	overlapB.points[i].z = cloudB->points[cor->at(i).index_match].z;
	//}
	////存储pcd文件
	//pcl::io::savePCDFileASCII("overlapA.pcd", overlapA);
	//pcl::io::savePCDFileASCII("overlapB.pcd", overlapB);

	//oa, ob.close();	
		return 0;
}


//去除重叠点云 ，用view2视角的点云代替view1
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

	int* mask_1 = new int[cloud_1->size()];  //2的点云是新的点云，所以重叠的部分显示二的，这个就存的是相应的1中的index
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
	int idxInTotalCnt = 0;  //在totalCloud中的index
	for (int i = 0; i < cloud2Cnt + cloud_1->size(); i++)
	{
		if (i < cloud_2->size())
		{
			totalPointCloud->points[idxInTotalCnt] = cloud_2->points[idxInTotalCnt];
			idxInTotalCnt++;
		}
		else
		{
			if (mask_1[i - cloud2Cnt] == -1)  //已经用2给他赋过值了
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
