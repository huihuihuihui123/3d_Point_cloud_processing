#include<pcl/registration/correspondence_estimation.h>
#include<pcl/io/pcd_io.h>
#include<pcl/common/time.h>
#include<pcl/kdtree/io.h>
#include<vector>
#include<fstream>
#include <pcl/filters/extract_indices.h>
#include<unordered_set>
using namespace std;

float computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int k);


////点云重叠区域提取
//void getOverlapAB(pcl::PointCloud<pcl::PointXYZ>::Ptr &tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr &src,
//	pcl::PointCloud<pcl::PointXYZ>::Ptr &Outputtgt, pcl::PointCloud<pcl::PointXYZ>::Ptr &Outputsrc, float Resolution)
//{
//	int K = 1;
//	cout << "tgt点云密度为: "<< Resolution << endl;
//	unordered_set<int> setA, setB;
//	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//	kdtree.setInputCloud(src);
//	pcl::PointXYZ searchPoint;
//	std::vector<int> pointIdxNKNSearch(K);
//	std::vector<float> pointNKNSquaredDistance(K);
//	//Outputcloud->width = cloud_filtered->width;
//	//Outputcloud->height = cloud_filtered->height;
//	//Outputcloud->points.resize(Outputcloud->width * Outputcloud->height);
//	// 寻找距离重心最近的点
//	for (int j = 0; j < tgt->points.size(); j++)
//	{
//		searchPoint.x = tgt->points[j].x;
//		searchPoint.y = tgt->points[j].y;
//		searchPoint.z = tgt->points[j].z;
//
//		if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 && pointNKNSquaredDistance[0] < Resolution)
//		{
//			if (setB.find(pointIdxNKNSearch[0]) == setB.end())
//			{
//				setB.insert(pointIdxNKNSearch[0]);
//				float x1 = src->points[pointIdxNKNSearch[0]].x;
//				float y1 = src->points[pointIdxNKNSearch[0]].y;
//				float z1 = src->points[pointIdxNKNSearch[0]].z;
//				Outputsrc->points.push_back(pcl::PointXYZ(x1, y1, z1));
//				float x2 = tgt->points[j].x;
//				float y2 = tgt->points[j].y;
//				float z2 = tgt->points[j].z;
//				Outputtgt->points.push_back(pcl::PointXYZ(x2, y2, z2));
//			}
//			else
//			{
//				float x2 = tgt->points[j].x;
//				float y2 = tgt->points[j].y;
//				float z2 = tgt->points[j].z;
//				Outputtgt->points.push_back(pcl::PointXYZ(x2, y2, z2));
//			}
//			setA.insert(j);
//
//		}
//	}
//	Outputtgt->width = Outputtgt->points.size();
//	Outputtgt->height = 1;
//	Outputsrc->width = Outputsrc->points.size();
//	Outputsrc->height = 1;
//}


// 点云重叠区域提取
void getOverlapAB(pcl::PointCloud<pcl::PointXYZ>::Ptr &tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr &src,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &Outputtgt, pcl::PointCloud<pcl::PointXYZ>::Ptr &Outputsrc, float Resolution)
{
	int K = 1;
	cout << "tgt点云密度为: " << Resolution << endl;
	unordered_set<int> setA, setB;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(src);
	pcl::PointXYZ searchPoint;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	//Outputcloud->width = cloud_filtered->width;
	//Outputcloud->height = cloud_filtered->height;
	//Outputcloud->points.resize(Outputcloud->width * Outputcloud->height);
	// 寻找距离重心最近的点
	for (int j = 0; j < tgt->points.size(); j++)
	{
		searchPoint.x = tgt->points[j].x;
		searchPoint.y = tgt->points[j].y;
		searchPoint.z = tgt->points[j].z;

		if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 && pointNKNSquaredDistance[0] < Resolution*0.5)
		{
			if (setB.find(pointIdxNKNSearch[0]) == setB.end())
			{
				setB.insert(pointIdxNKNSearch[0]);
				float x1 = src->points[pointIdxNKNSearch[0]].x;
				float y1 = src->points[pointIdxNKNSearch[0]].y;
				float z1 = src->points[pointIdxNKNSearch[0]].z;
				Outputsrc->points.push_back(pcl::PointXYZ(x1, y1, z1));
				float x2 = tgt->points[j].x;
				float y2 = tgt->points[j].y;
				float z2 = tgt->points[j].z;
				Outputtgt->points.push_back(pcl::PointXYZ(x2, y2, z2));
			}
			else
			{
				float x2 = tgt->points[j].x;
				float y2 = tgt->points[j].y;
				float z2 = tgt->points[j].z;
				Outputtgt->points.push_back(pcl::PointXYZ(x2, y2, z2));
			}
			setA.insert(j);

		}
	}
	kdtree.setInputCloud(tgt);
	for (int j = 0; j < src->points.size(); j++)
	{
		if (setB.find(j) != setB.end()) continue;
		searchPoint.x = src->points[j].x;
		searchPoint.y = src->points[j].y;
		searchPoint.z = src->points[j].z;

		if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 && pointNKNSquaredDistance[0] < Resolution)
		{
			if (setA.find(pointIdxNKNSearch[0]) == setA.end())
			{
				setA.insert(pointIdxNKNSearch[0]);
				float x1 = tgt->points[pointIdxNKNSearch[0]].x;
				float y1 = tgt->points[pointIdxNKNSearch[0]].y;
				float z1 = tgt->points[pointIdxNKNSearch[0]].z;
				Outputtgt->points.push_back(pcl::PointXYZ(x1, y1, z1));
				float x2 = src->points[j].x;
				float y2 = src->points[j].y;
				float z2 = src->points[j].z;
				Outputsrc->points.push_back(pcl::PointXYZ(x2, y2, z2));
				
			}
			else
			{
				float x2 = src->points[j].x;
				float y2 = src->points[j].y;
				float z2 = src->points[j].z;
				Outputsrc->points.push_back(pcl::PointXYZ(x2, y2, z2));
			}
			setB.insert(j);

		}
	}
	Outputtgt->width = Outputtgt->points.size();
	Outputtgt->height = 1;
	Outputsrc->width = Outputsrc->points.size();
	Outputsrc->height = 1;
}

//点云密度计算
float computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int k)
{
	double res = 0.0;
	int n_points = 0;
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);
	//?what means size_t
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
			//pcl_isfinite函数返回一个布尔值，检查某个值是不是正常数值
		{
			continue;
		}
		std::vector<int> indices(k);
		//创建一个动态数组，存储查询点近邻索引
		std::vector<float> sqr_distances(k);
		//存储近邻点对应平方距离
		if (tree.nearestKSearch(i, k, indices, sqr_distances) == k)
		{
			for (int i = 1; i < k; i++)
			{
				res += sqrt(sqr_distances[i]);
				++n_points;
			}
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}


int main()
{
	/*ofstream oa, ob;
	oa.open("overlap_a.txt");
	ob.open("overlap_b.txt");*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr OverlapA(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr OverlapB(new pcl::PointCloud<pcl::PointXYZ>);

	//读取PCD点云文件

	pcl::io::loadPCDFile<pcl::PointXYZ>("Z0smooth.pcd", *cloudA);
	pcl::io::loadPCDFile<pcl::PointXYZ>("Z60smooth.pcd", *cloudB);
	cout << "read PCD down" << endl;
	float Resolution = computeCloudResolution(cloudA, 2);
	getOverlapAB(cloudA, cloudB, OverlapA, OverlapB, Resolution*0.5);
	cout << "remove down" << endl;
	pcl::io::savePCDFileASCII("OverlapA2.pcd", *OverlapA);
	pcl::io::savePCDFileASCII("OverlapB2.pcd", *OverlapB);
	cout << "save down" << endl;
	return 0;
}
