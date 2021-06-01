#include <iostream>
#include <string>
#include <boost/make_shared.hpp>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>//包含fpfh加速计算的omp多核并行计算

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>




using namespace std;
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

typedef pcl::FPFHSignature33 FPFHT;
typedef pcl::PointCloud<FPFHT> FPFHCloud;

const float VOXEL_GRID_SIZE = 0.05;
const double radius_normal = 10;
const double radius_feature = 30;
const double max_sacia_iterations = 1000;
const double min_correspondence_dist = 0.01;
const double max_correspondence_dist = 1000;
// This is a tutorial so we can afford having global variables 
pcl::visualization::PCLVisualizer *p;
//its left and right viewports
int vp_1, vp_2;

pcl::console::TicToc timecal;
pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
//convenient structure to handle our pointclouds
struct PCD
{
	PointCloud::Ptr cloud;
	std::string f_name;

	PCD() : cloud(new PointCloud) {};
};

struct PCDComparator
{
	bool operator () (const PCD& p1, const PCD& p2)
	{
		return (p1.f_name < p2.f_name);
	}
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

public:
	MyPointRepresentation()
	{
		// Define the number of dimensions
		nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PointNormalT &p, float * out) const
	{
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};


void voxelFilter(PointCloud::Ptr &cloud_in, PointCloud::Ptr &cloud_out, float gridsize) {
	pcl::VoxelGrid<PointT> vox_grid;
	vox_grid.setLeafSize(gridsize, gridsize, gridsize);
	vox_grid.setInputCloud(cloud_in);
	vox_grid.filter(*cloud_out);
	cout << "PointCloud before voxelfiltering: " << cloud_in->width * cloud_in->height
		<< " data points (" << pcl::getFieldsList(*cloud_in) << ")." << endl;
	cout << "PointCloud after voxelfiltering: " << cloud_out->width * cloud_out->height
		<< " data points (" << pcl::getFieldsList(*cloud_out) << ").\n" << endl;
	return;
}

pcl::PointCloud<pcl::Normal>::Ptr getNormals(PointCloud::Ptr cloud, double radius)
{
	pcl::PointCloud<pcl::Normal>::Ptr normalsPtr(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> norm_est;
	norm_est.setInputCloud(cloud);
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(radius);
	//norm_est.setRadiusSearch(radius);
	norm_est.compute(*normalsPtr);
	return normalsPtr;

}

FPFHCloud::Ptr getFeatures(PointCloud::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double radius)
{
	FPFHCloud::Ptr features(new FPFHCloud);
	pcl::FPFHEstimationOMP<PointT, pcl::Normal, FPFHT> fpfh_est;
	fpfh_est.setNumberOfThreads(4);
	fpfh_est.setInputCloud(cloud);
	fpfh_est.setInputNormals(normals);
	fpfh_est.setSearchMethod(tree);
	fpfh_est.setKSearch(radius);
	// fpfh_est.setRadiusSearch(radius);
	fpfh_est.compute(*features);
	return features;
}
////////////////////////////////
//显示配准结果，刚性变换矩阵
void print4x4Matrix(Eigen::Matrix4f &matrix)
{

	printf("Rotation and Translation matrix 3x4:\n");
	printf("    | %6.3f %6.3f %6.3f %6.3f| \n", matrix(0, 0), matrix(0, 1), matrix(0, 2),matrix(0, 3));
	printf("RT =| %6.3f %6.3f %6.3f %6.3f| \n", matrix(1, 0), matrix(1, 1), matrix(1, 2),matrix(1, 3));
	printf("    | %6.3f %6.3f %6.3f %6.3f| \n", matrix(2, 0), matrix(2, 1), matrix(2, 2),matrix(2, 3));
}



//sac_ia配准
void sac_ia_align(PointCloud::Ptr source, PointCloud::Ptr target, PointCloud::Ptr& finalcloud, 
	Eigen::Matrix4f& init_transform,
	int max_sacia_iterations, double min_correspondence_dist, double max_correspondence_dist)
{

	vector<int> indices1;
	vector<int> indices2;
	PointCloud::Ptr sourceds(source);
	PointCloud::Ptr targetds(target);
	pcl::removeNaNFromPointCloud(*source, *source, indices1);
	pcl::removeNaNFromPointCloud(*target, *target, indices2);
	//降采样
	/*voxelFilter(source, sourceds, VOXEL_GRID_SIZE);
	voxelFilter(target, targetds, VOXEL_GRID_SIZE);*/
	cout << "1" << endl;
	//计算法向量
	pcl::PointCloud<pcl::Normal>::Ptr source_normal(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr target_normal(new pcl::PointCloud<pcl::Normal>);
	source_normal = getNormals(sourceds, radius_normal);
	target_normal = getNormals(targetds, radius_normal);
	cout << "2" << endl;
	//计算FPFH特征    
	FPFHCloud::Ptr source_feature(new FPFHCloud);
	FPFHCloud::Ptr target_feature(new FPFHCloud);
	source_feature = getFeatures(sourceds, source_normal, radius_feature);
	target_feature = getFeatures(targetds, target_normal, radius_feature);
	cout << "3" << endl;
	//SAC-IA配准  
	pcl::SampleConsensusInitialAlignment<PointT, PointT, FPFHT> sac_ia;
	Eigen::Matrix4f final_transformation;
	sac_ia.setInputTarget(targetds);
	sac_ia.setTargetFeatures(target_feature);
	sac_ia.setInputSource(sourceds);
	sac_ia.setSourceFeatures(source_feature);
	sac_ia.setMaximumIterations(max_sacia_iterations);
	sac_ia.setMinSampleDistance(min_correspondence_dist);
	sac_ia.setMaxCorrespondenceDistance(max_correspondence_dist);
	sac_ia.setRANSACIterations(30);
	sac_ia.setEuclideanFitnessEpsilon(0.001);
	sac_ia.setTransformationEpsilon(1e-10);
	sac_ia.setCorrespondenceRandomness(6);

	PointCloud::Ptr output(new PointCloud);
	timecal.tic();
	sac_ia.align(*output);
	cout << "Finished SAC_IA Initial Regisration in " << timecal.toc() << "ms\n" << endl;
	init_transform = sac_ia.getFinalTransformation();
	
	pcl::transformPointCloud(*source, *finalcloud, init_transform);
	cout << "粗配准:" << endl;
	print4x4Matrix(init_transform);
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
*
*/
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
	p->removePointCloud("vp1_target");
	p->removePointCloud("vp1_source");

	PointCloudColorHandlerCustom<PointT> tgt_h(cloud_target, 0, 255, 0);
	PointCloudColorHandlerCustom<PointT> src_h(cloud_source, 255, 0, 0);
	p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
	p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);


}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
*
*/
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
	p->removePointCloud("source");
	p->removePointCloud("target");


	PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(cloud_target, "curvature");
	if (!tgt_color_handler.isCapable())
		PCL_WARN("Cannot create curvature color handler!");

	PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(cloud_source, "curvature");
	if (!src_color_handler.isCapable())
		PCL_WARN("Cannot create curvature color handler!");


	p->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
	p->addPointCloud(cloud_source, src_color_handler, "source", vp_2);

	p->spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
* \param argc the number of arguments (pass from main ())
* \param argv the actual command line arguments (pass from main ())
* \param models the resultant vector of point cloud datasets
*/
void loadData(int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
	std::string extension(".pcd");
	// Suppose the first argument is the actual test model
	for (int i = 1; i < argc; i++)
	{
		std::string fname = std::string(argv[i]);
		// Needs to be at least 5: .plot
		if (fname.size() <= extension.size())
			continue;

		std::transform(fname.begin(), fname.end(), fname.begin(), (int(*)(int))tolower);

		//check that the argument is a pcd file
		if (fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0)
		{
			// Load the cloud and saves it into the global list of models
			PCD m;
			m.f_name = argv[i];
			pcl::io::loadPCDFile(argv[i], *m.cloud);
			//remove NAN points from the cloud
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);

			models.push_back(m);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
* \param cloud_src the source PointCloud
* \param cloud_tgt the target PointCloud
* \param output the resultant aligned source PointCloud
* \param final_transform the resultant transform between source and target
*/
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
	//
	// Downsample for consistency and speed
	// \note enable this for large datasets
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);
	pcl::VoxelGrid<PointT> grid;
	if (downsample)
	{
		grid.setLeafSize(0.003, 0.003, 0.003);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);

		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}


	// Compute surface normals and curvature
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

	pcl::NormalEstimation<PointT, PointNormalT> norm_est;

	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);
	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*src, *points_with_normals_src);

	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	//
	// Align
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	reg.setTransformationEpsilon(1e-6);
	//reg.setEuclideanFitnessEpsilon(1);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(5);
	//reg.RANSACOutlierRejectionThreshold(1.5);
	reg.setMaximumIterations(1000);
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));
	reg.setInputSource(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);

	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, SourceTotarget;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;

	for (int i = 0; i < 15; ++i)
	{
		points_with_normals_src = reg_result;// save cloud for visualization purpose
		reg.setInputSource(points_with_normals_src);
		timecal.tic();
		reg.align(*reg_result);
		//cout<<"Applied %ICP "<<time.toc()<<" ms\n";	
		PCL_INFO("TIME:Applied num %d ICP in %f ms", i + 1, timecal.toc());
		std::cout << "ICP has converged, score is\n " << reg.getFitnessScore() << std::endl;
		Ti = reg.getFinalTransformation() * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

		prev = reg.getLastIncrementalTransformation();

		// visualize current state
		showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	}

	// Get the transformation from target to source
	//SourceTotarget = Ti.inverse();
	SourceTotarget = Ti;
	final_transform = SourceTotarget;
	std::cout << "\nICP has converged, score is " << reg.getFitnessScore() << std::endl;
	//std::cout <<icp.getFinalTransformation ()<<std::endl;
	cout << "ICP_transform:" << endl;
	print4x4Matrix(final_transform);

	// Transform target back in source frame
	pcl::transformPointCloud(*cloud_src, *output, SourceTotarget);

	p->removePointCloud("source");
	p->removePointCloud("target");

	PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 255, 0, 0);
	PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 0, 255, 0);
	p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
	p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);

	//PCL_INFO("Press q to continue the registration.\n");
	//p->spin();
	////调试先注释

	//p->removePointCloud("source");
	//p->removePointCloud("target");

	//add the source to the transformed target
	//*output += *cloud_src;


}


/* ---[ */
int main(int argc, char** argv)
{

	PointCloud::Ptr source_ori(new PointCloud);
	PointCloud::Ptr target_ori(new PointCloud);
	PointCloud::Ptr source(new PointCloud);
	PointCloud::Ptr target(new PointCloud);
	pcl::io::loadPCDFile("segment0.pcd", *source_ori);
	pcl::io::loadPCDFile("segment1.pcd", *target_ori);
	if (true)
	{
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setLeafSize(0.01f, 0.01f, 0.01f);
		sor.setInputCloud(source_ori);
		sor.filter(*source);
		sor.setInputCloud(target_ori);
		sor.filter(*target);
	}


	// Create a PCLVisualizer object
	p = new pcl::visualization::PCLVisualizer(argc, argv, "Pairwise Incremental Registration example");
	p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
	p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);

	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;



	// Add visualization data
	showCloudsLeft(source, target);
	PointCloud::Ptr result(new PointCloud);
	PointCloud::Ptr temp(new PointCloud);
	PointCloud::Ptr final(new PointCloud);
	PointCloud::Ptr init_result(new PointCloud);
	Eigen::Matrix4f init_transform = Eigen::Matrix4f::Identity();
	//PCL_INFO("Aligning %s (%d) with %s (%d).\n", data[i - 1].f_name.c_str(), source->points.size(), data[i].f_name.c_str(), target->points.size());
	sac_ia_align(source, target, init_result, init_transform, max_sacia_iterations, min_correspondence_dist, max_correspondence_dist);
	pcl::io::savePCDFile("sac.pcd", *init_result);
	pairAlign(init_result, target, temp, pairTransform, false);
	/*pairTransform *= init_transform;

	pcl::transformPointCloud(*temp, *result, GlobalTransform);
	*final += *result;*/
	GlobalTransform = pairTransform * init_transform;//ICP.mat*SAC-IC.mat
	cout << "全局变换矩阵(粗配准+精配准:" << endl;
	print4x4Matrix(GlobalTransform);

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*target, centroid);
	std::cout << "XYZ:(" << centroid[0] << "," << centroid[1] << "," << centroid[2] << ")." << std::endl;
	cin.get();
}
