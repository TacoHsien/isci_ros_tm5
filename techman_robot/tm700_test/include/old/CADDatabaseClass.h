#include "PCD_Function.h"

class CADDatabaseClass
{
	private:
		pcl::PointCloud<pcl::PointXYZ>::Ptr CADModelCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr Downsampling_CADModelCloud;
		pcl::PointCloud<pcl::Normal>::Ptr CADModelNormal;
		pcl::PointCloud<pcl::Normal>::Ptr Downsampling_CADModelNormal;
		pcl::PointCloud<pcl::Boundary>::Ptr CADModelBoundaryInf;
		pcl::PointCloud<pcl::PointXYZ>::Ptr CADModelBoundaryCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr CADModelReferenceCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr CADModelSACSegmentationCloud;
		std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > CADModel_OriginalPCDVector;


	public:
		int HashTable_totalIndex;
		CADDatabaseClass();
		std::vector<int> CADModel_referencePoint_indices;
		pcl::PointCloud<pcl::PointXYZ>::Ptr getCADModelCloud();
		pcl::PointCloud<pcl::PointXYZ>::Ptr getDownsampling_CADModelCloud();
		pcl::PointCloud<pcl::Normal>::Ptr getDownsampling_CADModelNormal();
		pcl::PointCloud<pcl::Normal>::Ptr getCADModelNormal();
		pcl::PointCloud<pcl::PointXYZ>::Ptr getCADModelBoundaryCloud();
		pcl::PointCloud<pcl::Boundary>::Ptr getCADModelBoundaryInf();
		pcl::PointCloud<pcl::PointXYZ>::Ptr getCADModelReferenceCloud();
		pcl::PointCloud<pcl::PointXYZ>::Ptr getCADModelSACSegmentationCloud();
		std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > getCADModel_OriginalPCDVector();
};