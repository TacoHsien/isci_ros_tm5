#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include "CADDatabaseClass.h"

CADDatabaseClass::CADDatabaseClass()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cadModelCloud (new pcl::PointCloud<pcl::PointXYZ>);
	CADModelCloud = cadModelCloud;

	pcl::PointCloud<pcl::PointXYZ>::Ptr downsampling_CADModelCloud (new pcl::PointCloud<pcl::PointXYZ>);
	Downsampling_CADModelCloud = downsampling_CADModelCloud;

	pcl::PointCloud<pcl::Normal>::Ptr cadModelNormal (new pcl::PointCloud<pcl::Normal>);
	CADModelNormal = cadModelNormal;

	pcl::PointCloud<pcl::Normal>::Ptr downsampling_CADModelNormal (new pcl::PointCloud<pcl::Normal>);
	Downsampling_CADModelNormal = downsampling_CADModelNormal;

	pcl::PointCloud<pcl::Boundary>::Ptr cadModelBoundaryInf (new pcl::PointCloud<pcl::Boundary>);
	CADModelBoundaryInf = cadModelBoundaryInf;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cadModelBoundaryCloud (new pcl::PointCloud<pcl::PointXYZ>);
	CADModelBoundaryCloud = cadModelBoundaryCloud;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cadModelReferenceCloud (new pcl::PointCloud<pcl::PointXYZ>);
	CADModelReferenceCloud = cadModelReferenceCloud;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cadModelSACSegmentationCloud (new pcl::PointCloud<pcl::PointXYZ>);
	CADModelSACSegmentationCloud = cadModelSACSegmentationCloud;

	pcl::PointCloud<pcl::PointXYZ>::Ptr CADModel_OriginalPCD_0 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr CADModel_OriginalPCD_1 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr CADModel_OriginalPCD_2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr CADModel_OriginalPCD_3 (new pcl::PointCloud<pcl::PointXYZ>);
	CADModel_OriginalPCDVector.push_back(CADModel_OriginalPCD_0);
	CADModel_OriginalPCDVector.push_back(CADModel_OriginalPCD_1);
	CADModel_OriginalPCDVector.push_back(CADModel_OriginalPCD_2);
	CADModel_OriginalPCDVector.push_back(CADModel_OriginalPCD_3);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CADDatabaseClass::getCADModelSACSegmentationCloud()
{
	return CADModelSACSegmentationCloud;
}

std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > CADDatabaseClass::getCADModel_OriginalPCDVector()
{
	return CADModel_OriginalPCDVector;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CADDatabaseClass::getCADModelCloud()
{
	return CADModelCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CADDatabaseClass::getDownsampling_CADModelCloud()
{
	return Downsampling_CADModelCloud;
}

pcl::PointCloud<pcl::Normal>::Ptr CADDatabaseClass::getCADModelNormal()
{
	return CADModelNormal;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr CADDatabaseClass::getCADModelBoundaryCloud()
{
	return CADModelBoundaryCloud;
}

pcl::PointCloud<pcl::Boundary>::Ptr CADDatabaseClass::getCADModelBoundaryInf()
{
	return CADModelBoundaryInf;
}


pcl::PointCloud<pcl::Normal>::Ptr  CADDatabaseClass::getDownsampling_CADModelNormal()
{
	return Downsampling_CADModelNormal;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr CADDatabaseClass::getCADModelReferenceCloud()
{
	return CADModelReferenceCloud;
}