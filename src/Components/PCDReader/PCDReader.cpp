/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "PCDReader.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>


namespace Processors {
namespace PCDReader {

PCDReader::PCDReader(const std::string & name) :
		Base::Component(name), 
		filename("filename", std::string("")) 
		{
		registerProperty(filename);

}

PCDReader::~PCDReader() {
}

void PCDReader::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	// Register handlers
	h_Read.setup(boost::bind(&PCDReader::Read, this));
	registerHandler("Read", &h_Read);
	//addDependency("Read", NULL);

}

bool PCDReader::onInit() {

	return true;
}

bool PCDReader::onFinish() {
	return true;
}

bool PCDReader::onStop() {
	return true;
}

bool PCDReader::onStart() {
	return true;
}

void PCDReader::Read() {
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
	  if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud_xyz) == -1) //* load the file
	  {
		cout <<"Błąd"<<endl;
	  }
	  out_cloud_xyz.write(cloud_xyz);
	  
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud_xyzrgb) == -1) //* load the file
	  {
		cout <<"Błąd"<<endl;
	  }
	  out_cloud_xyzrgb.write(cloud_xyzrgb);
	  
	  pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift (new pcl::PointCloud<PointXYZSIFT>);
	  if (pcl::io::loadPCDFile<PointXYZSIFT> (filename, *cloud_xyzsift) == -1) //* load the file
	  {
		cout <<"Błąd"<<endl;
	  }
	  out_cloud_xyzsift.write(cloud_xyzsift);	
}



} //: namespace PCDReader
} //: namespace Processors
