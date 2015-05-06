/*!
 * \file
 * \brief
 * \author tkornuta,,,
 */

#include <memory>
#include <string>

#include "SOMJSONReader.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

namespace Processors {
namespace SOMJSONReader {

SOMJSONReader::SOMJSONReader(const std::string & name) :
		Base::Component(name) , 
		filenames("filenames", boost::bind(&SOMJSONReader::onFilenamesChanged, this, _1, _2), "")
{
	registerProperty(filenames);

}

SOMJSONReader::~SOMJSONReader() {
}

void SOMJSONReader::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_models", &out_models);

	// Register handlers
	registerHandler("loadModels", boost::bind(&SOMJSONReader::loadModels, this));

}

bool SOMJSONReader::onInit() {
	LOG(LTRACE) << "SOMJSONReader::onInit()";
	// Load models at start.
	loadModels();
	return true;
}

bool SOMJSONReader::onFinish() {
	return true;
}

bool SOMJSONReader::onStop() {
	return true;
}

bool SOMJSONReader::onStart() {
	return true;
}

void SOMJSONReader::loadModels() {
	LOG(LTRACE) << "SOMJSONReader::loadModels()";

	// List of the returned SOMs.
	std::vector<AbstractObject*> models;
	
	// Names of models/JSON files.	
	std::vector<std::string> namesList;
	std::string s= filenames;
	boost::split(namesList, s, boost::is_any_of(";"));

	// Temporary variables - names.
	std::string name_cloud_xyzrgb;
	std::string name_cloud_xyzsift;
	
	// Iterate through JSON files.
	for (size_t i = 0; i < namesList.size(); i++){
		ptree ptree_file;
		try{
			// Open JSON file and load it to ptree.
			read_json(namesList[i], ptree_file);
			// Read JSON properties.
			model_name = ptree_file.get<std::string>("name");
			mean_viewpoint_features_number = ptree_file.get<int>("mean_viewpoint_features_number");
			name_cloud_xyzrgb = ptree_file.get<std::string>("cloud_xyzrgb");
			name_cloud_xyzsift = ptree_file.get<std::string>("cloud_xyzsift");
		}//: try
		catch(std::exception const& e){
			LOG(LERROR) << "SOMJSONReader: file "<< namesList[i] <<" not found or invalid\n";
			continue;	
		}//: catch

		LOG(LDEBUG) << "name_cloud_xyzrgb:" << name_cloud_xyzrgb;
		LOG(LDEBUG) << "name_cloud_xyzsift:" << name_cloud_xyzsift;
		

		// Read XYZRGB cloud.
		cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
		// Try to load the file.
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (name_cloud_xyzrgb, *cloud_xyzrgb) == -1) 
		{
			LOG(LERROR) << "SOMJSONReader: file "<< name_cloud_xyzrgb <<" not found\n";
			continue;
		}//: if

		// Read XYZRGB cloud.
		cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
		// Try to load the file.
		if (pcl::io::loadPCDFile<PointXYZSIFT> (name_cloud_xyzsift, *cloud_xyzsift) == -1) 
		{
			LOG(LERROR) << "SOMJSONReader: file "<< name_cloud_xyzsift <<" not found\n";
			continue;
		}//: if

		// Create SOModel and add it to list.
		SIFTObjectModel* model;
		model = dynamic_cast<SIFTObjectModel*>(produce());
		models.push_back(model);

	}//: for

	// Push models to output datastream.
	out_models.write(models);
}


void SOMJSONReader::onFilenamesChanged(const std::string & old_filenames, const std::string & new_filenames) {
	filenames = new_filenames;
	CLOG(LTRACE) << "onFilenamesChanged: " << std::string(filenames) << std::endl;
}

} //: namespace SOMJSONReader
} //: namespace Processors
