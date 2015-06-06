/*!
 * \file
 * \brief 
 * \author tkornuta,,,
 */

#ifndef SOMJSONREADER_HPP_
#define SOMJSONREADER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

//#include <Types/SIFTObjectModel.hpp> 
#include <Types/SIFTObjectModelFactory.hpp> 

namespace Processors {
namespace SOMJSONReader {

/*!
 * \class SOMJSONReader
 * \brief SOMJSONReader processor class.
 *
 * SOMJSONReader processor.
 */
class SOMJSONReader: public Base::Component, SIFTObjectModelFactory {
public:
	/*!
	 * Constructor.
	 */
	SOMJSONReader(const std::string & name = "SOMJSONReader");

	/*!
	 * Destructor
	 */
	virtual ~SOMJSONReader();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();

	/// Output data stream containing models.
	Base::DataStreamOut<std::vector<AbstractObject*> > out_models;

	/// List of the files containing models to be read.
	Base::Property<string> filenames;

	
	/// Load models from files.
	void loadModels();

	/*!
	 * Callback called when list of filenames changes.
	 */
	void onFilenamesChanged(const std::string & old_filenames, const std::string & new_filenames);

};

} //: namespace SOMJSONReader
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SOMJSONReader", Processors::SOMJSONReader::SOMJSONReader)

#endif /* SOMJSONREADER_HPP_ */
