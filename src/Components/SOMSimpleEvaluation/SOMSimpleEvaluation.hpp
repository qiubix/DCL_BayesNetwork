/*!
 * \file SOMSimpleEvaluation.hpp
 * \brief
 */

#ifndef SOM_SIMPLE_EVALUATION_HPP_
#define SOM_SIMPLE_EVALUATION_HPP_

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

#include "../../../lib/SMILE/smile.h"
#include <opencv2/core/core.hpp>


namespace Processors {
namespace Network {

/*!
 * \class SOMSimpleEvaluation
 * \brief Class used to evaluate instances basing on information about models' features multiplicity
 * \author Karol Katerżawa
 */
class SOMSimpleEvaluation: public Base::Component
{
public:
    /*!
     * Constructor.
     */
    SOMSimpleEvaluation(const std::string & name = "SOMSimpleEvaluation");

    /*!
     * Destructor
     */
    virtual ~SOMSimpleEvaluation();

    /*!
     * Prepare data streams and handlers
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

    /// Event handlers

    /*!
     * Event handler function.
     */

private:
};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("SOMSimpleEvaluation", Processors::Network::SOMSimpleEvaluation)

#endif /* SOM_SIMPLE_EVALUATION_HPP_ */

