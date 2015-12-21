#ifndef ABSTRACTOBJECTFACTORY_HPP_
#define ABSTRACTOBJECTFACTORY_HPP_
#include <Types/AbstractObject.hpp> 

//namespace Types {

class AbstractObjectFactory
{
	public:
	virtual AbstractObject* produce()=0;
	virtual ~AbstractObjectFactory(){}
	AbstractObjectFactory(){}
};


//} //: namespace Types

#endif /* ABSTRACTOBJECTFACTORY_HPP_ */
