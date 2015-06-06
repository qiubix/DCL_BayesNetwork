#ifndef NETWORK_BUILDER_EXCEPTIONS
#define NETWORK_BUILDER_EXCEPTIONS

#include <exception>

class PointCloudIsEmptyException : public std::exception
{
    virtual const char* what() const throw() {
        return "Trying to build Bayes network based on empty point cloud.";
    }
};

#endif //NETWORK_BUILDER_EXCEPTIONS
