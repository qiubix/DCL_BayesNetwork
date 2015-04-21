#ifndef BAYES_NETWORK_EXCEPTIONS
#define BAYES_NETWORK_EXCEPTIONS

#include <exception>

class NodeNotFoundException : public std::exception
{
  virtual const char* what() const throw() {
    return "Node could not be found.";
  }
};

#endif //BAYES_NETWORK_EXCEPTIONS
