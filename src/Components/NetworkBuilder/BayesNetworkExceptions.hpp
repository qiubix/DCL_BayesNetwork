#ifndef BAYES_NETWORK_EXCEPTIONS
#define BAYES_NETWORK_EXCEPTIONS

#include <exception>

class NodeAlreadyExistsException : public std::exception
{
  virtual const char* what() const throw() {
    return "This node already exists in network.";
  }
};

class NodeNotFoundException : public std::exception
{
  virtual const char* what() const throw() {
    return "Node could not be found.";
  }
};

class UnableToConnectNodesException : public std::exception
{
  virtual const char* what() const throw() {
    return "Nodes cannot be connected.";
  }
};

#endif //BAYES_NETWORK_EXCEPTIONS
