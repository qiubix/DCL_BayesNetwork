#ifndef BAYES_NETWORK_EXCEPTIONS
#define BAYES_NETWORK_EXCEPTIONS

#include <exception>

class NodeAlreadyExistsException : public std::exception
{
public:
  NodeAlreadyExistsException(const char* nodeName) : name(nodeName) {}
  ~NodeAlreadyExistsException() throw() {}
  virtual const char* what() const throw() {
    std::string message = "This node already exists in network: " + name;
    return message.c_str();
  }
private:
  std::string name;
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

class UnableToCreateNodeNameWithThisIdException : public std::exception
{
  virtual const char* what() const throw() {
    return "Cannot create node name with using this id.";
  }
};

#endif //BAYES_NETWORK_EXCEPTIONS
