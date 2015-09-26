#ifndef ABSTRACT_NETWORK_HPP
#define ABSTRACT_NETWORK_HPP

class AbstractNetwork
{
public:
  virtual ~AbstractNetwork(){};

  virtual double getNodeProbability(const std::string& name) = 0;
  virtual void clearEvidence() = 0;
  virtual bool nodeExists(const std::string& nodeName) = 0;
  virtual void setNodeEvidence(const std::string& nodeName, int state) = 0;

private:
  /* data */
};

#endif //ABSTRACT_NETWORK_HPP
