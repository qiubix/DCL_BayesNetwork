#ifndef CPT_MANAGER_EXCEPTIONS
#define CPT_MANAGER_EXCEPTIONS

#include <exception>

class DivergentCPTSizeException : public std::exception
{
  virtual const char* what() const throw() {
    return "CPT and probabilities vector have different sizes.";
  }
};

#endif //CPT_MANAGER_EXCEPTIONS
