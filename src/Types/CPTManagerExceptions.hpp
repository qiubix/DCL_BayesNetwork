#pragma once
#ifndef CPT_MANAGER_EXCEPTIONS
#define CPT_MANAGER_EXCEPTIONS

#include <exception>

class DivergentCPTSizeException : public std::exception
{
  virtual const char* what() const throw() {
    return "CPT and probabilities vector have different sizes.";
  }
};

class IncorrectProbabilityValueException : public std::exception
{
  virtual const char* what() const throw() {
    return "Probability doesn't have correct value from range <0,1>.";
  }
};

#endif //CPT_MANAGER_EXCEPTIONS
