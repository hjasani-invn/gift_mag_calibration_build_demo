#ifndef MEDIAN_TEMPLATE_H
#define MEDIAN_TEMPLATE_H
#include <algorithm>


///Represents the exception for taking the median of an empty list
class median_of_empty_list_exception:public std::exception{
  virtual const char* what() const throw() {
    return "Attempt to take the median of an empty list of numbers.  "
      "The median of an empty list is undefined.";
  }
};

///Return the median of a sequence of numbers defined by the random
///access iterators begin and end.  The sequence must not be empty
///(median is undefined for an empty set).
///
///The numbers must be convertible to double.
template<class RandAccessIter>
double median(RandAccessIter begin, RandAccessIter end) 
#ifdef _WIN32
#pragma warning( push )
#pragma warning( disable : 4290 )

  throw(median_of_empty_list_exception)
#pragma warning( pop )
{
  if(begin == end){ throw median_of_empty_list_exception(); }

#else
{
#endif
  std::size_t size = end - begin;
  std::size_t middleIdx = size/2;
  RandAccessIter target = begin + middleIdx;
  std::nth_element(begin, target, end);

  if(size % 2 != 0){ //Odd number of elements
    return *target;
  }else{            //Even number of elements
    double a = *target;
    RandAccessIter targetNeighbor= target-1;
    std::nth_element(begin, targetNeighbor, end);
    return (a+*targetNeighbor)/2.0;
  }
}

#endif //MEDIAN_TEMPLATE_H
