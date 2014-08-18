#ifndef _UTIL_HH_
#define _UTIL_HH_

#include <string>
#include <sstream>
/// util namespace defines a bunch of utility functions
namespace util
{
/// A function template that convert number to string
/*!
  \param n A reference of T type
  \return Converted string
*/
template < typename T > std::string to_string( const T& n )
{
    std::ostringstream stm ;
    stm << n ;
    return stm.str() ;
}
/// Converts int to string
/*!
  \param number Number needs to be converted
  \return Converted string
*/
inline string Int2String(int number)
{
  stringstream ss; //create a stringstream
  ss << number;    //add number to the stream
  return ss.str(); //return a string with the contents of the stream
} // Int2String
}

#endif