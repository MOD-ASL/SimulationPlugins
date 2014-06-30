#ifndef _UTIL_HH_
#define _UTIL_HH_

#include <string>
#include <sstream>

namespace util
{
template < typename T > std::string to_string( const T& n )
{
    std::ostringstream stm ;
    stm << n ;
    return stm.str() ;
}
inline string Int2String(int number)
{
  stringstream ss; //create a stringstream
  ss << number;    //add number to the stream
  return ss.str(); //return a string with the contents of the stream
} // Int2String
}

#endif