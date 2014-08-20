//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Joel Sjögren
// Description: This is a utility header file that help us to output
//               color log texts. More information anout this can be
//              found from its original location: http://stackoverflow.com/
// questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef _COLOR_LOG_HH_
#define _COLOR_LOG_HH_
#include <ostream>
/// Color namespace has functions can log colorful texts in terminal
using std::cout;
using std::endl;
namespace Color {
/// Color code definition
enum Code {
  FG_RED      = 31,
  FG_GREEN    = 32,
  FG_BLUE     = 34,
  FG_DEFAULT  = 39,
  BG_RED      = 41,
  BG_GREEN    = 42,
  BG_BLUE     = 44,
  BG_DEFAULT  = 49,
  FG_YELLOW   = 33,
};
/// Can be used to generate a specific type of color logs
class Modifier {
  /// Color code that can be used
  Code code;
  /// Whether the log is showed in bold font
  bool bold;
 public:
  /// Constructor
  /*!
    \param pCode Color code name
  */
  Modifier(Code pCode) : code(pCode) {bold = false;}
  /// Constructor
  /*!
    \param pCode Color code name
    \param pBold Bold font setup flag
  */
  Modifier(Code pCode,bool pBold) : code(pCode) {bold = pBold;}
  friend std::ostream&
  /// Defines a special color log operator
  /*!
    \param os A reference of ostream object
    \param mod A reference to the Modifier object
    \return A piece of cout that will change the log color and font weight
  */
  operator<<(std::ostream& os, const Modifier& mod) {
    if (mod.bold) {
      return os << "\033[1;" << mod.code << "m";
    }
    return os << "\033[0;" << mod.code << "m";
  }
};
/// Warning message log function
/*!
  \param message_str The text that needs to be loged
*/
inline void Warning(const char* message_str)
{
  Color::Modifier yellow_log(Color::FG_YELLOW, true);
  Color::Modifier def_log(Color::FG_DEFAULT);
  cout<<yellow_log<<"[Warning] "<<message_str<<def_log<<endl;
}
/// Error message log function
/*!
  \param message_str The text that needs to be loged
*/
inline void Error(const char* message_str)
{
  Color::Modifier red_log(Color::FG_RED, true);
  Color::Modifier def_log(Color::FG_DEFAULT);
  cout<<red_log<<"[Error] "<<message_str<<def_log<<endl;
}
}
#endif