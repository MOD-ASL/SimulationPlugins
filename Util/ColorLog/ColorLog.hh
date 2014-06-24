//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Joel Sjögren
// Description: This is a utility header file that help us to output
//               color log texts. More information anout this can be
//              found from its original location: http://
//stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef _COLOR_LOG_HH_
#define _COLOR_LOG_HH_
#include <ostream>
namespace Color {
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
class Modifier {
  Code code;
  bool bold;
 public:
  Modifier(Code pCode) : code(pCode) {bold = false;}
  Modifier(Code pCode,bool pBold) : code(pCode) {bold = pBold;}
  friend std::ostream&
  operator<<(std::ostream& os, const Modifier& mod) {
    if (mod.bold) {
      return os << "\033[1;" << mod.code << "m";
    }
    return os << "\033[0;" << mod.code << "m";
  }
};
}
#endif