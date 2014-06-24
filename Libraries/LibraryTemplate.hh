#ifndef _LIBRARY_TEMPLATE_HH_
#define _LIBRARY_TEMPLATE_HH_

#include "WorldServer.hh"

using namespace std;

namespace gazebo
{
	class ControlCenter;
}

class LibraryTemplate
{
	public: 
		LibraryTemplate();

		virtual ~LibraryTemplate();

		virtual void Load(gazebo::ControlCenter *CurrentWorld);

		virtual void WhenRunning(void);
};

#endif