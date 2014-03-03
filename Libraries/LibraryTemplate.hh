#ifndef _LIBRARY_TEMPLATE_HH_
#define _LIBRARY_TEMPLATE_HH_

#include "SuperServer.hh"

using namespace std;

class LibraryTemplate
{
	public: 
		LibraryTemplate();

		virtual ~LibraryTemplate();

		virtual void Load(gazebo::ControlCenter *CurrentWorld);

		virtual void WhenRunning(void);
};

#endif