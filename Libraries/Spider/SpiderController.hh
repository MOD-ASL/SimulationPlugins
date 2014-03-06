#ifndef _SPIDER_CONTROLLER_HH_
#define _SPIDER_CONTROLLER_HH_

#include "LibraryTemplate.hh"

using namespace std;

class SpiderController : public LibraryTemplate
{
public:
	SpiderController();

	~SpiderController();

	void Load(gazebo::ControlCenter *CurrentWorld);

	void WhenRunning(void);

};

#endif