#include "SpiderController.hh"

using namespace std;

SpiderController::SpiderController()
{}

SpiderController::~SpiderController()
{}

void SpiderController::Load(gazebo::ControlCenter *CurrentWorld)
{}

void SpiderController::WhenRunning(void)
{
	cout<<"I am in the spider pugin"<<endl;
}

extern "C" {
LibraryTemplate *maker()
{
   return new SpiderController;
}
void destroy(LibraryTemplate* p)
{
  delete p;   // Can use a base class or derived class pointer here
}
}