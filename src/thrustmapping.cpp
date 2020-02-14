#include <usv_gazebo_plugins/impeller.h>

std::ostream& operator<<(std::ostream& os, const ThrustMapping tm)
{
	switch(tm)
	{
			case ThrustMapping::linear	: os << "Linear"; break;
			case ThrustMapping::GLF 	: os << "GLF"; break;
			default						: os.setstate(std::ios_base::failbit);
	}
	return os;
}
