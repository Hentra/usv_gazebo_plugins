#ifndef THRUSTMAPPING_H
#define THRUSTMAPPING_H

#include <iostream>

enum ThrustMapping 
{
	linear=0, 
	GLF=1
};

std::ostream& operator<<(std::ostream& os, const ThrustMapping tm);
#endif
