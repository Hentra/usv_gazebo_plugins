#ifndef IMPELLER_H
#define IMPELLER_H

#include <iostream>
#include <usv_gazebo_plugins/thrustmapping.h>

namespace gazebo 
{
	class Impeller	
	{
		public:

			Impeller(double maxForceFwd, double maxForceRev);
			Impeller(double maxForceFwd, double maxForceRev, ThrustMapping thrustMapping);

			double scaleThrust(double percentage);

			double getMaxForceFwd();
			double getMaxForceRev();
			ThrustMapping getThrustMapping();

			void setMaxForceFwd(double maxForce);
			void setMaxForceRev(double maxForce);
			void setThrustMapping(ThrustMapping thrustMapping);

		private:
			double maxForceFwd_;
			double maxForceRev_;
			ThrustMapping thrustMapping_;

			double scaleThrustLinear(double percentage);
			double scaleThrustGLF(double percentage);
			double glf(double x, float A, float K, float B,
	       					     float v, float C, float M);
	};

}
#endif
