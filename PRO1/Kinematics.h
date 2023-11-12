#pragma once
#include <vector>

class Kinematics
{
private:

public:
	double calculatePistonPosition(double angleDeg, double halfStroke, double conrodLength);
	double calculatePistonPositionFirstOrder(double angleDeg, double halfStroke);
	double calculatePistonPositionSecondOrder(double angleDeg, double halfStroke, double conrodLength);

	double calculatePistonVelocity(double RPM, double angleDeg, double halfStroke, double conrodLength);
	double calculatePistonVelocityFirstOrder(double RPM, double angleDeg, double halfStroke);
	double calculatePistonVelocitySecondOrder(double RPM, double angleDeg, double halfStroke, double conrodLength);

	double calculatePistonAcceleration(double RPM, double angleDeg, double halfStroke, double conrodLength);
	double calculatePistonAccelerationFirstOrder(double RPM, double angleDeg, double halfStroke);
	double calculatePistonAccelerationSecondOrder(double RPM, double angleDeg, double halfStroke, double conrodLength);

	void debugPrint(double RPM, double halfStroke, double conrodLength);




};

