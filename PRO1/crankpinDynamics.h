#pragma once
#include <vector>
class crankpinDynamics
{
private:

public:

	double calculateCrankpinInertia(double pistonMass, double conrodSlidingMass, double pistonAcceleration);


	double calculateAlongCylinderTotalForce(double pressureForce, double crankpinInertia);

	double calculateAlongConrodTotalForce(double AlongCylinderTotalForce, double angleDeg, double halfStroke, double conrodLength);


	double calculateNormalForce(double AlongConrodTotalForce, double angleDeg, double halfStroke, double conrodLength);


	double calculateConrodRadialForce(double AlongConrodTotalForce, double angleDeg, double halfStroke, double conrodLength);


	double calculateConrodTangentialForce(double AlongConrodTotalForce, double angleDeg, double halfStroke, double conrodLength);


	double calculateConrodRotationalInertia(double conrodRotatingMass, double halfStroke, double RPM);


	double calculateTotalRadialForce(double conrodRadialForce, double conrodRotationalInertia, double conrodTangentialForce);

	void debugPrint(std::vector<double> pistonAcceleration, std::vector<double> pressureForce, std::vector<double> angleDeg, double pistonMass, double conrodSlidingMass, double conrodRotatingMass, double halfStroke, double conrodLength, double RPM);
};

