#pragma once
#include <vector>

class pistonDynamics
{
public:
	double calculatePistonPressureForce(double pistonPressure, double surface, double pressureAtmospheric);
	double calculatePistonInertia(double pistonMass, double conrodSlidingMass, double pistonAcceleration);
	double calculatePistonTotalForce(double pistonPressureForce, double pistonInertialForce);
	int debugPrint(std::vector<double> angle, std::vector<double> pressureForce, std::vector<double> pistonInertialForce, std::vector<double> pistonTotalForce);
};

