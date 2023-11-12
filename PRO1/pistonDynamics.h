#pragma once
#include <vector>
class pistonDynamics
{
private:
public:

	double calculatePistonPressureForce(double pressure, double surface, double pressureAtmospheric);

	double calculatePistonInertia(double pistonMass, double pistonAcceleration);

	double calculatePistonTotalForce(double pistonPressureForce, double pistonInertia);
	
	void debugPrint(std::vector<double> pistonAcceleration, std::vector<double> angle, std::vector<double> pressure, double surface, double pressureAtmospheric, double pistonMass);
};

