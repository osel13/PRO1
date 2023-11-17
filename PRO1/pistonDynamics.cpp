#pragma once
#include "pistonDynamics.h"
#include <vector>
#include <fstream>

double pistonDynamics::calculatePistonPressureForce(double pistonPressure, double surface, double pressureAtmospheric)
{
	double PPF = (pistonPressure  - pressureAtmospheric) * surface;
	return PPF;
}

double pistonDynamics::calculatePistonInertia(double pistonMass, double conrodSlidingMass, double pistonAcceleration)
{
	double PI = -(pistonMass + conrodSlidingMass) * pistonAcceleration;
	return PI;
}

double pistonDynamics::calculatePistonTotalForce(double pistonPressureForce, double pistonInertialForce)
{
	double PTF = pistonPressureForce + pistonInertialForce;
	return PTF;
}

int pistonDynamics::debugPrint(std::vector<double> angle, std::vector<double> pressureForce, std::vector<double> pistonInertialForce, std::vector<double> pistonTotalForce)
{
	std::ofstream Debug;
	Debug.open("pistonDynamicsDebug.csv");
	Debug << "angle [deg],pressureForce[N],pistonInertialForce[N],pistonTotalForce[N]";
	Debug << std::endl;

	for (int i =0;i<angle.size();i++)
	{
		Debug << angle[i];
		Debug << ",";
		Debug << pressureForce[i];
		Debug << ",";
		Debug << pistonInertialForce[i];
		Debug << ",";
		Debug << pistonTotalForce[i];
		Debug << std::endl;
	}

	Debug.close();

	return 0;
}
