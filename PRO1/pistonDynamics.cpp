#pragma once
#include "pistonDynamics.h"
#include <fstream>
#include <vector>

double pistonDynamics::calculatePistonPressureForce(double pressure,double surface, double pressureAtmospheric)
{

	double pressureForce = (pressure - pressureAtmospheric) * surface;
	return pressureForce;
}

double pistonDynamics::calculatePistonInertia(double pistonMass, double pistonAcceleration)
{
	double pistonInertia = pistonMass * pistonAcceleration*0.001; //m*s^-2
	return pistonInertia;
}

double pistonDynamics::calculatePistonTotalForce(double pistonPressureForce, double pistonInertia)
{
	double totalForce = pistonPressureForce+pistonInertia;
	return totalForce;
}

void pistonDynamics::debugPrint(std::vector<double> pistonAcceleration, std::vector<double> angle, std::vector<double> pressure, double surface, double pressureAtmospheric,double pistonMass )
{
	pistonDynamics pD;
	std::ofstream Debug;
	Debug.open("pistonDynamicsDebug.csv");
	Debug << "angle" << ",";
	Debug << "pistonPressureForce [N]" << ",";
	Debug << "pistonInertiaForce [N]" << ",";
	Debug << "pistonTotalForce [N]" << std:: endl;

	for (int i = 0; i < pistonAcceleration.size(); i++)
	{
		Debug << angle[i] << ",";
		double PPF = pD.calculatePistonPressureForce(pressure[i],surface,pressureAtmospheric) ;
		Debug << PPF << ",";
		double PIF = pD.calculatePistonInertia(pistonMass,pistonAcceleration[i]);
		Debug << PIF << ",";
		double PTF = pD.calculatePistonTotalForce(PPF,PIF);
		Debug << PTF << std::endl;

	}

	Debug.close();


}
