#pragma once
#include "wristpinDynamics.h"
#include <math.h>
#include <fstream>
#include <vector>
#include <iostream>


double wristpinDynamics::calculatePistonSideForce(double pistonTotalForce,double angleDeg,double halfStroke, double conrodLength)
{
	double angleRad = angleDeg * 3.14159 / 180;
	double lambda = halfStroke / conrodLength;
	double beta = sqrt(1 - pow(lambda * sin(angleRad), 2));
	double PSF = pistonTotalForce / cos(beta);
	return PSF;


}


double wristpinDynamics::calculateAlongConrodForce(double pistonTotalForce, double angleDeg, double halfStroke, double conrodLength)
{
	double angleRad = angleDeg * 3.14159 / 180;
	double lambda = halfStroke / conrodLength;
	double beta = sqrt(1 - pow(lambda * sin(angleRad), 2));
	double ACF = pistonTotalForce*tan(beta);
	return ACF;
}

void wristpinDynamics::debugPrint(std::vector<double> pistonTotalForce,std::vector<double> angleDeg, double halfStroke, double conrodLength)
{
	wristpinDynamics wD;

	std::fstream Debug;
	Debug.open("DebugWristpinDynamics.csv");
	Debug << "angle" << ",";
	Debug << "PistonSideForce" << ",";
	Debug << "AlongConrodForce" << std::endl;

	for (int i = 0; i < angleDeg.size(); i++)
	{
		Debug << angleDeg[i] << ",";
		Debug << wD.calculatePistonSideForce(pistonTotalForce[i], angleDeg[i], halfStroke, conrodLength) << ",";
		Debug << wD.calculateAlongConrodForce(pistonTotalForce[i], angleDeg[i], halfStroke, conrodLength) << std::endl;
		
	}
	Debug.close();
}
