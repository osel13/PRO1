#pragma once
#include "crankpinDynamics.h"
#include <math.h>
#include <fstream>
#include <vector>



double crankpinDynamics::calculateCrankpinInertia(double pistonMass, double conrodSlidingMass, double pistonAcceleration)
{
	double crankpinInertia = (pistonMass + conrodSlidingMass) * pistonAcceleration;
	return crankpinInertia;
}


double crankpinDynamics::calculateAlongCylinderTotalForce(double pressureForce, double crankpinInertia)
{
	double ACylTF = pressureForce - crankpinInertia;
	return ACylTF;
}



double crankpinDynamics::calculateAlongConrodTotalForce(double AlongCylinderTotalForce, double angleDeg,double halfStroke, double conrodLength)
{
	double angleRad = angleDeg * 3.14159 / 180;
	double lambda = halfStroke / conrodLength;
	double beta = sqrt(1 - pow(lambda * sin(angleRad), 2));
	double AConTF = AlongCylinderTotalForce / (cos(beta));
	return AConTF;
}



double crankpinDynamics::calculateNormalForce(double AlongConrodTotalForce, double angleDeg, double halfStroke, double conrodLength)
{
	double angleRad = angleDeg * 3.14159 / 180;
	double lambda = halfStroke / conrodLength;
	double beta = sqrt(1 - pow(lambda * sin(angleRad), 2));
	double NF = AlongConrodTotalForce * sin(beta);
	return NF;
}



double crankpinDynamics::calculateConrodRadialForce(double AlongConrodTotalForce, double angleDeg, double halfStroke, double conrodLength)
{
	double angleRad = angleDeg * 3.14159 / 180;
	double lambda = halfStroke / conrodLength;
	double beta = sqrt(1 - pow(lambda * sin(angleRad), 2));
	double CRF = -AlongConrodTotalForce * (cos(angleRad + beta));
	return CRF;
}



double crankpinDynamics::calculateConrodTangentialForce(double AlongConrodTotalForce, double angleDeg, double halfStroke, double conrodLength)
{
	double angleRad = angleDeg * 3.14159 / 180;
	double lambda = halfStroke / conrodLength;
	double beta = sqrt(1 - pow(lambda * sin(angleRad), 2));
	double CTF = AlongConrodTotalForce * (sin(angleRad + beta));
	return CTF;
}



double crankpinDynamics::calculateConrodRotationalInertia(double conrodRotatingMass,double halfStroke, double RPM)
{
	double angularVelocity = (RPM / 60) * 2 * 3.14159;
	double CRIF = conrodRotatingMass * halfStroke * pow(angularVelocity, 2);
	return CRIF;

}



double crankpinDynamics::calculateTotalRadialForce(double conrodRadialForce, double conrodRotationalInertia)
{
	double TRF = conrodRadialForce + conrodRotationalInertia;
	return TRF;
}

void crankpinDynamics::debugPrint(std::vector<double> pistonAcceleration, std::vector<double> pressureForce, std::vector<double> angleDeg,double pistonMass, double conrodSlidingMass, double conrodRotatingMass, double halfStroke, double conrodLength, double RPM)
{
	crankpinDynamics cD;
	std::ofstream Debug;
	Debug.open("crankpinDynamicsDebug.csv");
	Debug << "CrankpinInertia" << ",";
	Debug << "AlongCylinderTotalForce" << ",";
	Debug << "calculateAlongConrodTotalForce" << ",";
	Debug << "NormalForce" << ",";
	Debug << "ConrodRadialForce" << ",";
	Debug << "ConrodTangentialForce" << ",";
	Debug << "ConrodRotationalInertia" << ",";
	Debug << "TotalRadialForce" << std::endl;

	for (int i = 0; i < angleDeg.size(); i++)
	{
		double CI = cD.calculateCrankpinInertia(pistonMass, conrodSlidingMass, pistonAcceleration[i]);
		double ACylTF = cD.calculateAlongCylinderTotalForce( pressureForce[i], CI);
		double AConTF = cD.calculateAlongConrodTotalForce(ACylTF,angleDeg[i], halfStroke, conrodLength);
		double NF = cD.calculateNormalForce(AConTF, angleDeg[i], halfStroke, conrodLength);
		double CRF = cD.calculateConrodRadialForce(AConTF, angleDeg[i], halfStroke, conrodLength);
		double CTF = cD.calculateConrodTangentialForce(AConTF, angleDeg[i], halfStroke, conrodLength);
		double CRI = cD.calculateConrodRotationalInertia(conrodRotatingMass, halfStroke, RPM);
		double TRF = cD.calculateTotalRadialForce(CRF, CRI);

		Debug << CI; 
		Debug << ",";
		Debug << ACylTF;
		Debug << ",";
		Debug << AConTF;
		Debug << ",";
		Debug << NF;
		Debug << ",";
		Debug << CRF;
		Debug << ",";
		Debug << CTF;
		Debug << ",";
		Debug << CRI;
		Debug << ",";
		Debug << TRF;
		Debug << std::endl;
	
	}

	Debug.close();
}
