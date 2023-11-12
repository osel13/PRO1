#pragma once
#include "Kinematics.h"
#include <fstream>
#include <vector>

double Kinematics::calculatePistonPosition(double angleDeg, double halfStroke, double conrodLength)
{
	double angleRad = angleDeg * 3.14159 / 180;
	double lambda = halfStroke / conrodLength;
	double s = halfStroke * (1 - cos(angleRad) + (lambda / 4) * (1 - cos(2*angleRad)));
	return s;
}

double Kinematics::calculatePistonPositionFirstOrder(double angleDeg, double halfStroke)
{
	double angleRad = angleDeg * 3.14159 / 180;
	double s1 = halfStroke * (1 - cos(angleRad));
	return s1;
}

double Kinematics::calculatePistonPositionSecondOrder(double angleDeg, double halfStroke, double conrodLength)
{
	double angleRad = angleDeg * 3.14159 / 180;
	double lambda = halfStroke / conrodLength;
	double s2 = halfStroke * ((lambda/4)*(1 - cos(2*angleRad)));
	return s2;
}

double Kinematics::calculatePistonVelocity(double RPM, double angleDeg, double halfStroke, double conrodLength)
{
	double angleRad = angleDeg * 3.14159 / 180;
	double angularVelocity = (RPM/60)*2* 3.14159;
	double lambda = halfStroke / conrodLength;
	double v = halfStroke * angularVelocity * (sin(angleRad)+(lambda/2)*sin(2*angleRad));
	return v;
}

double Kinematics::calculatePistonVelocityFirstOrder(double RPM, double angleDeg, double halfStroke)
{
	double angleRad = angleDeg * 3.14159 / 180;
	double angularVelocity = (RPM / 60) * 2 * 3.14159;
	double v1 = halfStroke * angularVelocity * sin(angleRad);
	return v1;
}

double Kinematics::calculatePistonVelocitySecondOrder(double RPM, double angleDeg, double halfStroke, double conrodLength)
{
	double angleRad = angleDeg * 3.14159 / 180;
	double angularVelocity = (RPM / 60) * 2 * 3.14159;
	double lambda = halfStroke / conrodLength;
	double v2 = halfStroke * angularVelocity * ((lambda / 2) * sin( 2* angleRad));
	return v2;
}

double Kinematics::calculatePistonAcceleration(double RPM, double angleDeg, double halfStroke, double conrodLength)
{
	double angleRad = angleDeg * 3.14159 / 180;
	double angularVelocity = (RPM / 60) * 2 * 3.14159;
	double lambda = halfStroke / conrodLength;
	double a = halfStroke * pow(angularVelocity, 2) * (cos(angleRad)+lambda*cos(2*angleRad));
	return a;
}

double Kinematics::calculatePistonAccelerationFirstOrder(double RPM, double angleDeg, double halfStroke)
{
	double angleRad = angleDeg * 3.14159 / 180;
	double angularVelocity = (RPM / 60) * 2 * 3.14159;
	double a1 = halfStroke * pow(angularVelocity, 2) * (cos(angleRad));
	return a1;
}

double Kinematics::calculatePistonAccelerationSecondOrder(double RPM, double angleDeg, double halfStroke, double conrodLength)
{
	double angleRad = angleDeg * 3.14159 / 180;
	double angularVelocity = (RPM / 60) * 2 * 3.14159;
	double lambda = halfStroke / conrodLength;
	double a2 = halfStroke * pow(angularVelocity, 2) * (lambda * cos(2 * angleRad));
	return a2;

}

void Kinematics::debugPrint(double RPM, double halfStroke, double conrodLength)
{
	Kinematics k;
	//generate angle
	double angleMax = 720;
	double angleStep = 0.5;

	std::ofstream Debug;
	Debug.open("debugKinematics.csv");
	Debug << "angle[deg]" << ",";
	Debug << "pistonPosition[mm]" << ",";
	Debug << "pistonPositionFirstOrder[mm]" << ",";
	Debug << "pistonPositionSecondOrder[mm]" << ",";
	Debug << "pistonVelocity[mm]" << ",";
	Debug << "pistonVelocityFirstOrder[mm]" << ",";
	Debug << "pistonVelocitySecondOrder[mm]" << ",";
	Debug << "pistonAcceleration[mm]" << ",";
	Debug << "pistonAccelerationFirstOrder[mm]" << ",";
	Debug << "pistonAccelerationSecondOrder[mm]" << ",";
	Debug << std::endl;


	for (int i = 0; i < angleMax / angleStep; i++)
	{
		Debug << i * angleStep << ",";
		double PP = k.calculatePistonPosition(i*angleStep,halfStroke,conrodLength);
		Debug << PP << ",";
		double PPFO = k.calculatePistonPositionFirstOrder(i * angleStep, halfStroke);
		Debug << PPFO << ",";
		double PPSO = k.calculatePistonPositionSecondOrder(i * angleStep, halfStroke,  conrodLength);
		Debug << PPSO << ",";

		double PV = k.calculatePistonVelocity( RPM, i * angleStep,  halfStroke,  conrodLength);
		Debug << PV << ",";
		double PVFO = k.calculatePistonVelocityFirstOrder( RPM, i * angleStep,  halfStroke);
		Debug << PVFO << ",";
		double PVSO = k.calculatePistonVelocitySecondOrder( RPM, i * angleStep,  halfStroke,  conrodLength);
		Debug << PVSO << ",";

		double PA = k.calculatePistonAcceleration( RPM, i * angleStep,  halfStroke,  conrodLength);
		Debug << PA << ",";
		double PAFO = k.calculatePistonAccelerationFirstOrder( RPM, i * angleStep,  halfStroke);
		Debug << PAFO << ",";
		double PASO = k.calculatePistonAccelerationSecondOrder( RPM, i * angleStep,  halfStroke,  conrodLength);
		Debug << PASO << std::endl;

	}

	Debug.close();


}
