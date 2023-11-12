#pragma once
#include <vector>
class Momentums
{
private:
public:
	std::vector<double> momentumToNthCylinder(std::vector<double> tangentialForces, std::vector<double> angleDeg, int cylinderNumber, int numberOfCylinders, double halfStroke);
	/*
	std::vector<double> slidingInertialForcesFirstOrder();

	std::vector<double> slidingInertialForcesSecondOrder();
	
	std::vector<double> slidingMomentumFirstOrder();

	std::vector<double> slidingMomentumSecondOrder();
	*/
	//double rotatingMomentum()

	std::vector<double> Sum(std::vector<double> tangentialForces, std::vector<double> angleDeg, int numberOfCylinders, double halfStroke);

	double Average(std::vector<double> tangentialForces, std::vector<double> angleDeg, int numberOfCylinders, double halfStroke);// Nm

	void DebugPrint(std::vector<double> tangentialForces, std::vector<double> angleDeg, /*int cylinderNumber, int numberOfCylinders,*/ double halfStroke);
};


