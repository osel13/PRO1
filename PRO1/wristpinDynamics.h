#pragma once
#include <vector>
class wristpinDynamics
{
private:
public:
	double calculatePistonSideForce(double pistonTotalForce, double angleDeg, double halfStroke, double conrodLength);
	
	double calculateAlongConrodForce(double pistonTotalForce, double angleDeg, double halfStroke, double conrodLength);

	void debugPrint(std::vector<double> pistonTotalForce, std::vector<double> angleDeg, double halfStroke, double conrodLength);

};

