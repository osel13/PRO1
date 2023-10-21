/*
Zadani:
zážehový ètyøválec
setrvaèník + kliková høídel I_M= 0.12 
z CATIA I_M = 0.237 kg*m^2
Zátìž I_Z = 1.55 kg*m^2

z felicie:
kompresní pomìr

*/

#include <math.h>

double PI = 3.14159265359;

double bore = 75.5; // vrtání D = 75.5 mm
double stroke = 72; // zdvih Z = 72 mm
double surface = PI*bore*bore*0.25; //mm^2
double pistonMass; //mass of whole piston

double conrodMass;
double conrodInertia;
double conrodLength;
double conrodLengthA;
double conrodLengthB;

double halfStroke = stroke / 2;
double lambda = halfStroke / conrodLength;
double conrodSlidingMass = (conrodLengthB/conrodLength)*conrodMass;
double conrodRotatingMass = (conrodLengthA/conrodLength)*conrodMass;
double conrodInertiaPrime = conrodRotatingMass*conrodLengthB*conrodLengthB+conrodSlidingMass*conrodLengthA*conrodLengthA;
double conrodInertiaDelta = conrodInertia - conrodInertiaPrime;

double wristpinPosition(double angle);
double wristpinVelocity(double angle, double angularVelocity);
double wristpinAcceleration(double angle, double angularVelocity);

double conrodSlidingAcceleration(double angle, double angularVelocity);
double conrodAngularAcceleration(double angle, double angularVelocity);
double conrodMomentumDelta(double angle, double dConrodAngularAcceleration);

double crankpinRadialAcceleration(double angularVelocity);

double forceFromGas(double pressure);
double forceWristpinCentrifugal(double dConrodSlidingAcceleration);
double forceCrankpinRadial(double force, double angle);
double forceCrankpinTangent(double force, double angle);
double forceCrankpinCentrifugal(double angularVelocity);
double forceCrankpin(double dForceCrankpinRadial, double dForceCrankpinTangent, double dForceCrankpinCentrifugal);

int main()
{
	int RPM = 4000; //otáèky n = 4000 RPM
	double RPS = RPM / 60;
	double angularVelocity = RPS * 2 * PI; // rad/sec
	double timeStep;

	double angleDeg;
	double angleRad;
	double angle;
	double time;

	double pressure;

	double dWristpinPosition = wristpinPosition(angle);
	double dWristpinVelocity = wristpinVelocity(angle, angularVelocity);
	double dWristpinAcceleration = wristpinAcceleration(angle, angularVelocity);

	double dConrodSlidingAcceleration = conrodSlidingAcceleration(angle, angularVelocity);
	double dConrodAngularAcceleration = conrodAngularAcceleration(angle, angularVelocity);
	double dConrodMomentumDelta = conrodMomentumDelta(angle, dConrodAngularAcceleration);

	double dCrankpinRadialAcceleration = crankpinRadialAcceleration(angularVelocity);

	double dForceFromGas = forceFromGas(pressure);
	double dForceWristpinCentrifugal = forceWristpinCentrifugal(dConrodSlidingAcceleration);
	double dForceCrankpinRadial = forceCrankpinRadial(dForceFromGas, angle);
	double dForceCrankpinTangent = forceCrankpinTangent(dForceFromGas, angle);
	double dForceCrankpinCentrifugal = forceCrankpinCentrifugal(angularVelocity);
	double dForceCrankpin = forceCrankpin(dForceCrankpinRadial, dForceCrankpinTangent, dForceCrankpinCentrifugal);

	return 0;

	//getline
	//read
}

double wristpinPosition(double angle)
{
	/*
	double root = 1 - lambda * lambda * sin(angle) * sin(angle);
	double xp = halfStroke - halfStroke * cos(angle) + conrodLength - conrodLength * sqrt(root);
	*/

	double x_p = halfStroke * (conrodLength - cos(angle) + lambda / 2 * sin(angle) * sin(angle));
	return x_p;
}

double wristpinVelocity(double angle, double angularVelocity)
{
	/*
	double root = 1 - lambda * lambda * sin(angle) * sin(angle);
	double vp = sin(angle) * (((lambda * lambda* conrodLength*cos(angle))/sqrt(root)) + halfStroke);
	*/

	double v_p = halfStroke * angularVelocity * (sin(angle) - lambda / 2 * (2 * angle));
	return v_p;
}

double wristpinAcceleration(double angle, double angularVelocity)
{
	/*
	double root = pow((1-lambda*lambda*sin(angle)*sin(angle)), (3 / 2));
	double ap = ((lambda * lambda * conrodLength * sin(angle)*sin(angle)*(lambda*lambda*sin(angle)*sin(angle)-1)+lambda*lambda*conrodLength*cos(angle)*cos(angle)+halfStroke*cos(angle)*root)/root);
	*/

	double a_p = halfStroke * angularVelocity * angularVelocity*(cos(angle)+lambda*cos(2*angle));
	return a_p;
}

double conrodSlidingAcceleration(double angle, double angularVelocity)
{
	double a_r = halfStroke * angularVelocity * angularVelocity;
	double a_p = a_r * (cos(angle) + lambda * cos(2 * angle));

	return a_p;
}

double conrodAngularAcceleration(double angle, double angularVelocity)
{
	double epsilon = -lambda * angularVelocity * angularVelocity * ((1 + lambda * lambda / 8) * sin(angle) - (3 / 8) * sin(3 * angle));
	return epsilon;
}


double conrodMomentumDelta(double angle, double dConrodAngularAcceleration)
{
	double compensationMomentum = -conrodInertiaDelta*lambda*((1+(lambda*lambda)/8)*sin(angle)-(3*lambda*lambda*sin(3*angle))/8);
	double dM = conrodInertiaDelta * dConrodAngularAcceleration + compensationMomentum;
	return dM;
}

double crankpinRadialAcceleration(double angularVelocity)
{
	double a_r = halfStroke * angularVelocity * angularVelocity;
	return a_r;
}

double forceFromGas(double pressure)
{
	double force = surface * pressure;
	return force;
}

double forceWristpinCentrifugal(double dConrodSlidingAcceleration)
{
	double force = (pistonMass + conrodSlidingMass) * dConrodSlidingAcceleration;
	return force;
}

double forceCrankpinRadial(double force, double angle)
{
	double P_r = force * (cos(angle) - lambda * sin(angle) * sin(angle));
	return P_r;
}

double forceCrankpinTangent(double force, double angle)
{
	double P_t = force * (sin(angle) + lambda / 2 * sin(2 * angle));
	return P_t;
}

double forceCrankpinCentrifugal(double angularVelocity)
{
	double S_or = conrodRotatingMass * halfStroke * angularVelocity * angularVelocity;
	return S_or;
}


double forceCrankpin(double dForceCrankpinRadial, double dForceCrankpinTangent, double dForceCrankpinCentrifugal)
{
	double P_oL = sqrt((dForceCrankpinRadial + dForceCrankpinCentrifugal) * (dForceCrankpinRadial + dForceCrankpinCentrifugal) + dForceCrankpinTangent * dForceCrankpinTangent);
	return P_oL;
}

