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
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include "Kinematics.h"
#include "pistonDynamics.h"
#include "wristpinDynamics.h"
#include "crankpinDynamics.h"
#include "Momentums.h"
#include "Fourier.h"


using namespace std;

struct angleVsPressure
{

	vector <double> angle;
	vector <double> pressure;

};

//double PI = 3.14159265359;

double bore = 75.5; // vrtání D = 75.5 mm
double stroke = 72.0; // zdvih Z = 72 mm
double surface = 3.14159 * bore * bore * 0.25; //mm^2
double pistonMass = 0.405; // kg piston_old.CATproduct mass of whole piston

double pressureAtmospheric = 101325.0; //Pa, Temperature, elevation dependent

double conrodMass = 0.824; //kg
double conrodInertia = 30000; //kg*mm^2
double conrodLength = 150.0; //mm
double conrodLengthB = 45.0; //mm
double conrodLengthA = conrodLength - conrodLengthB;//mm

double angleStepDeg = 0.5;
int numberOfCylinders = 4;

double freqStep = 0.1;
double freqMin = 0;
double frequMax = 10;

double halfStroke = stroke / 2;
double lambda = halfStroke / conrodLength;
double conrodSlidingMass = (conrodInertia - conrodMass * conrodLengthB * conrodLengthB) / (conrodLengthA * conrodLengthA - conrodLengthB * conrodLengthB);
double conrodRotatingMass = conrodMass - conrodSlidingMass;



angleVsPressure readInput();
angleVsPressure recalculate(vector<double> angle, vector<double> pressure);
angleVsPressure linearize(vector<double> angle, vector<double> pressure, double angleStepDeg);

//void writeResults(vector<calculations> calcul, vector<double> angle, vector<double> pressure);

//str 14,50
int main()
{
	double RPM = 4000.0; //otáèky n = 4000 RPM
	double RPS = RPM / 60;
	double angularVelocity = RPS * 2 * 3.14159265359; // rad/sec



	
	angleVsPressure input = readInput();
	cout << "Input Read" << endl;
	angleVsPressure recalculated = recalculate(input.angle, input.pressure);
	cout << "Recalculated" << endl;
	angleVsPressure linearized = linearize(recalculated.angle, recalculated.pressure, angleStepDeg);
	cout << "Linearized" << endl;

	Kinematics k;
	std::vector<double> pistonPosition;
	std::vector<double> pistonVelocity;
	std::vector<double> pistonAcceleration;

	for (int i = 0; i < linearized.angle.size(); i++)
	{
		pistonPosition.push_back(k.calculatePistonPosition(linearized.angle[i], halfStroke, conrodLength));
		pistonVelocity.push_back(k.calculatePistonVelocity(RPM, linearized.angle[i], halfStroke, conrodLength));
		pistonAcceleration.push_back(k.calculatePistonAcceleration(RPM, linearized.angle[i], halfStroke, conrodLength));

	}

	cout << "Kinematics loaded" << endl;
	
	//k.debugPrint();
	
	pistonDynamics pD;
	std::vector<double> pistonPressureForce;
	std::vector<double> pistonInertialForce;
	std::vector<double> pistonTotalForce;
	
	for (int j = 0; j < linearized.angle.size(); j++)
	{
		pistonPressureForce.push_back(pD.calculatePistonPressureForce(linearized.pressure[j],surface,pressureAtmospheric));
		pistonInertialForce.push_back(pD.calculatePistonInertia(pistonMass,pistonAcceleration[j]));
		pistonTotalForce.push_back(pD.calculatePistonTotalForce(pistonPressureForce[j], pistonInertialForce[j]));

	}

	//pD.debugPrint();
	
	cout << "PistonDynamics loaded" << endl;

	wristpinDynamics wD;
	std::vector<double> pistonSideForce;
	std::vector<double> forceAlongConrod;
	
	for (int k = 0; k < linearized.angle.size(); k++)
	{
		pistonSideForce.push_back(wD.calculatePistonSideForce(pistonTotalForce[k],linearized.angle[k],halfStroke,conrodLength));
		forceAlongConrod.push_back(wD.calculateAlongConrodForce(pistonTotalForce[k], linearized.angle[k], halfStroke, conrodLength));
	}

	cout << "wristpinDynamics loaded" << endl;
	
	//wD.debugPrint();

	crankpinDynamics cD;
	vector<double> CrankpinInertia;
	vector<double> AlongCylinderTotalForce;
	vector<double> AlongConrodTotalForce;
	vector<double> NormalForce;
	vector<double> ConrodRadialForce;
	vector<double> ConrodTangentialForce;
	vector<double> ConrodRotationalInertia;
	vector<double> TotalRadialForce;

	//	cD.debugPrint(pistonAcceleration, linearized.pressure, linearized.angle, pistonMass, conrodSlidingMass, conrodRotatingMass, halfStroke, conrodLength, RPM);

	for (int l = 0; l < linearized.angle.size(); l++)
	{
		CrankpinInertia.push_back(cD.calculateCrankpinInertia(pistonMass, conrodSlidingMass, pistonAcceleration[l]));
		AlongCylinderTotalForce.push_back(cD.calculateAlongCylinderTotalForce(pistonPressureForce[l], CrankpinInertia[l]));
		AlongConrodTotalForce.push_back(cD.calculateAlongConrodTotalForce(AlongCylinderTotalForce[l], linearized.angle[l], halfStroke, conrodLength));
		NormalForce.push_back(cD.calculateNormalForce(AlongConrodTotalForce[l], linearized.angle[l], halfStroke, conrodLength));
		ConrodRadialForce.push_back(cD.calculateConrodRadialForce(AlongConrodTotalForce[l], linearized.angle[l],  halfStroke,  conrodLength));
		ConrodTangentialForce.push_back(cD.calculateConrodTangentialForce(AlongConrodTotalForce[l], linearized.angle[l], halfStroke, conrodLength));
		ConrodRotationalInertia.push_back(cD.calculateConrodRotationalInertia(conrodRotatingMass, halfStroke, RPM));
		TotalRadialForce.push_back(cD.calculateTotalRadialForce( ConrodRadialForce[l], ConrodRotationalInertia[l]));
	}

	cout << "crankpinDynamics loaded" << endl;


	Momentums m;

	vector<double>momentumSum = m.Sum(ConrodTangentialForce, linearized.angle, numberOfCylinders, halfStroke);
	//m.DebugPrint(ConrodTangentialForce, linearized.angle, halfStroke);
	double momentumAverage = m.Average(ConrodTangentialForce, linearized.angle, numberOfCylinders, halfStroke);
	cout << momentumAverage << endl;

	cout << "Momentums loaded" << endl;

	Fourier f;
	int freqSpan = 1000;
	//f.debugPrint(momentumSum, 1000);
	std::vector<Fourier::Res> Four = f.dFT(momentumSum, freqSpan);
	std::vector<double> mag;
	std::vector<double> real;
	std::vector<double> img;

	for (int n = 0; n < Four.size(); n++)
	{
		real.push_back(Four[n].real);
		img.push_back(Four[n].img);
		double root = sqrt(pow(Four[n].real, 2) + pow(Four[n].img, 2));
		mag.push_back(root);
	}


	cout << "cFourier Transformed" << endl;



	return 0;

}

/*


vector<double> fouriuer(vector<double> res, double freqStep, double freqMin, double frequMax)
{
	vector<double> Xr;
	vector<double> Xi;
	vector<double> X;

	vector<double> frequences;
	double frequenceStep = freqStep;
	double frequenceMin = freqMin;
	double frequenceMax = frequMax;

	vector<double> fourier;

	for (int j = 0; frequenceMin + (frequenceStep * j) < frequenceMax; j++)
	{
		double freq = frequenceMin * (3.14159 / 180) + (frequenceStep * (3.14159 / 180) * j);
		frequences.push_back(freq);
	}



	for (int k = 0; k < frequences.size(); k++)
	{

		double xr = 0;
		double xi = 0;
		double x = 0;

		for (int i = 0; i < res.size(); i++)
		{
			xr = xr + res[i] * cos(2 * 3.14159 * frequences[k] * i / res.size()); // ?
			xi = xi + res[i] * sin(2 * 3.14159 * frequences[k] * i / res.size());
			//x = x + sqrt((xr * xr) + (xi * xi));

		}
		x = x + sqrt((xr * xr) + (xi * xi));
		Xi.push_back(xi);
		Xr.push_back(xr);
		X.push_back(x);
	}


	//write file
	ofstream frier("fourier.csv");
	for (int write = 1; write < frequences.size(); write++)
	{
		double f = frequences[write] * 180 / 3.14159;
		double xix = Xi[write];
		double xrx = Xr[write];
		double xxx = X[write];

		frier << f << "," << xix << "," << xrx << "," << xxx << endl;
	}

	frier.close();

	return X;
}

*/

angleVsPressure readInput()
{
	int lineCount = 0;
	string line;
	ifstream inputFile("4000.txt");
	vector<string> lines;
	vector<double> pressureValues;
	vector<double> angleValues;

	if (inputFile.is_open())
	{
		//get number of lines + content

		//cout << "file accessed" << endl;
		while (getline(inputFile, line))
		{

			lines.push_back(line);
			lineCount = lineCount + 1;
		}

		double dLineAngle;
		double dLinePressure;
		for (int i = 1; i < lineCount; i++)
		{
			string lineA = lines[i];
			bool bAngle = true;
			string lineAngle = "";
			string linePressure = "";
			for (int j = 0; j < lineA.size(); j++)
			{
				if (lineA[j] == '\t')
				{
					bAngle = false;
					continue;
				}

				if (bAngle)
				{
					lineAngle = lineAngle + lineA[j];
				}
				else
				{
					linePressure = linePressure + lineA[j]; //from bar to pa
				}
			}


			dLinePressure = stod(linePressure) *100000;
			pressureValues.push_back(dLinePressure);

			dLineAngle = stod(lineAngle);
			angleValues.push_back(dLineAngle);

		}




	}
	else
	{
		cout << "file  not accessed" << endl;

	}

	//cout << angleValues[angleValues.size()-1] << endl;
	inputFile.close();

	angleVsPressure result = { angleValues, pressureValues };

	return result;
}

angleVsPressure recalculate(vector<double> angle, vector<double> pressure)
{
	vector<double> tempAngle;
	vector<double> tempPressure;

	vector<double> angl;
	vector<double> press;

	for (int i = 0; i < angle.size(); i++)
	{

		if (angle[i] < 0)
		{
			if (angle[i + 1] > 0) //první hodnota pøed 0
			{
				angl.push_back(angle[i]);
				press.push_back(pressure[i]);
			}
			else
			{
				tempAngle.push_back(angle[i]);
				tempPressure.push_back(pressure[i]);
			}
		}
		else
		{
			angl.push_back(angle[i]);
			press.push_back(pressure[i]);
		}
	}

	for (int j = 0; j < tempAngle.size(); j++)
	{
		angl.push_back(720 + tempAngle[j]);//merily se dve otacky 4taktu 
		press.push_back(tempPressure[j]);
	}

	//cout << angl[angl.size() - 1] << endl;

	angleVsPressure result = { angl,press };
	return result;
}

angleVsPressure linearize(vector<double> angle, vector<double> pressure, double angleStepDeg) //poèty vektorù
{
	double degree = 0.0;
	vector<double> angl;
	vector<double> press;


	while (degree < angle[angle.size()-1])
	{

		for (int i = 0; i < angle.size(); i++)
		{
			if (i < 1)
			{
				//angl.push_back(degree);
			}


			if (angle[i]<degree && angle[i + 1] > degree)
			{
				double deg = angle[i];
				double degPlusOne = angle[i + 1];
				double pre = pressure[i];
				double prePlusOne = pressure[i + 1];
				double p;

				p = pre + ((degree - deg) / (degPlusOne - deg)) * (prePlusOne - pre);
				press.push_back(p);
				angl.push_back(degree);

			}
			else
			{
				continue;
				cout << "continued" << endl;
			}

		}
		degree = degree + angleStepDeg;

		

	}
	//angl.push_back(degree);

	angleVsPressure result = { angl,press };
	return result;
}