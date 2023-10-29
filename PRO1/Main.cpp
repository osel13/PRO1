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

struct angleVsPressure
{

	vector <double> angle;
	vector <double> pressure;

	//pair< vector<double>, vector<double>> pairVectors;
};

struct calculations
{
	double wristpinPosition;
	double wristpinVelocity;
	double wristpinAcceleration;
	double conrodSlidingAcceleration;
	double conrodAngularAcceleration;
	double conrodMomentumDelta;
	double crankpinRadialAcceleration;
	double forceFromGas;
	double forceWristpinCentrifugal;
	double forceCrankpinRadial;
	double forceCrankpinTangent;
	double forceCrankpinCentrifugal;
	double forceCrankpin;
};

using namespace std;

double PI = 3.14159265359;

double bore = 75.5; // vrtání D = 75.5 mm
double stroke = 72.0; // zdvih Z = 72 mm
double surface = PI*bore*bore*0.25; //mm^2
double pistonMass = 0.405; // kg piston_old.CATproduct mass of whole piston

double conrodMass = 0.824; //kg
double conrodInertia = 30000; //kg*m^2
double conrodLength = 150.0; //mm
double conrodLengthB = 45.0; //mm
double conrodLengthA = conrodLength-conrodLengthB;//mm

double angleStepDeg = 0.1;
int numberOfCylinders = 4;

double freqStep = 0.01;
double freqMin = 0;
double frequMax = 1000;

double halfStroke = stroke / 2;
double lambda = halfStroke / conrodLength;
double conrodSlidingMass = (conrodLengthB/conrodLength)*conrodMass;
double conrodRotatingMass = (conrodLengthA/conrodLength)*conrodMass;
double conrodInertiaPrime = conrodRotatingMass*conrodLengthB*conrodLengthB+conrodSlidingMass*conrodLengthA*conrodLengthA;
double conrodInertiaDelta = conrodInertia - conrodInertiaPrime;


//funkce
angleVsPressure readInput();
angleVsPressure recalculate(vector<double> angle, vector<double> pressure);
calculations calculate(double angle, double pressure, double angularVelocity);
vector<double> forcesToMomentum(vector<calculations> calcul);
vector<double> toAllCylinders(vector<double> momentum, int numberOfCylinders, double angleStepDeg);
vector<double> fouriuer(vector<double> res, double freqStep, double freqMin, double frequMax);

void writeResults(vector<calculations> calcul, vector<double> angle, vector<double> pressure, double angularVelocity);

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
	double RPM = 4000.0; //otáèky n = 4000 RPM
	double RPS = RPM/60;
	double angularVelocity = RPS * 2 * PI; // rad/sec


	vector<calculations> calcVector;
	
	angleVsPressure input = readInput();
	vector<double>ia = input.angle;
	vector<double>ip = input.pressure;
	angleVsPressure recalculated = recalculate(ia,ip);
	angleVsPressure linearized = linearize(recalculated.angle, recalculated.pressure, angleStepDeg);




	
	for (int i = 0; i < linearized.angle.size(); i++)
	{
		calculations calc = calculate(linearized.angle[i], linearized.pressure[i], angularVelocity);
		calcVector.push_back(calc);
	}
	
	vector<double> momentum = forcesToMomentum(calcVector);
	vector<double> allCylM = toAllCylinders(momentum, numberOfCylinders, angleStepDeg);
	vector<double> fourier = fouriuer(allCylM, freqStep, freqMin, frequMax);



	//void writeResults(vector<calculations> calcul, vector<double> angle, vector<double> pressure, double angularVelocity);

	return 0;

}

vector<double> forcesToMomentum(vector<calculations> calcul)
{

	vector<double> momentum;
	
	for (int i = 0;i<calcul.size();i++)
	{
		calculations c = calcul[i];
		double mom = c.forceCrankpinTangent * halfStroke;
		momentum.push_back(mom);
	}
	
	return momentum;
}

vector<double> toAllCylinders(vector<double> momentum, int numberOfCylinders, double angleStepDeg)
{
	double angle = 720 / numberOfCylinders;

	
	vector < vector<double>> m;
	//m.push_back(momentum);
	vector<double> res;

	for (int j = 0; j < numberOfCylinders;j++)
	{
		vector<double> mo;
		vector<double> mom;
		vector<double> momTemp;
		//vector<double> 

		for (int i = 0; i < momentum.size(); i++)
		{
			double mm = momentum[i];
			if (i < floor(720-(angleStepDeg * angle * numberOfCylinders)))
			{
				mom.push_back(mm);
			}
			else
			{
				momTemp.push_back(mm);
			}
		}

		for (int j = 0; j < momTemp.size(); j++)
		{
			mo.push_back(momTemp[j]);
		}

		for (int k = 0; k < mom.size(); k++)
		{
			mo.push_back(mom[k]);
		}

		m.push_back(mo);
	}

	for (int l = 0;l<momentum.size();l++)
	{
		double r=0;

		for(int n = 0;n < numberOfCylinders;n++)
		{ 
			r = r + m[l][n];

		}

		res.push_back(r);

	}
	return res;

}

vector<double> fouriuer(vector<double> res,double freqStep,double freqMin, double frequMax)
{
	vector<double> Xr;
	vector<double> Xi;
	vector<double> X;

	vector<double> frequences;
	double frequenceStep = freqStep;
	double frequenceMin = freqMin;
	double frequenceMax = frequMax;
	
	vector<double> fourier;

	for (int j = 0;frequenceMin+(frequenceStep*j)<frequenceMax ;j++)
	{
		double freq = frequenceMin + (frequenceStep * j);
			frequences.push_back(freq);
	}

	for (int k = 0; k < frequences.size(); k++)
	{
		double xr;
		double xi;
		double x;
		for (int i = 0; i < res.size(); i++)
		{

			xr = xr + Xr[i] * cos(2 * PI * frequences[k] * i / res.size()); // ?
			xi = xi + Xi[i] * sin(2 * PI * frequences[k] * i / res.size());
			x = x + sqrt(xr * xr + xi * xi);

		}
		Xi.push_back(xi);
		Xr.push_back(xr);
		X.push_back(x);
	}


	//write file
	fstream frier("fourier.csv");
	for (int write = 1; write < frequences.size(); write++)
	{
		double f = frequences[write];
		double xix = Xi[write];
		double xrx = Xr[write];
		double xxx = X[write];

		frier << f << "," << xix << "," << xrx << "," << xxx << endl;
	}

	frier.close();

	return X;
}


void torsion()
{

}

void writeResults(vector<calculations> calcul, vector<double> angle,  vector<double> pressure, double angularVelocity)
{
	vector<double> angleValues = angle;
	vector<double> pressureValues = pressure;
	

	ofstream outputFile("output.csv");
	outputFile << "Crank angle, Chamber pressure,Wristpin position,Wristpin velocity, Wristpin acceleration,Conrod sliding acceleration, Conrod angular acceleration,Conrod momentum delta, Crankpin radial acceleration, Force from gas, Centrifugal wristpin force, Radial crankpin force,Tangent crankpin force, Centrifugal crankpin force, Total crankpin force " << endl;

	for (int k = 0; k < angleValues.size(); k++)
	{
		calculations calc = calcul[k];
		double angleDeg = angleValues[k];
		outputFile << angleDeg << ",";

		double angle = angleDeg * (PI / 180);

		//double time = 0;
		double pressure = pressureValues[k];
		outputFile << pressure << ",";

		double dWristpinPosition = calc.wristpinPosition;
		outputFile << dWristpinPosition << ",";
		double dWristpinVelocity = calc.wristpinVelocity;
		outputFile << dWristpinVelocity << ",";
		double dWristpinAcceleration = calc.wristpinAcceleration;
		outputFile << dWristpinAcceleration << ",";

		double dConrodSlidingAcceleration = calc.conrodSlidingAcceleration;
		outputFile << dConrodSlidingAcceleration << ",";
		double dConrodAngularAcceleration = calc.conrodAngularAcceleration;
		outputFile << dConrodAngularAcceleration << ",";
		double dConrodMomentumDelta = calc.conrodMomentumDelta;
		outputFile << dConrodMomentumDelta << ",";

		double dCrankpinRadialAcceleration = calc.crankpinRadialAcceleration;
		outputFile << dCrankpinRadialAcceleration << ",";

		double dForceFromGas = calc.forceFromGas;
		outputFile << dForceFromGas << ",";
		double dForceWristpinCentrifugal = calc.forceWristpinCentrifugal;
		outputFile << dForceWristpinCentrifugal << ",";
		double dForceCrankpinRadial = calc.forceCrankpinRadial;
		outputFile << dForceCrankpinRadial << ",";
		double dForceCrankpinTangent = calc.forceCrankpinTangent;
		outputFile << dForceCrankpinTangent << ",";
		double dForceCrankpinCentrifugal = calc.forceCrankpinCentrifugal;
		outputFile << dForceCrankpinCentrifugal << ",";
		double dForceCrankpin = calc.forceCrankpin;
		outputFile << dForceCrankpin << ",";
		outputFile << endl;
	}

	outputFile.close();
}

calculations calculate(double angle, double pressure, double angularVelocity)
{
	double dWristpinPosition = wristpinPosition(angle);
	double dWristpinVelocity = wristpinVelocity(angle, angularVelocity);
	double dWristpinAcceleration = wristpinAcceleration(angle, angularVelocity);
	double dConrodSlidingAcceleration = conrodSlidingAcceleration(angle, angularVelocity);
	double dConrodAngularAcceleration = conrodAngularAcceleration(angle, angularVelocity);
	double dConrodMomentumDelta = conrodMomentumDelta(angle, dConrodAngularAcceleration);
	double dForceFromGas = forceFromGas(pressure);
	double dForceWristpinCentrifugal = forceWristpinCentrifugal(dConrodSlidingAcceleration);
	double dForceCrankpinRadial = forceCrankpinRadial(dForceFromGas, angle);
	double dForceCrankpinTangent = forceCrankpinTangent(dForceFromGas, angle);
	double dForceCrankpinCentrifugal = forceCrankpinCentrifugal(angularVelocity);
	double dForceCrankpin = forceCrankpin(dForceCrankpinRadial, dForceCrankpinTangent, dForceCrankpinCentrifugal);

	calculations calc = { dWristpinPosition, dWristpinVelocity, dWristpinAcceleration, dConrodSlidingAcceleration, dConrodAngularAcceleration, dConrodMomentumDelta, dForceFromGas, dForceWristpinCentrifugal, dForceCrankpinRadial, dForceCrankpinTangent, dForceCrankpinCentrifugal, dForceCrankpin };
	return calc;
}

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
			lineCount++;
			lines.push_back(line);
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
					linePressure = linePressure + lineA[j];
				}
			}


			double dLinePressure = stod(linePressure);
			pressureValues.push_back(dLinePressure);

			double dLineAngle = stod(lineAngle);
			angleValues.push_back(dLineAngle);

		}



	}
	else
	{
		cout << "file  not accessed" << endl;
		
	}

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
			if (angle[i + 1]>0) //první hodnota pøed 0
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
		angl.push_back(720+tempAngle[j]);//merily se dve otacky 4taktu 
		press.push_back(tempPressure[j]);
	}


	angleVsPressure result = {angl,press};
	return result;
}

angleVsPressure linearize(vector<double> angle, vector<double> pressure, double angleStepDeg)
{
	double degree = 0.0;
	vector<double> angl;
	vector<double> press;

	while (degree <= (720 - angleStepDeg));
	{
		angl.push_back(degree);
		for (int i = 0; i < angle.size();)
		{
			if (angle[i]<degree && angle [i+1] > degree)
			{
				double deg = angle[i];
				double degPlusOne = angle[i + 1];
				double pre = pressure[i];
				double prePlusOne = pressure[i + 1];
				double p;

				p = pre + ((degree - deg) / (degPlusOne - deg)) * (prePlusOne - pre);
				press.push_back(p);


			}
			else
			{
				continue;
			}
			
		}
		degree = degree + angleStepDeg;
	}

	angleVsPressure result = { angl,press };
	return result;
}

double wristpinPosition(double angle)
{
	/*
	double root = 1 - lambda * lambda * sin(angle) * sin(angle);
	double xp = halfStroke - halfStroke * cos(angle) + conrodLength - conrodLength * sqrt(root);
	*/

	double x_p = bore - (halfStroke * (1 - cos(angle) + (lambda / 2) * sin(angle) * sin(angle)));
	return x_p;
}

double wristpinVelocity(double angle, double angularVelocity)
{
	/*
	double root = 1 - lambda * lambda * sin(angle) * sin(angle);
	double vp = sin(angle) * (((lambda * lambda* conrodLength*cos(angle))/sqrt(root)) + halfStroke);
	*/

	double v_p = halfStroke * angularVelocity * (sin(angle) + lambda*0.5*sin(2 * angle));
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

