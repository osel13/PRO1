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
using namespace std;

struct angleVsPressure
{

vector <double> angle;
vector <double> pressure;

};

struct calculations
{
	double wristpinPosition;
	double wristpinVelocity;
	double wristpinAcceleration;
	double forcePressure;
	double forcePressureConrod;
	double forcePressureWall;
	double forcePressureRadial;
	double forcePressureTangent;
	double forceInertiaPiston;
	double forceInertiaConrod;
	double forceInertiaWall;
	double forceInertiaRadial;
	double forceInertiaTangent;
};



double PI = 3.14159265359;

double bore = 75.5; // vrtání D = 75.5 mm
double stroke = 72.0; // zdvih Z = 72 mm
double surface = PI*bore*bore*0.25; //mm^2
double pistonMass = 0.405; // kg piston_old.CATproduct mass of whole piston

double conrodMass = 0.824; //kg
double conrodInertia = 30000; //kg*mm^2
double conrodLength = 150.0; //mm
double conrodLengthB = 45.0; //mm
double conrodLengthA = conrodLength-conrodLengthB;//mm

double angleStepDeg = 0.5;
int numberOfCylinders = 4;

double freqStep = 0.1;
double freqMin = 0;
double frequMax = 10;

double halfStroke = stroke / 2;
double lambda = halfStroke / conrodLength;
double conrodSlidingMass = (conrodInertia-conrodMass*conrodLengthB*conrodLengthB)/(conrodLengthA*conrodLengthA-conrodLengthB*conrodLengthB);
double conrodRotatingMass = conrodMass - conrodSlidingMass;



//funkce
angleVsPressure readInput();

angleVsPressure recalculate(vector<double> angle, vector<double> pressure);
angleVsPressure linearize(vector<double> angle, vector<double> pressure, double angleStepDeg);
calculations calculate(double angle, double pressure, double angularVelocity);
vector<double> forcesToMomentum(vector<calculations> calcul);
vector<double> toAllCylinders(vector<double> momentum, int numberOfCylinders, double angleStepDeg);
vector<double> fouriuer(vector<double> res, double freqStep, double freqMin, double frequMax);

void writeResults(vector<calculations> calcul, vector<double> angle, vector<double> pressure);

double wristpinPosition(double angle);
double wristpinVelocity(double angle, double angularVelocity);
double wristpinAcceleration(double angle, double angularVelocity);

//primární síly
double forcePressure(double pressure);
double forcePressureConrod(double forcePressure, double angle );
double forcePressureWall(double forcePressure, double angle);
double forcePressureRadial(double forcePressureConrod, double angle);
double forcePressureTangent(double forcePressureConrod, double angle);

//sekundární síly
double forceInertiaPiston(double wristpinAcceleration);
double forceInertiaConrod(double forceInertiaPiston, double angle);
double forceInertiaWall(double forceInertiaPiston, double angle);
double forceInertiaRadial(double forceInertiaConrod, double angle);
double forceInertiaTangent(double forceInertiaConrod, double angle);
//double forceCentrifugal(double angularVelocity);

//str 14,50
int main()
{
	double RPM = 4000.0; //otáèky n = 4000 RPM
	double RPS = RPM/60;
	double angularVelocity = RPS * 2 * PI; // rad/sec


	
	
	angleVsPressure input = readInput();
	cout << "Input Read" << endl;
	angleVsPressure recalculated = recalculate(input.angle, input.pressure);
	cout << "Recalculated" << endl;
	angleVsPressure linearized = linearize(recalculated.angle, recalculated.pressure, angleStepDeg);
	cout << "Linearized" << endl;

	///Tady je problém, pøetejká vektor

	vector<calculations> calcVector;
	
	for (int i = 0; i < linearized.angle.size(); i++) //velikost vektoru "-1"
	{
		calculations calc = calculate(linearized.angle[i], linearized.pressure[i], angularVelocity);
		calcVector.push_back(calc);
	}

	cout << "for success" << endl;


	writeResults(calcVector, linearized.angle, linearized.pressure);
	cout << "results written" << endl;

	vector<double> momentum = forcesToMomentum(calcVector);
	cout << "Momentum calculated" << endl;
	vector<double> allCylM = toAllCylinders(momentum, numberOfCylinders, angleStepDeg);
	cout << "cylinders multiplied" << endl;
	
	vector<double> omed;
	double f;
	for (int demo = 0; demo < 1000; demo++)
	{
		f = sin(2 * demo*PI/180) + sin(3 * demo* PI / 180);
		omed.push_back(f);
	}

	vector<double> fourier = fouriuer(omed, freqStep, freqMin, frequMax);
	cout << "fourier calculated" << endl;



	//void writeResults(vector<calculations> calcul, vector<double> angle, vector<double> pressure, double angularVelocity);
	return 0;
	

}

vector<double> forcesToMomentum(vector<calculations> calcul)
{

	vector<double> momentum;
	
	for (int i = 0;i<calcul.size();i++)
	{
		calculations c = calcul[i];
		double mom = (c.forcePressureRadial-c.forceInertiaRadial) * halfStroke;
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

	for (int l = 0;l<m.size();l++)
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
		double freq = frequenceMin * (PI / 180) + (frequenceStep * (PI / 180) * j);
			frequences.push_back(freq);
	}



	for (int k = 0; k < frequences.size(); k++)
	{

		double xr = 0;
		double xi = 0;
		double x = 0;

		for (int i = 0; i < res.size(); i++)
		{
				xr = xr + res[i] * cos(2 * PI * frequences[k] * i / res.size()); // ?
				xi = xi + res[i] * sin(2 * PI * frequences[k] * i / res.size());
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
		double f = frequences[write]*180/PI;
		double xix = Xi[write];
		double xrx = Xr[write];
		double xxx = X[write];

		frier << f << "," << xix << "," << xrx << "," << xxx << endl;
	}

	frier.close();

	return X;
}

/*
void torsion()
{

}
*/

void writeResults(vector<calculations> calcul, vector<double> angle,  vector<double> pressure)
{
	vector<double> angleValues = angle;
	vector<double> pressureValues = pressure;
	

	ofstream outputFile("output.csv");
	outputFile << "Crank angle," ;   
	outputFile << "Chamber pressure,";
	outputFile << "Wristpin position,";
	outputFile << "Wristpin velocity,";
	outputFile << "Wristpin acceleration,";
	outputFile << "dForcePressure,";
	outputFile << "dForcePressureConrod,";
	outputFile << "dForcePressureWall,";
	outputFile << "dForcePressureRadial,";
	outputFile << "dForcePressureTangent,";
	outputFile << "dForceInertiaPiston,";
	outputFile << "dForceInertiaConrod,";
	outputFile << "dForceInertiaWall,";
	outputFile << "dForceInertiaRadial,";
	outputFile << "dforceInertiaTangent" << endl;
		

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

		double dForcePressure = calc.forcePressure;
		outputFile << dForcePressure << ",";
		double dForcePressureConrod = calc.forcePressureConrod;
		outputFile << dForcePressureConrod << ",";
		double dForcePressureWall = calc.forcePressureWall;
		outputFile << dForcePressureWall << ",";
		double dForcePressureRadial = calc.forcePressureRadial;
		outputFile << dForcePressureRadial << ",";
		double dForcePressureTangent = calc.forcePressureTangent;
		outputFile << dForcePressureTangent << ",";
		double dForceInertiaPiston = calc.forceInertiaPiston;
		outputFile << dForceInertiaPiston << ",";
		double dForceInertiaConrod = calc.forceInertiaConrod;
		outputFile << dForceInertiaConrod << ",";
		double dForceInertiaWall = calc.forceInertiaWall;
		outputFile << dForceInertiaWall << ",";
		double dForceInertiaRadial = calc.forceInertiaRadial;
		outputFile << dForceInertiaRadial << ",";
		double dForceInertiaTangent = calc.forceInertiaTangent;
		outputFile << dForceInertiaTangent << ",";

		outputFile << endl;
	}

	outputFile.close();
}

calculations calculate(double angle, double pressure, double angularVelocity)
{
	double dWristpinPosition = wristpinPosition(angle);
	double dWristpinVelocity = wristpinVelocity(angle, angularVelocity);
	double dWristpinAcceleration = wristpinAcceleration(angle, angularVelocity);

	//primární síly
	double dForcePressure = forcePressure(pressure);
	double dForcePressureConrod = forcePressureConrod(dForcePressure, angle);
	double dForcePressureWall = forcePressureWall( dForcePressure, angle);
	double dForcePressureRadial = forcePressureRadial(dForcePressureConrod,angle);
	double dForcePressureTangent = forcePressureTangent(dForcePressureConrod, angle);

	//sekundární síly
	double dForceInertiaPiston = forceInertiaPiston(dWristpinAcceleration);
	double dForceInertiaConrod = forceInertiaConrod(dForceInertiaPiston, angle);
	double dForceInertiaWall = forceInertiaWall(dForceInertiaPiston, angle);
	double dForceInertiaRadial = forceInertiaRadial(dForceInertiaConrod, angle);
	double dforceInertiaTangent = forceInertiaTangent(dForceInertiaConrod, angle);
	//double forceCentrifugal(double angularVelocity);

	calculations calc = { 
		dWristpinPosition, 
		dWristpinVelocity, 
		dWristpinAcceleration, 
		dForcePressure,
		dForcePressureConrod, 
		dForcePressureWall, 
		dForcePressureRadial,
		dForcePressureTangent,
		dForceInertiaPiston,
		dForceInertiaConrod, 
		dForceInertiaWall, 
		dForceInertiaRadial,
		dforceInertiaTangent };
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
					linePressure = linePressure + lineA[j];
				}
			}


			dLinePressure = stod(linePressure);
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

	//cout << angl[angl.size() - 1] << endl;

	angleVsPressure result = {angl,press};
	return result;
}

angleVsPressure linearize(vector<double> angle, vector<double> pressure, double angleStepDeg)
{
	double degree = 0.0;
	vector<double> angl;
	vector<double> press;
	

	while(degree < angle[angle.size()-1])
	{
		
		for (int i = 0; i < angle.size()-1;i++)
		{	
			if (i < 1)
			{
				angl.push_back(degree);
			}

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
				cout << "continued" << endl;
			}
			
		}
		degree = degree + angleStepDeg;
		angl.push_back(degree);
		
	}

	angleVsPressure result = { angl,press };
	return result;
}

double wristpinPosition(double degrees)
{
	double angle = degrees * (PI / 180);
	double x_p = halfStroke*cos(angle) + sqrt((conrodLength*conrodLength)-(halfStroke*sin(angle)*halfStroke*sin(angle))); // viz diplomka
	return x_p;
}

double wristpinVelocity(double degrees, double angularVelocity)
{
	double angle = degrees * (PI / 180);
	double root(sqrt((conrodLength * conrodLength) - (halfStroke * sin(angle) * halfStroke * sin(angle))));
	double brace = halfStroke * sin(angle) * (((halfStroke * cos(angle)) / (root)) - 1);
	double v_p = angularVelocity * brace;
	return v_p;
}

double wristpinAcceleration(double degrees, double angularVelocity)
{
	double angle = degrees * (PI / 180);
	double root(sqrt((conrodLength * conrodLength) - (halfStroke * sin(angle) * halfStroke * sin(angle))));
	double brace = halfStroke * sin(angle) * (((halfStroke * cos(angle)) / (root)) );
	double middle = (pow(halfStroke, 3) * sin(angle) * pow(cos(angle), 2)) / (pow(conrodLength * conrodLength - halfStroke * halfStroke * sin(angle) * sin(angle), 3 / 2));
	double a = halfStroke*cos(angle)*(brace-1) + halfStroke*sin(angle)*(middle-brace);
	double a_p = angularVelocity * a;
	return a_p;
}


double forcePressure(double pressure)
{
	double Fp = pressure * surface;
	return Fp;
}

double forcePressureConrod(double forcePressure, double angle)
{
	double angleRad = angle * PI / 180;
	double beta = sqrt(1-lambda*lambda*sin(angleRad)*sin(angleRad));
	double Fpc = forcePressure / cos(beta);
	return Fpc;
}

double forcePressureWall(double forcePressure, double angle)
{
	double angleRad = angle * PI / 180;
	double beta = sqrt(1 - lambda * lambda * sin(angleRad) * sin(angleRad));
	double Fpn = forcePressure * tan(beta);
	return Fpn;
}

double forcePressureRadial(double forcePressureConrod, double angle)
{
	double angleRad = angle * PI / 180;
	double beta = sqrt(1 - lambda * lambda * sin(angleRad) * sin(angleRad));
	double Fpr = forcePressureConrod * cos(angleRad + beta);
	return Fpr;
}

double forcePressureTangent(double forcePressureConrod, double angle)
{
	double angleRad = angle * PI / 180;
	double beta = sqrt(1 - lambda * lambda * sin(angleRad) * sin(angleRad));
	double Fpt = forcePressureConrod * sin(angleRad + beta);
	return Fpt;
}

//sekundární síly
double forceInertiaPiston(double wristpinAcceleration)
{
	double Fip = (pistonMass + conrodSlidingMass) * wristpinAcceleration;
	return Fip;
}
double forceInertiaConrod(double forceInertiaPiston, double angle)
{
	double angleRad = angle * PI / 180;
	double beta = sqrt(1 - lambda * lambda * sin(angleRad) * sin(angleRad));
	double Fic = forceInertiaPiston / cos(beta);
	return Fic;
}

double forceInertiaWall(double forceInertiaPiston, double angle)
{
	double angleRad = angle * PI / 180;
	double beta = sqrt(1 - lambda * lambda * sin(angleRad) * sin(angleRad));
	double Fin = forceInertiaPiston * tan(beta);
	return Fin;
}
double forceInertiaRadial(double forceInertiaConrod, double angle)
{
	double angleRad = angle * PI / 180;
	double beta = sqrt(1 - lambda * lambda * sin(angleRad) * sin(angleRad));
	double Fir = forceInertiaConrod * cos(angleRad + beta);
	return Fir;
}
double forceInertiaTangent(double forceInertiaConrod, double angle)
{
	double angleRad = angle * PI / 180;
	double beta = sqrt(1 - lambda * lambda * sin(angleRad) * sin(angleRad));
	double Fit = forceInertiaConrod * sin(angleRad + beta);
	return Fit;
}

/*
double forceCentrifugal(double angularVelocity)
{
	double Fc(conrodRotationalMass + crankshaftMass)*halfStroke*angularVelocity*angularVelocity;
	return Fc;
}

*/