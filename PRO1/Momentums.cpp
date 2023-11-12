#pragma once
#include "Momentums.h"
#include <vector>
#include <fstream>

std::vector<double> Momentums::momentumToNthCylinder(std::vector<double> tangentialForces, std::vector<double> angleDeg,  int cylinderNumber, int numberOfCylinders, double halfStroke)
{
    std::vector<double> result;
    std::vector<double> temp;
    double maxAngle = 720.0;
    double splitAngle = (maxAngle / numberOfCylinders) * cylinderNumber;

    for (int i=0; i<angleDeg.size(); i++)
    {
        double momentum = tangentialForces[i] * halfStroke;
        if (angleDeg[i] < splitAngle)
        {
            temp.push_back(momentum);
        }
        else
        {
            result.push_back(momentum);
        }
    }

    for (int j = 0; j < temp.size(); j++)
    {
        result.push_back(temp[j]);
    }
    return result;
}

/*
double Momentums::inertialForcesToNthCylinderFirstOrder(double RPM, double angleDeg, int numberOfCylinders, int cylinderNumber, double pistonMass, double conrodSlidingMass, double halfStroke)
{
    double maxAngle = 720.0;
    double angularVelocity = (RPM / 60) * 2 * 3.14159;
    double angleRad = angleDeg * 3.14159 / 180;
    double splitAngleRad = ((maxAngle / numberOfCylinders) * cylinderNumber) * 3.14159 / 180;
    double P1 = (pistonMass + conrodSlidingMass) * halfStroke * pow(angularVelocity, 2) * (cos(angleRad + splitAngleRad)); // pøedloha divná
    return P1;
}

double Momentums::inertialForcesToNthCylinderSecondOrder(double RPM, double angleDeg, int numberOfCylinders, int cylinderNumber, double pistonMass, double conrodSlidingMass, double halfStroke, double conrodLength)
{
    double maxAngle = 720.0;
    double angularVelocity = (RPM / 60) * 2 * 3.14159;
    double angleRad = angleDeg * 3.14159 / 180;
    double splitAngleRad = ((maxAngle / numberOfCylinders) * cylinderNumber) * 3.14159 / 180;
    double lambda = halfStroke / conrodLength;
    double P2 = lambda * (pistonMass + conrodSlidingMass) * halfStroke * pow(angularVelocity, 2) * cos(2 * angleRad + splitAngleRad);
    return P2;
}


double Momentums::momentumToNthCylinderFirstOrder(double bore, double RPM, double angleDeg, int numberOfCylinders, int cylinderNumber, double pistonMass, double conrodSlidingMass, double halfStroke)
{
    double b = bore + 5; //rozteè mezi válci
    double maxAngle = 720.0;
    double angularVelocity = (RPM / 60) * 2 * 3.14159;
    double angleRad = angleDeg * 3.14159 / 180;
    double splitAngleRad = ((maxAngle / numberOfCylinders) * cylinderNumber) * 3.14159 / 180;
    double M1 = (pistonMass + conrodSlidingMass) * halfStroke * pow(angularVelocity, 2) * (cos(angleRad + splitAngleRad));
    return M1;
}

double Momentums::momentumToNthCylinderwSecondOrder()
{
    return 0.0;
}
*/

void Momentums::DebugPrint(std::vector<double> tangentialForces, std::vector<double> angleDeg, /*int cylinderNumber, int numberOfCylinders,*/ double halfStroke)
{
    Momentums m;
    std::vector<double> firstCyl;
    std::vector<double> secondCyl;
    std::vector<double> thirdCyl;
    std::vector<double> fourthCyl;
    std::ofstream Debug;
   
    firstCyl = m.momentumToNthCylinder(tangentialForces, angleDeg, 0, 4, halfStroke);
    secondCyl = m.momentumToNthCylinder(tangentialForces, angleDeg, 1, 4, halfStroke);
    thirdCyl = m.momentumToNthCylinder(tangentialForces, angleDeg, 2, 4, halfStroke);
    fourthCyl = m.momentumToNthCylinder(tangentialForces, angleDeg, 3, 4, halfStroke);

    Debug.open("MomentumsDebug.csv");
    for (int i = 0; i < angleDeg.size(); i++)
    {
        double sum = (firstCyl[i] + secondCyl[i] + thirdCyl[i] + fourthCyl[i]);
        Debug << angleDeg[i] << ",";
        Debug << firstCyl[i] << ",";
        Debug << secondCyl[i] << ",";
        Debug << thirdCyl[i] << ",";
        Debug << fourthCyl[i] << ",";
        Debug << sum << std::endl;

    }

    Debug.close();
}

std::vector<double> Momentums::Sum(std::vector<double> tangentialForces, std::vector<double> angleDeg,int numberOfCylinders, double halfStroke)
{
    Momentums m;
    std::vector<std::vector<double>> momentumToCylinders;
    std::vector<double> temp;
    std::vector<double> result;
    temp.clear();
    for (int i = 0; i < numberOfCylinders; i++)
    {
        temp = m.momentumToNthCylinder(tangentialForces, angleDeg, i, numberOfCylinders, halfStroke);
        momentumToCylinders.push_back(temp);
    }


    for (int j = 0; j < angleDeg.size(); j++)
    {
        double dTemp =0;
        for (int k = 0; k < momentumToCylinders.size(); k++)
        {
            dTemp = dTemp + momentumToCylinders[k][j];
        }
        result.push_back(dTemp);
    }

    return result;
}

double Momentums::Average(std::vector<double> tangentialForces, std::vector<double> angleDeg, int numberOfCylinders, double halfStroke) //weird
{
    Momentums m;
    std::vector<std::vector<double>> momentumToCylinders;
    std::vector<double> temp;
    std::vector<double> tTemp;
    
    temp.clear();
    for (int i = 0; i < numberOfCylinders; i++)
    {
        temp = m.momentumToNthCylinder(tangentialForces, angleDeg, i, numberOfCylinders, halfStroke);
        momentumToCylinders.push_back(temp);
    }


    for (int j = 0; j < angleDeg.size(); j++)
    {
        double dTemp = 0;
        
        for (int k = 0; k < momentumToCylinders.size(); k++)
        {
            dTemp = dTemp + momentumToCylinders[k][j];
        }
        tTemp.push_back(dTemp/ momentumToCylinders.size());
    }
    double result = 0;
    for (int l = 0; l < angleDeg.size(); l++)
    {
        result = result + tTemp[l];
    }

    return result/ angleDeg.size();
}