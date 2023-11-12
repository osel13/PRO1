#pragma once
#include "Fourier.h"
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>


std::vector<Fourier::Res> Fourier::dFT(std::vector<double> momentSum, int freqSpan)
{
	//Fourier f;
	int N = momentSum.size();
	int K = freqSpan;

	Fourier::Res intSum;

	std::vector<Fourier::Res> output;
	output.reserve(K);

	for (int k = 0; k < K; k++)
	{
		intSum = { 0, 0 };
		for (int n = 0; n < N; n++)
		{
			double realPart = cos(((2 * 3.14159) / N) * k * n);
			double imagPart = sin(((2 * 3.14159) / N) * k * n);
			Fourier::Res w = { realPart,-imagPart };
			intSum.real += momentSum[n] * w.real;
			intSum.img += momentSum[n] * w.img;
		}
		output.push_back(intSum);
	}
	return output;
}


void Fourier::debugPrint(std::vector<double> momentumSum,int freqSpan)
{
	Fourier f;
	std::vector<Fourier::Res> Four = f.dFT(momentumSum, freqSpan);
	std::vector<double> mag;
	std::vector<double> real;
	std::vector<double> img;

	for (int i = 0; i < Four.size(); i++)
	{
		real.push_back(Four[i].real);
		img.push_back(Four[i].img);
		double root = sqrt(pow(Four[i].real, 2) + pow(Four[i].img, 2));
		mag.push_back(root);
	}




	std::ofstream Debug;
	Debug.open("FourierDebug.csv");
	Debug << "frequency, real, imaginary, magnitude" << std::endl;
	for (int j = 0; j < Four.size(); j++)
	{
		Debug << j << ",";
		Debug << Four[j].real << ",";
		Debug << Four[j].img << ",";
		Debug << mag[j] <<std::endl;

	}
	Debug.close();
}
