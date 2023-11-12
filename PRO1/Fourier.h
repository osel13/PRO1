#pragma once
#include <vector>
class Fourier
{
public:
	struct Res
	{
		double real;
		double img;
	};

	std::vector<Fourier::Res> dFT(std::vector<double> momentSum, int freqSpan);

	void debugPrint(std::vector<double> momentumSum,int FreqSpan);
};

