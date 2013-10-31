#include "SamplingLikelihood.h"

SamplingLikelihood::SamplingLikelihood()
{
    numberSamples = 1000;
	numberCycles = 2;
	transform = &SamplingLikelihood::identity;
}

SamplingLikelihood::SamplingLikelihood( double (*_transform)(double) )
{
    numberSamples = 100000;
    numberCycles = 2;
    transform = _transform;
}

SamplingLikelihood::~SamplingLikelihood()
{
}


void SamplingLikelihood::setSamplingParameters(int Samples, int Cycles)
{
	numberSamples = Samples;
	numberCycles = Cycles;
}

/*
 * vec SamplingLikelihood::modelFunction(const vec x) const
{
	vec y(x.size());
	for(int i = 0; i < x.size(); i++)
	{
		y(i) = modelFunction(x(i));
	}
	return y;
}

*/
