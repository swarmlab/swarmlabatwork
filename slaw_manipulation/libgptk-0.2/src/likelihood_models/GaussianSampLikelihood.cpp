#include "GaussianSampLikelihood.h"

#include <cmath>

using namespace std;
using namespace itpp;

GaussianSampLikelihood::GaussianSampLikelihood(double Mean, double Variance)
{
	likelihoodMean = Mean;
	likelihoodVariance = Variance;
}

GaussianSampLikelihood::GaussianSampLikelihood(double Mean, double Variance, double (*_transform)(double))
{
    likelihoodMean = Mean;
    likelihoodVariance = Variance;
    transform = _transform;
}

GaussianSampLikelihood::~GaussianSampLikelihood()
{
}

// TODO  we need a variance check to make sure the variance doesn't collapse

double GaussianSampLikelihood::updateCoefficients(double& K1, double& K2, double Observation, double ModelMean, double ModelVariance) const
{
   
    
	double sigX = sqrt(ModelVariance);
	double y = Observation - likelihoodMean;

	// importance sampler mean and deviation
	double mq = ModelMean; 
	

	// Inflate the prior in the initial population Monte Carlo run
	// This is an arbitrary factor.
	double sq = 4 * sigX;   
	double ev = 0, pM = 0, pV = 0;
	
	// population monte carlo bit here
	for(int iCyc=0; iCyc < SamplingLikelihood::numberCycles; iCyc++)
	{
		vec sx = mq + (itpp::randn(SamplingLikelihood::numberSamples) * sq);
		
		vec transH = apply_function( transform, sx);
		
		//	computing weights
		vec importanceWeights = (itpp::pow((sx - ModelMean) / sigX, 2.0) - itpp::pow((sx - mq) / sq, 2.0)) / 2.0;

		importanceWeights = itpp::exp(-importanceWeights) * sq / sigX;
		
		// multiply the importance weight with likelihood
		importanceWeights = itpp::elem_mult(importanceWeights, exp(-itpp::pow(y - transH, 2.0) / (2.0 * likelihoodVariance)) / sqrt(2.0 * itpp::pi * likelihoodVariance));
		ev = itpp::sum(importanceWeights);
		importanceWeights = importanceWeights / ev;
		
		pM = itpp::sum(itpp::elem_mult(sx, importanceWeights));		
		pV = itpp::sum(itpp::elem_mult(itpp::pow(sx, 2.0), importanceWeights)) - pow(pM, 2.0);

		//	Update the proposal distribution
		mq = pM;
		sq = sqrt(abs(pV));
		
		if (isnan(sq)) {
		    cout << endl;
		    cout << "sx = " << sx << endl;
		    cout << "transH = " << transH << endl;
		    cout << "y = " << y << endl;
		    getchar();
		}
	}
	
	//	Sampling
	K1 = (pM - ModelMean) / ModelVariance;
	K2 = (pV - ModelVariance) / pow(ModelVariance, 2.0);
	
	return log(ev);
	
}
