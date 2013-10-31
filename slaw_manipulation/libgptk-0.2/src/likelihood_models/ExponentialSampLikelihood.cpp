#include "ExponentialSampLikelihood.h"

#include <cmath>

using namespace std;
using namespace itpp;

/**
 * Constructor with default (identity) observation operator.
 * Use this is the data is observed directly.
 */
ExponentialSampLikelihood::ExponentialSampLikelihood(double Lambda)
{
	likelihoodLambda = Lambda;
	likelihoodLikpar = 1 / Lambda;
}

/**
 * Constructor with custom observation operator.
 * Specify the mapping from state space to observation space.
 */
ExponentialSampLikelihood::ExponentialSampLikelihood(double Lambda, double (*_transform)(double))
{
    likelihoodLambda = Lambda;
    likelihoodLikpar = 1 / Lambda;
    transform = _transform;
}

ExponentialSampLikelihood::~ExponentialSampLikelihood()
{
}

// TODO  we need a variance check to make sure the variance doesn't collapse

double ExponentialSampLikelihood::updateCoefficients(double& K1, double& K2, double Observation, double ModelMean, double ModelVariance) const
{
    double sigX = sqrt(ModelVariance);
	double y = Observation;

	// importance sampler mean and deviation
	double mq = ModelMean;
	
	// Inflate the prior in the initial population Monte Carlo run
	// This is an arbitrary factor - increase for the exponential case
	// to ensure that some samples fall below the observations.
	double sq = 8.0 * sigX;   
	double ev = 0, pM = 0, pV = 0;
	
	// population monte carlo bit here
	for(int iCyc=0; iCyc < SamplingLikelihood::numberCycles; iCyc++)
	{
	    // Sample from the variance-inflated model 
		vec sx = mq + (itpp::randn(SamplingLikelihood::numberSamples) * sq);
		
		vec transH = apply_function(transform, sx);
		
		//	computing weights
		vec importanceWeights = (itpp::pow((sx - ModelMean) / sigX, 2.0) - itpp::pow((sx - mq) / sq, 2.0)) / 2.0;
		
		importanceWeights = itpp::exp(-importanceWeights) * sq / sigX;
		
		//	multiply the importance weight with likelihood
		vec likFunc = likelihoodLambda * exp(-likelihoodLambda*abs(y - transH));
		
		// Under the exponential likelihood, we must have y>x (the 
		// tranformed samples must lie under the observation - this is
		// due to the one sided nature of postitive exponential noise).
		// Discard samples that do not conform to that inequality 
		for(int i=0 ; i < likFunc.length(); i++) {
			if( y <= transH(i)) {
				likFunc(i) = 0.0;
			}
		}
		
		importanceWeights = itpp::elem_mult(importanceWeights, likFunc);
		
		ev = itpp::sum(importanceWeights);
		importanceWeights = importanceWeights / ev;
		
		
		pM = itpp::sum(itpp::elem_mult(sx, importanceWeights));		
		pV = itpp::sum(itpp::elem_mult(itpp::pow(sx, 2.0), importanceWeights)) - pow(pM, 2.0);
		
		//	Update the proposal distribution
		mq = pM;
		sq = sqrt(abs(pV));
	}

	K1 = (pM - ModelMean) / ModelVariance;
	K2 = -(ModelVariance - pV) / pow(ModelVariance, 2.0);

	return log(ev);
	
}
