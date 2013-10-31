#include "WhiteNoiseCF.h"

/**
 * Constructor
 *
 * @param _variance the noise variance
 */
WhiteNoiseCF::WhiteNoiseCF(double _variance)
: CovarianceFunction("Gaussian white noise", 1), variance(parameters[0])
{
	variance = _variance;
	parametersNames[0] = "nugget variance";
}

/**
 * Destructor
 */
WhiteNoiseCF::~WhiteNoiseCF()
{
}

/**
 * Covariance between two inputs. If the two inputs are identical, this returns
 * the noise variance, and zero otherwise.
 *
 * @param A the first input
 * @param B the second input
 * @return the noise variance if A == B, 0 otherwise
 */
inline double WhiteNoiseCF::computeElement(const vec& A, const vec& B) const
{
	if (A==B)
	    return variance;
	else
		return 0.0;
}

/**
 * Variance of a single input. This always returns the noise variance.
 * @param A the input
 * @return the noise variance
 */
inline double WhiteNoiseCF::computeDiagonalElement(const vec& A) const
{
	return variance;
}


/**
 * Gradient of the covariance matrix for a set of inputs, with respect to
 * a given parameter.
 *
 * @param G    the gradient of cov(X,X) with respect to parameter p
 * @param p    the parameter number
 * @param X    a set of (row) inputs
 */
void WhiteNoiseCF::covarianceGradient(mat& G, const int p, const mat& X) const
{
	assert(p == 0);

	Transform* t = getTransform(p);
	double gradientModifier = t->gradientTransform(parameters[p]);

	switch(p)
	{
		case 0 :
		{
			covariance(G, X);
			G *= (gradientModifier / variance);
			return;
			break;
		}
	}
}
