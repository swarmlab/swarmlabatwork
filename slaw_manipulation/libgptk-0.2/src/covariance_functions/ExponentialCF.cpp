#include "ExponentialCF.h"

using namespace std;
using namespace itpp;

/**
 * Default constructor
 *
 * @param variance      the process variance
 * @param lengthScale   the correlation length scale
 */
ExponentialCF::ExponentialCF(double lengthScale, double variance)
: StationaryCF("Isotropic Exponential", lengthScale, variance)
{
}

ExponentialCF::~ExponentialCF()
{
}

/**
 * Exponential correlation function (isotropic) between two inputs
 *
 * @param sqDist the squared distance between the two inputs
 * @return the correlation between the two inputs
 */
inline double ExponentialCF::correlation(double sqDist) const
{
	return exp( - 0.5 * sqrt(sqDist) / lengthScale );
}

/**
 * Gradient of the Gaussian correlation function (isotropic) between two inputs,
 * with respect to a given parameter
 *
 * @param p      the number of the parameter
 * @param sqDist the squared distance between two inputs
 * @return       the gradient with respect to parameter
 */
double ExponentialCF::correlationGradient(int p, double sqDist) const
{
	assert(p>=0 && p<numberParameters);

	switch (p) {
	// Gradient of correlation function with respect to length scale
	case 0:
		return 0.5 * sqrt(sqDist) * correlation(sqDist) / sqr(lengthScale);

	// Gradient of correlation function with respect to process variance
	case 1:
		return 0.0;
	}
}
