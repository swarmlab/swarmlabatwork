#include "Matern3CF.h"

/*
 * Constructor
 */
Matern3CF::Matern3CF(double lengthScale, double variance)
: StationaryCF("Matern 3/2 covariance function", lengthScale, variance)
{
}


/*
 * Destructor
 */
Matern3CF::~Matern3CF()
{
}


/**
 * Matern 3/2 correlation function (isotropic) between two inputs
 *
 * @param sqDist the squared distance between the two inputs
 * @return the correlation between the two inputs
 */
double Matern3CF::correlation(double sqDist) const
{
    double r = sqrt(3.0 * sqDist) / lengthScale;
     
    return (1.0+r) * exp(-r);
}


/**
 * Gradient of the Matern 3/2 correlation function (isotropic) between two inputs,
 * with respect to a given parameter
 *
 * @param p      the number of the parameter
 * @param sqDist the squared distance between two inputs
 * @return       the gradient with respect to parameter
 */
double Matern3CF::correlationGradient(int p, double sqDist) const
{
	double r;
	assert(p>=0 && p<numberParameters);

    switch (p) {
    // Gradient with respect to length scale
    case 0:
    	r = sqrt(3.0 * sqDist) / lengthScale;
    	return sqr(r) / lengthScale * exp(-r);

    // Gradient with respect to process variance
    case 1:
        return 0.0;
    }
}

