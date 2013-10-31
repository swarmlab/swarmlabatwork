#include "Matern5CF.h"

/*
 * Constructor
 */
Matern5CF::Matern5CF(double lengthScale, double variance)
: StationaryCF("Matern 5/2 covariance function", lengthScale, variance)
{
}


/*
 * Destructor
 */
Matern5CF::~Matern5CF()
{
}

/**
 * Matern 5/2 correlation function (isotropic) between two inputs
 *
 * @param sqDist the squared distance between the two inputs
 * @return the correlation between the two inputs
 */
double Matern5CF::correlation(double sqDist) const
{
    double r = sqrt(5.0 * sqDist) / lengthScale;
     
    return ( 1.0 + r + sqr(r)/3.0 ) * exp(-r);
}


/**
 * Gradient of the Matern 5/2 correlation function (isotropic) between two inputs,
 * with respect to a given parameter
 *
 * @param p      the number of the parameter
 * @param sqDist the squared distance between two inputs
 * @return       the gradient with respect to parameter
 */
double Matern5CF::correlationGradient(int p, double sqDist) const
{
	double r;
	assert(p>=0 && p<numberParameters);

    switch (p) {
    // Gradient with respect to length scale
    case 0:
    	r = sqrt(5.0 * sqDist) / lengthScale;
    	return (r+sqr(r)) * r / (3.0 * lengthScale) * exp(-r);

    // Gradient with respect to process variance
    case 1:
        return 0.0;
    }
}


