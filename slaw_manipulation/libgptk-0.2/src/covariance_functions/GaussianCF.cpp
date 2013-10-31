/*
 * GaussianCF.cpp
 *
 *  Created on: 28 Oct 2009
 *      Author: barillrl
 */

#include "GaussianCF.h"


/**
 * Default constructor
 *
 * @param _variance      the process variance
 * @param _lengthScale   the correlation length scale
 */
GaussianCF::GaussianCF(double lengthScale, double variance)
: StationaryCF("Isotropic Gaussian (squared exponential)", lengthScale, variance)
{
}


/**
 * Default destructor
 */
GaussianCF::~GaussianCF()
{
}


/**
 * Gaussian correlation function (isotropic) between two inputs
 *
 * @param sqDist the squared distance between the two inputs
 * @return the correlation between the two inputs
 */
double GaussianCF::correlation(double sqDist) const
{
    return exp(-0.5 * sqDist / sqr(lengthScale) );
}


/**
 * Gradient of the Gaussian correlation function (isotropic) between two inputs,
 * with respect to a given parameter
 *
 * @param p      the number of the parameter
 * @param sqDist the squared distance between two inputs
 * @return       the gradient with respect to parameter
 */
double GaussianCF::correlationGradient(int p, double sqDist) const
{
    assert(p>=0 && p<numberParameters);

    switch (p) {
    // Gradient with respect to length scale
    case 0:
        return sqDist * correlation(sqDist) / pow(lengthScale,3.0);

    // Gradient with respect to process variance
    case 1:
        return 0.0;
    }
}

