/*
 * StationaryCF.cpp
 *
 *  Created on: 27 Oct 2009
 *      Author: barillrl
 */

#include "StationaryCF.h"

/**
 * Default constructor
 *
 * @param _variance    Process variance
 * @param _lengthScale Correlation length scale
 * @param name         Identifier of the covariance function
 */
StationaryCF::StationaryCF(string name, double _lengthScale, double _variance)
: CovarianceFunction(name, 2), lengthScale(parameters[0]), variance(parameters[1])
{
    // Set parameter values
    variance = _variance;
    lengthScale = _lengthScale;

    // Parameter names
    parametersNames[0] = "length scale";
    parametersNames[1] = "variance";
}


/**
 * Default destructor
 */
StationaryCF::~StationaryCF()
{
    // TODO Auto-generated destructor stub
}


/**
 * Computes the covariance between inputs u and v
 *
 * @param u the first input
 * @param v the second input
 * @return the covariance between u and v
 */
double StationaryCF::computeElement(const vec& u, const vec& v) const
{
    return computeElement( sqDist(u,v) );
}


/**
 * Compute the auto-covariance of input u (for stationary covariance
 * functions, this simply returns the process variance)
 *
 * @param u the input
 * @return the auto-covariance of u (i.e. the process variance)
 */
double StationaryCF::computeDiagonalElement(const vec& u) const
{
    return variance * correlation(0.0);
}


/**
 * Compute the covariance for a given squared distance
 */
double StationaryCF::computeElement(double sqDist) const
{
    return variance * correlation(sqDist);
}


/**
 * Computes the symmetric covariance matrix between all inputs in X.
 *
 * @param C the covariance matrix cov(X,X)
 * @param X a set of (row) inputs
 */
void StationaryCF::covariance(mat& C, const mat& X) const
{
    // Compute matrix of squared distances
    sqDistMatrix(C, X);

    // Apply correlation function
    applyCorrelation(C);

    // Finally, scale by process variance
    C *= variance;
}


/**
 * Gradient of the covariance matrix with respect to a given
 * parameter, i.e. d/dp cov(X,X)
 *
 * @param D      the gradient of the covariance matrix cov(X,X) with respect
 *               to parameter number p
 * @param p      the parameter number
 * @param X      a set of inputs
 */
void StationaryCF::covarianceGradient(mat& D, const int p, const mat& X) const
{
    assert(p>=0 && p<numberParameters);

    // Compute matrix of squared distances
    sqDistMatrix(D, X);

    Transform* t = getTransform(p);
    double gradientTransform = t->gradientTransform(parameters[p]);

    switch (p)
    {
    case 0:
        applyCorrelationGradient(p, D);        // Compute gradient of correlation matrix
        D *= variance;                           // Multiply by variance
        break;

    case 1:
        applyCorrelation(D);
        break;

    }

    // Multiply by the gradient of the transform
    D *= gradientTransform;
}


/**
 * Apply the correlation function to a matrix of square distances.
 *
 * @param D A matrix of square distances. These get overriden with the
 *          correlation for each square distance.
 */
void StationaryCF::applyCorrelation(mat& D) const
{
    for(int i=0; i<D.rows(); i++) {
        for (int j=0; j<D.cols(); j++) {
            D(i,j) = correlation(D(i,j));
        }
    }
}


/**
 * Apply the gradient of the correlation function to a matrix
 * of square distances.
 *
 * @param p the number of the parameter to differentiate with respect to
 * @param D A matrix of square distances. These get overriden with the
 *          gradient of the correlation for each square distance
 */
void StationaryCF::applyCorrelationGradient(int p, mat& D) const
{
    for(int i=0; i<D.rows(); i++) {
        for (int j=0; j<D.cols(); j++) {
            D(i,j) = correlationGradient(p, D(i,j));
        }
    }
}



/**
 * Compute the square distance between two vectors (i.e. the squared 2-norm)
 *
 * @param u the first vector
 * @param v the second vector
 * @return the squared distance between u and v
 */
double StationaryCF::sqDist(vec u,vec v)
{
    assert(length(u) == length(v));

    double d = 0.0;
    for (int i=0; i<length(u); i++) d += (u(i)-v(i))*(u(i)-v(i));
    return d;
}


/**
 * Computes the matrix of square distances between any two inputs in X.
 *
 * @param D  the matrix of squared distances
 * @param X  a set of (row) inputs
 */
void StationaryCF::sqDistMatrix(mat &D, const mat X)
{
    D = zeros(X.rows(), X.rows());

    for (int i=0; i<X.rows(); i++) {
        for(int j=0; j<i; j++) {
            D(i,j) = sqDist(X.get_row(i), X.get_row(j));
            D(j,i) = D(i,j);
        }
    }
}
