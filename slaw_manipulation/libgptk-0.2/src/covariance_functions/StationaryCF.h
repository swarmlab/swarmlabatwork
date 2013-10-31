/**
 * StationaryCF.h
 *
 *  Created on: 27 Oct 2009
 *      Author: barillrl
 */

#ifndef STATIONARYCF_H_
#define STATIONARYCF_H_

#include "CovarianceFunction.h"
#include "parameter_transforms/LogTransform.h"

/**
 * Stationary covariance function. This is the base class for stationary
 * covariance functions of the form: sigma * k(sqdit), where sigma is a
 * scaling factor (process variance) and k is a correlation function which
 * only depends on the squared distance between two inputs (sqdist).
 */
class StationaryCF: public CovarianceFunction
{
public:
    StationaryCF(string name, double variance, double lengthScale);
    virtual ~StationaryCF();

    virtual void covariance(mat& C, const mat& X) const;
    virtual void covarianceGradient(mat& grad, const int parameterNumber, const mat& X) const;

    static void sqDistMatrix(mat &D, const mat X);
    static double sqDist(vec u, vec v);

protected:
    double &variance;        // Process variance
    double &lengthScale;     // Length scale

    virtual double computeElement(const vec& u, const vec& v) const;
    virtual double computeDiagonalElement(const vec& u) const;
    double computeElement(double sqDist) const;

    virtual double correlation(double sqDist) const = 0;
    virtual double correlationGradient(int paramNumber, double sqDist) const = 0;

    void applyCorrelation(mat &sqDist) const;
    void applyCorrelationGradient(int paramNumber, mat &sqDist) const;

};

#endif /* STATIONARYCF_H_ */
