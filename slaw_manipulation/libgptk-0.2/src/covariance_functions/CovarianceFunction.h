/***************************************************************************
 *   AstonGeostats, algorithms for low-rank geostatistical models          *
 *                                                                         *
 *   Copyright (C) Ben Ingram, Remi Barillec, 2008-09                      *
 *                                                                         *
 *   Ben Ingram <IngramBR@Aston.ac.uk>                                     *
 *   Remi Barillec <r.barillec@aston.ac.uk>                                *
 *   Neural Computing Research Group,                                      *
 *   Aston University,                                                     *
 *   Aston Street, Aston Triangle,                                         *
 *   Birmingham. B4 7ET.                                                   *
 *   United Kingdom                                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef COVARIANCEFUNCTION_H_
#define COVARIANCEFUNCTION_H_

#include <iostream>
#include <vector>
#include <string>
#include <cassert>
#include <itpp/itbase.h>

#include "parameter_transforms/LogTransform.h"

using namespace std;
using namespace itpp;

/**
 * Abstract class for covariance function objects. This should be overloaded
 * by specific covariance functions.
 *
 * Transforms are applied to the parameters of the covariance function to ensure
 * they remain sensible when being optimised via Maximum Likelihood. By default,
 * the transform is set to identity, i.e. the parameters are optimised accross
 * the whole real space. Specific covariance should overload the setDefaultTransforms()
 * method to set sensible transforms for all parameters (e.g. one could use a LogTransform
 * to constrain some parameters to remain positive).
 */
class CovarianceFunction
{
public:
    CovarianceFunction(string name, int numParameters);
    CovarianceFunction(string name, int numParameters, Transform* t);
    virtual ~CovarianceFunction();

    virtual double computeElement(const vec& A, const vec& B) const = 0;
    virtual double computeDiagonalElement(const vec& A) const;

    virtual void covariance(double& c, const vec& X) const;
    virtual void covariance(mat& C, const mat& X) const;
    virtual void covariance(vec& C, const mat& X, const vec& x) const;
    virtual void covariance(mat& C, const mat& X1, const mat& X2) const;

    virtual void covarianceGradient(mat& grad, const int parameterNumber, const mat& X) const = 0;

    virtual void computeDiagonal(mat& C, const mat& X) const;
    virtual void computeDiagonal(vec& C, const mat& X) const;

    virtual void setParameter(const int parameterNumber, const double value);
    virtual double getParameter(const int parameterNumber) const;

    virtual string getParameterName(const int parameterNumber) const;

    virtual void setTransform(int parameterNumber, Transform* newTransform);
    virtual Transform* getTransform(int parameterNumber) const;

    virtual void setParameters(const vec p);
    virtual vec getParameters();

    virtual void setTransformedParameters(const vec p);
    virtual vec getTransformedParameters();

    int getNumberParameters() const;

    virtual void displayCovarianceParameters(int nspaces = 0) const;

protected:
    void setDefaultTransform(Transform* t);
    void applyDefaultTransform();

    string covarianceName;
    int numberParameters;
    bool transformsApplied;

    double *parameters;                    // Array of parameters
    vector<string> parametersNames;        // Array of parameter names

    Transform* defaultTransform;
    vector<Transform *> transforms;

};

#endif /*COVARIANCEFUNCTION_H_*/
