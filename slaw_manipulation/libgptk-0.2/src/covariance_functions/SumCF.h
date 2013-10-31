/***************************************************************************
 *   AstonGeostats, algorithms for low-rank geostatistical models          *
 *                                                                         *
 *   Copyright (C) Ben Ingram, 2008                                        *
 *                                                                         *
 *   Ben Ingram, IngramBR@Aston.ac.uk                                      *
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

#ifndef SUMCOVARIANCEFUNCTION_H_
#define SUMCOVARIANCEFUNCTION_H_

#include "CovarianceFunction.h"
#include "parameter_transforms/Transform.h"

#include <cmath>
#include <vector>
#include <cassert>
#include <itpp/itbase.h>

using namespace std;
using namespace itpp;

/**
 * Sum of covariance functions. This allows to build complex
 * covariance functions using a mixture of basic ones.
 */
class SumCF : public CovarianceFunction
{
public:
	SumCF();
	SumCF(CovarianceFunction& cf);
	~SumCF();

	inline double computeElement(const vec& A, const vec& B) const;
	inline double computeDiagonalElement(const vec& A) const;
	
	void covarianceGradient(mat& G, const int p, const mat& X) const;
	

	// We need to override all methods dealing with parameter indexes,
	// as these will be different
	void   setParameter(const int parameterNumber, const double value);
	double getParameter(const int parameterNumber) const;
	string getParameterName(const int parameterNumber) const;

	void   setTransformedParameters(const vec p);
	vec    getTransformedParameters();

	void setTransform(int parameterNumber, Transform* newTransform);
	Transform* getTransform(int parameterNumber) const;

	void add(CovarianceFunction& cf);
	void displayCovarianceParameters(int nspaces = 0) const;
	
private:
	vector<CovarianceFunction *> covFunctions;
	void reindex(int& cfIndex, int& parcfIndex, int parIndex) const;
};



#endif /*SUMCOVARIANCEFUNCTION_H_*/
