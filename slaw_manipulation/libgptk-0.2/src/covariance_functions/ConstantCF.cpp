#include "ConstantCF.h"

/**
 * Initialise constant covariance object
 * The bias field is a reference to the first element in the array of
 * parameters.
 */
ConstantCF::ConstantCF(double _bias)
: CovarianceFunction("Constant", 1), bias(parameters[0])
{
	bias = _bias;
	parametersNames[0] = "bias";
}

ConstantCF::~ConstantCF()
{
}

inline double ConstantCF::computeElement(const vec& A, const vec& B) const
{
	return bias;
}

inline double ConstantCF::computeDiagonalElement(const vec& A) const
{
	return bias;
}

void ConstantCF::covarianceGradient(mat& grad, const int parameterNumber, const mat& X) const
{
	assert(parameterNumber == 0);

	Transform* t = getTransform(parameterNumber);
	double gradientTransform = t->gradientTransform(getParameter(parameterNumber));

	switch(parameterNumber)
	{
		case 0 :
		{
		    grad = gradientTransform * ones(X.rows(), X.rows());
		}
	}
}
