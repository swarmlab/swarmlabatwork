#include "SumCF.h"

/**
 * Constructor
 */
SumCF::SumCF()
: CovarianceFunction("Sum Covariance",0)
{
	covFunctions.clear();
}

SumCF::SumCF(CovarianceFunction& cf)
: CovarianceFunction("Sum Covariance", 0)
{
	covFunctions.clear();
	add(cf);
}



/**
 * Destructor
 */
SumCF::~SumCF()
{
}


/**
 * Add a covariance function to the sum
 *
 * @param cf a covariance function object
 */
void SumCF::add(CovarianceFunction& cf)
{
	covFunctions.push_back(&cf);
	numberParameters += cf.getNumberParameters();
}


/**
 * Computes the covariance between inputs A and B
 *
 * @param A the first input
 * @param B the second input
 * @return the covariance between A and B
 */
inline double SumCF::computeElement(const vec& A, const vec& B) const
{
	double k = 0.0;

	for(std::vector<CovarianceFunction *>::size_type i = 0; i < covFunctions.size(); i++)
	{
		k = k + covFunctions[i]->computeElement(A, B);
	}

	return k;
}


/**
 * Computes the variance of an input A
 *
 * @param A an input
 * @return the variance of A
 */
inline double SumCF::computeDiagonalElement(const vec& A) const
{
	double k = 0.0;

	for(std::vector<CovarianceFunction *>::size_type i = 0; i < covFunctions.size(); i++)
	{
		k = k + covFunctions[i]->computeDiagonalElement(A);
	}

	return k;
}


/**
 * Display information about the current parameters of the covariance functions.
 * This can be indented by an optional number of space characters (useful
 * for mixtures of nested covariance functions).
 *
 * @param nspaces Number of space characters to indent by.
 */
void SumCF::displayCovarianceParameters(int nspaces) const
{
	string space(nspaces, ' ');
	
    cout << space << "Covariance function : Sum" << endl;
	for(std::vector<CovarianceFunction *>::size_type i = 0; i < covFunctions.size(); i++)
	{
		cout << space << "+ Component: " << (i+1) << endl;
		covFunctions[i]->displayCovarianceParameters(nspaces+2);
	}
}


/**
 * Gradient of the covariance matrix with respect to a given
 * parameter, i.e. d/dparameter cov(X,X)
 *
 * @param D      the gradient of the covariance matrix cov(X,X) with respect
 *               to parameter number p
 * @param p      the parameter number
 * @param X      a set of inputs
 */
void SumCF::covarianceGradient(mat& D, const int p, const mat& X) const
{
    int cfIndex;
    int parcfIndex;

    reindex(cfIndex, parcfIndex, p);
    covFunctions[cfIndex]->covarianceGradient(D, parcfIndex, X);
}


/**
 * Return the transform for parameter number p
 *
 * @param p the parameter number
 * @return a pointer to the transform object for p
 */
Transform* SumCF::getTransform(int p) const
{
    int cfIndex;
    int parcfIndex;

    reindex(cfIndex, parcfIndex, p);

    return covFunctions[cfIndex]->getTransform(parcfIndex);
}


/**
 * Set the transform for parameter number p
 *
 * @param p the parameter number
 * @param t a pointer to the new transform for parameter number p
 */
void SumCF::setTransform(int p, Transform* t)
{
    int cfIndex;
    int parcfIndex;

    reindex(cfIndex, parcfIndex, p);

    covFunctions[cfIndex]->setTransform(parcfIndex, t);
}


/**
 * Set all parameters from a vector of transformed values
 *
 * @param pvec a vector of parameter values (in transformed space, e.g. log of the
 *             actual value for log-transformed parameters)
 */
void SumCF::setTransformedParameters(const vec pvec)
{
	int parFrom = 0;
	int parTo = 0;
	vector<CovarianceFunction*>::iterator cf;

	// Extract the transformed parameters for each covariance and
	// assign them
	for (cf = covFunctions.begin(); cf != covFunctions.end(); cf++)
	{
	    parFrom = parTo;
	    parTo += (*cf)->getNumberParameters();
	    (*cf)->setTransformedParameters( pvec(parFrom, parTo-1) );
	}
}


/**
 * Get a vector of transformed parameter values
 *
 * @return a vector of transformed parameter values (e.g. log of the parameter
 *         value if the parameter has a log-transform)
 */
vec SumCF::getTransformedParameters()
{
	vec transPar;
	vector<CovarianceFunction*>::iterator cf;

	// Append the transformed parameters from each covariance function
	for (cf = covFunctions.begin(); cf != covFunctions.end(); cf++)
	{
		transPar = concat(transPar, (*cf)->getTransformedParameters());
	}

	return transPar;
}


/**
 * Set the value for a given parameter. See setParameters for a description
 * of parameter indexing in SumCF.
 * @param p      the parameter number, between 0 and the total number of parameters
      *          (from all covariance functions)
 * @param value  the new value for the parameter
 */
void SumCF::setParameter(const int p, const double value)
{
    int cfIndex;
    int parcfIndex;

    reindex(cfIndex, parcfIndex, p);
    covFunctions[cfIndex]->setParameter(parcfIndex, value);
}


/**
 * Get a given parameter (out of the parameters from all covariance functions)
 * @param p the parameter number, between 0 and the total number of parameters
 * @return
 * @note parameters are indexed in the order in which the covariance functions
 *       are added to the sum, i.e. for a sum of 2 covariance functions, each with
 *       3 parameters, parameters 0-2 correspond to the parameters of the 1st
 *       covariance function, and parameters 3-5 correspond to those of the second
 *       covariance function.
 * @see reindex
 */
double SumCF::getParameter(const int p) const
{
	int cfIndex;
	int parcfIndex;

	reindex(cfIndex, parcfIndex, p);

	return covFunctions[cfIndex]->getParameter(parcfIndex);
}


/**
 * Return the name of the given parameter. See setParameters for a description
 * of parameter indexing in SumCF.
 *
 * @param p the parameter number, between 0 and the total number of parameters
 *          (from all covariance functions)
 * @return the name of the parameter
 */
string SumCF::getParameterName(const int p) const
{
    int cfIndex;
    int parcfIndex;

    reindex(cfIndex, parcfIndex, p);

    return (covFunctions[cfIndex]->getParameterName(parcfIndex));
}


/**
 * Retrieve the index of the covariance function and of the parameter within
 * from the given parameter number. The given parameter number is between 0
 * and the total number of parameters (from all covariance functions in the sum).
 *
 * @param cfIndex index of the covariance function
 * @param parcfIndex index of the parameter of the covariance function
 * @param parIndex overall parameter number (in all parameters in the sum)
 * @example for a sum of 2 covariance functions, each with 3 parameters (so
 *        a total of 6 parameters indexed 0 to 5), reindex(cfIndex, parcfIndex, 3)
 *        will give:
 *        cfIndex = 1 (the second covariance function)
 *        parcfIndex = 0 (the first parameter in that covariance function)
 */
void SumCF::reindex(int& cfIndex, int& parcfIndex, int parIndex) const
{
    assert(parIndex >= 0 && parIndex < getNumberParameters());

    int p = parIndex;
    int i = 0;
    int npar = covFunctions[i++]->getNumberParameters();

    while ( p >=  npar )
    {
        p -= npar;
        npar = covFunctions[i++]->getNumberParameters();
    }

    // Index of covariance function
    cfIndex = i-1;

    // Index of parameter in covariance function
    parcfIndex = p;
}
