#include "CovarianceFunction.h"

using namespace std;
using namespace itpp;

/**
 * Constructor
 * @param name            a string identifier for the covariance function
 * @param numParameters   the number of parameters for this covariance function
 */
CovarianceFunction::CovarianceFunction(string name, int numParameters)
{
    numberParameters = numParameters;

    parameters = new double[numParameters];
    parametersNames = vector<string>(numParameters);

    covarianceName = name;

    // The default transform, if none specified, is log
    defaultTransform = new LogTransform();
    applyDefaultTransform();
}

/**
 * Constructor
 * @param name            a string identifier for the covariance function
 * @param numParameters   the number of parameters for this covariance function
 * @param t               the transform to be used for all parameters
 */
CovarianceFunction::CovarianceFunction(string name, int numParameters, Transform* t)
{
    numberParameters = numParameters;

    parameters = new double[numParameters];
    parametersNames = vector<string>(numParameters);

    covarianceName = name;

    setDefaultTransform(t);
}



/**
 * Default destructor
 */
CovarianceFunction::~CovarianceFunction()
{
}


/**
 * Compute the auto-covariance of a single input
 *
 * @param c The auto-covariance of x
 * @param x An input vector
 */
void CovarianceFunction::covariance(double &c, const vec& x) const
{
   mat C(1,1);
   covariance(C,x.transpose());
   c = C(0,0);
}


/**
 * Compute the covariance matrix of a set of inputs
 * @param C The covariance matrix of X
 * @param X A matrix of inputs (one input per row)
 */
void CovarianceFunction::covariance(mat& C, const mat& X) const
{
	// ensure that data dimensions match supplied covariance matrix
	assert(C.rows() == X.rows());
	assert(C.cols() == X.rows());

	if (X.rows() == 1)
	{
	    C.set(0, 0, computeDiagonalElement(X.get_row(0)));
	    return;
	}
	
	// calculate the lower and upper triangles
	double d;
	
	for(int i=0; i<X.rows() ; i++)
	{
		for(int j=0; j<i; j++)
		{
		    d = computeElement(X.get_row(i), X.get_row(j));
		    C.set(i, j, d);
			C.set(j, i, d);
		}
	}

	// calculate the diagonal part
	for(int i=0; i<X.rows() ; i++)
	{
		C.set(i, i, computeDiagonalElement(X.get_row(i)));
	}
}


/**
 * Gradient of the covariance matrix of a set of inputs with respect
 * to the parameters of the covariance function
 *
 * @param g The gradient vector
 * @param X A set of inputs (one per row)
 */
// void CovarianceFunction::gradCovariance(vec& g, const mat& X) const
// {
// }


/**
 * Covariance between a set of n inputs X and another input x
 *
 * @param c The nx1 vector covariance vector cov(X,x)
 * @param X A set of n inputs (one per row)
 * @param x Another input
 */
void CovarianceFunction::covariance(vec& c, const mat& X, const vec& x) const
{
    mat Xmat(x);
    mat C(X.rows(),1);
    covariance(C,X,Xmat.transpose());
    c = C.get_col(0);
}


/**
 * Covariance between two sets of inputs X1 and X2
 *
 * @param C  The nxm covariance matrix cov(X1,X2)
 * @param X1 A set of n inputs
 * @param X2 A set of m inputs
 */
void CovarianceFunction::covariance(mat& C, const mat& X1, const mat& X2) const
{
	assert(C.rows() == X1.rows());
	assert(C.cols() == X2.rows());

	for(int i=0; i<X1.rows() ; i++)
	{
		for(int j=0; j<X2.rows(); j++)
		{
			C.set(i, j, computeElement(X1.get_row(i), X2.get_row(j)));
		}
	}
}


/**
 * Compute the variance of a diagonal element, i.e. cov(A,A)
 * This is the default and returns computeElement(A,A). This should
 * be overridden when possible with a more efficient implementation,
 * specific to the type of covariance (for instance, for stationary
 * covariance functions, this takes the value of the variance parameter).
 */
double CovarianceFunction::computeDiagonalElement(const vec& A) const {
    return computeElement(A, A);
}

/**
 * Diagonal covariance matrix cov(X,X). The non-diagonal terms are ignored.
 *
 * @param C A diagonal matrix with with diagonal elements C_ii = cov(X_i, X_i)
 * @param X A set of inputs (one per row)
 */
void CovarianceFunction::computeDiagonal(mat& C, const mat& X) const
{
    C = zeros(X.rows(), X.rows());

	// calculate the diagonal part
	for(int i=0; i<X.rows() ; i++)
	{
		C.set(i, i, computeDiagonalElement(X.get_row(i)));
	}
}


/**
 * Diagonal elements of the matrix cov(X,X).
 *
 * @param C A vector of diagonal elements C_i = cov(X_i, X_i)
 * @param X A set of inputs (one per row)
 */
void CovarianceFunction::computeDiagonal(vec& C, const mat& X) const
{
	// calculate the diagonal part
	for(int i=0; i<X.rows() ; i++)
	{
		C.set(i, computeDiagonalElement(X.get_row(i)));
	}
}


/**
 * Returns the name of the given parameter
 *
 * @param parameterNumber the number of the parameter
 * @return the name of the parameter
 */
string CovarianceFunction::getParameterName(const int parameterNumber) const
{
    return parametersNames[parameterNumber];
}


/**
 * Display information about the current parameters of the covariance function.
 * This can be indented by an optional number of space characters (useful
 * for mixtures of nested covariance functions).
 *
 * @param nspaces Number of space characters to indent by.
 */
 void CovarianceFunction::displayCovarianceParameters(int nspaces) const
{
	cout.setf(ios::fixed);
	cout.precision(4);

	string space = string(nspaces, ' ');
	
	cout << space << "Covariance function : " << covarianceName << endl;

	for(int i=0; i < numberParameters; i++)
	{
		cout << space << getParameterName(i) << " : ";
        cout << getParameter(i);
        cout << " (" << transforms[i]->type() << ")" << endl;
	}
}



/**
 * Get the current covariance function parameters
 *
 * @return A vector of parameter values
 */
vec CovarianceFunction::getParameters()
{
	vec result;
	result.set_size(numberParameters);

	for(int i = 0; i < numberParameters; i++)
	{
		result[i] = getParameter(i);
	}
	return result;
}


/**
 * Set the parameters of the covariance function to that of the argument vector.
 *
 * @param p Vector of new parameter values
 */
void CovarianceFunction::setParameters(const vec p)
{
    assert(p.size() == numberParameters);

    for(int i = 0; i < numberParameters; i++)
    {
        setParameter(i, p(i));
    }
}


/**
 * Get the current covariance function parameters (transformed)
 *
 * @return A vector containing all parameters of the covariance function,
 *         in transformed space (e.g. log-space for positive parameters).
 */
vec CovarianceFunction::getTransformedParameters()
{
    assert(transforms.size() == numberParameters);
    vec result;
    result.set_size(numberParameters);

    for(int i = 0; i < numberParameters ; i++)
    {
        result[i] = transforms[i]->forwardTransform(getParameter(i));
    }
    return result;
}


/**
 * Set the parameters of the covariance function to that of the argument vector.
 * The argument vector must contain one value for each parameter and these values
 * must be transformed (i.e. if a parameter is optimised in log-space, then its
 * new value must be in log-space too).
 *
 * @param p Vector of new parameter values (transformed)
 */
void CovarianceFunction::setTransformedParameters(const vec p)
{
    assert(transforms.size() == p.size());
    for(int i = 0; i < getNumberParameters() ; i++)
    {
        setParameter(i, transforms[i]->backwardTransform(p(i)));
    }
}


/**
 * Returns the number of parameters in the covariance function
 * @return The number of parameters in the covariance function
 */
int CovarianceFunction::getNumberParameters() const
{
	return numberParameters;
}

/**
 * Return a given parameter value
 *
 * @param p the parameter number
 * @return the value of parameter number p
 */
double CovarianceFunction::getParameter(int p) const
{
    assert(p>=0 && p<numberParameters);

    return parameters[p];
}


/**
 * Set the value of a given parameter
 *
 * @param p the parameter number
 * @param value the new value of the parameter
 */
void CovarianceFunction::setParameter(int p, double value)
{
    assert(p>=0 && p<numberParameters);

    parameters[p] = value;
}


/**
 * Applies the default transform to all parameters. By default,
 * the default transform is a LogTransform (meaning all parameters
 * are assumed positive). A different default transform can be specified
 * in the constructor or using the setDefaultTransform method. Individual
 * transforms can also be set for each parameter using the setTransform
 * method.
 *
 *
 * @see CovarianceFunction, setDefaultTransform, setTransform
 */
void CovarianceFunction::applyDefaultTransform()
{
    transforms.clear();
    for(int i = 0; i < numberParameters; i++)
    {
        transforms.push_back(defaultTransform);
    }
    transformsApplied = true;
}

/**
 * Set the default transform for all parameters. The new
 * transform is applied immediately.
 *
 * @param t a pointer to an instance of Transform
 */
void CovarianceFunction::setDefaultTransform(Transform* t)
{
    defaultTransform = t;

    // Apply the new transform to all parameters
    applyDefaultTransform();
}


/**
 * Set the transform for a given parameter.
 *
 * @param parameterNumber The parameter number
 * @param newTransform    The transform to be applied to the parameter (e.g.
 *                        a pointer to an instance of LogTransform)
 */
void CovarianceFunction::setTransform(int parameterNumber, Transform* newTransform)
{
	assert(parameterNumber >= 0);
	assert(parameterNumber < numberParameters);
	assert(parameterNumber < transforms.size());

	transforms[parameterNumber] = newTransform;
}


/**
 * Get the transform for a given parameter of the covariance function
 *
 * @param parameterNumber The parameter number
 * @return a pointer to the transform object currently applied to that parameter
 */
Transform* CovarianceFunction::getTransform(int parameterNumber) const
{
	assert(parameterNumber >= 0);
	assert(parameterNumber < getNumberParameters());
	return transforms[parameterNumber];
}

