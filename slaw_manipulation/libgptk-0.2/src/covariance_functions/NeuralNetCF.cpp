#include "NeuralNetCF.h"

/*
 * Constructor - pass in the length scale, variance and offset
 */
NeuralNetCF::NeuralNetCF(double ls, double var, double offst)
: CovarianceFunction("Neural network covariance function", 3), sigma2(parameters[0]),
variance(parameters[1]), offset(parameters[2])
{
    // Make sure parameters are positive 
    assert( ls > 0.0 && var > 0.0 && offset > 0.0);  
    
    sigma2 = ls;   // Length scale
    variance = var;
    offset = offst;
    
    // Parameter names
    parametersNames[0] = "sigma2";
    parametersNames[1] = "variance";
    parametersNames[2] = "offset";
}


/**
 * Destructor
 */ 
NeuralNetCF::~NeuralNetCF()
{
}

/**
 * Covariance between two points A and B
 */
inline double NeuralNetCF::computeElement(const vec& A, const vec& B) const
{
    // vec Ahat = A*sqrt(sigma2);
    // vec Bhat = B*sqrt(sigma2);
    
    // return variance * asin( 2*(offset+dot(Ahat,Bhat)) /
    //                        sqrt( (1.0+offset+dot(Ahat,Ahat)) * (1.0+offset+dot(Bhat,Bhat)) )
    //                      );}
    
    double u = offset+dot(A,B)*sigma2;
    double vA = 1.0+offset+dot(A,A)*sigma2;
    double vB = 1.0+offset+dot(B,B)*sigma2;
    double v = sqrt(vA*vB);
    
    return variance * asin( u / v) * 2/M_PI;
}

/** 
 * Gradient of cov(X) w.r.t. given parameter number
 */
void NeuralNetCF::covarianceGradient(mat& grad, const int parameterNumber, const mat& X) const
{
    assert(parameterNumber < getNumberParameters());
    assert(parameterNumber >= 0);

    Transform* t = getTransform(parameterNumber);
    double gradientModifier = t->gradientTransform(getParameter(parameterNumber));

    switch(parameterNumber)
    {
        // Derivative with respect to sigma2
		case 0 :
        {
            vec Xi, Xj;
            
            for (int i=0; i<X.rows(); i++)
            {
                Xi = X.get_row(i);
                
                double vA = 1 + offset + dot(Xi, Xi) * sigma2;
                double dvA = dot(Xi, Xi);
                
                for (int j=0; j<=i; j++) 
                {
                    Xj = X.get_row(j);
                    
                    double u = offset+dot(Xi,Xj)*sigma2;
                    double vB = 1 + offset + dot(Xj, Xj) * sigma2;
                    double v = sqrt(vA*vB);
                    
                    double du = dot(Xi,Xj);
                    double dvB = dot(Xj, Xj);
                    double dv = 0.5 * (dvA*vB + vA*dvB) / v;
                    
                    grad(i,j) = (du*v - u*dv) / (v*sqrt(v*v - u*u));
                    grad(j,i) = grad(i,j);
                }
            }
            grad *= variance * 2/M_PI;

            break;
        }

        // Derivative with respect to variance
        case 1 :
        {
            covariance(grad, X);
            grad /= variance;
            break;
        }

        // Derivative with respect to offset
        case 2:
        	vec Xi, Xj;

        	for (int i=0; i<X.rows(); i++)
        	{
        		Xi = X.get_row(i);

        		double vA = 1 + offset + dot(Xi, Xi) * sigma2;

        		for (int j=0; j<=i; j++) 
        		{
        			Xj = X.get_row(j);

        			double u = offset+dot(Xi,Xj)*sigma2;
        			double vB = 1 + offset + dot(Xj, Xj) * sigma2;
        			double v = sqrt(vA*vB);

        			double dv = 0.5 * (vB + vA) / v;

        			grad(i,j) = (v - u*dv) / (v*sqrt(v*v - u*u));
        			grad(j,i) = grad(i,j);
        		}
        	}
        	grad *= variance * 2/M_PI;

        	break;
    }
    
    
    grad *= gradientModifier;

}
