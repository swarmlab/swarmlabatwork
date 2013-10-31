#include "GreedyMaxMinDesign.h"

GreedyMaxMinDesign::GreedyMaxMinDesign(double _zweight)
{
    zweight = _zweight;
}

GreedyMaxMinDesign::~GreedyMaxMinDesign()
{
    
}

/**
 *  
 * 
 */     
ivec GreedyMaxMinDesign::subsample(mat X, int n)
{
    if (n <= 0)
        cerr << "Invalid sample size in GreedyMaxMinDesign::subsample(...)" << endl;
    
    int imax;
    
    ivec isample(n);        // Indices of points in sample
    ivec iremain;           // Indices of remaining points
    vec D;                  // Vector of min distances to sample
    
    iremain = to_ivec(linspace(0,X.rows()-1,X.rows()));
    
    // Start with location of maximum Z
    max(X.get_col(X.cols()-1), imax);
    isample(0) = imax;
    iremain.del(imax);

    // Add points having maximum minimum distance to sample
    // until subsample size is reached 
    for (int i=1; i<n; i++) 
    {
    	vec xnew = X.get_row(isample(i-1));                // Last point added 
        vec Dnew = dist(X.get_rows(iremain), xnew);        // Distances to xnew
                
        // Update minimum distances to sample
        if (D.length() == 0) 
            D = Dnew;
        else
            D = min(D,Dnew);
        
        // Find point with max min distance
        max(D,imax);
        
        // Add it to sample 
    	isample(i) = iremain(imax);
    	
    	// And delete it from remainder 
    	iremain.del(imax);
    	D.del(imax);
    }
    
    return isample;
}

/**
 * Returns a vector of distances between the points in R and point y 
 */
vec GreedyMaxMinDesign::dist(mat R, vec y)
{
    vec distances(R.rows());
    
    // For each point x in R
    for (int i=0; i<R.rows(); i++) {
        vec x = R.get_row(i);
        distances(i) = weightedSquareNorm(x-y);
    }
    
    return distances;
}

/**
 * Computes the squared norm of vector v but last component
 * weighted according to zweight (allows to give more importance
 * to the z component over the x and y components)
 */
double GreedyMaxMinDesign::weightedSquareNorm(vec v) 
{
   double sqnorm = 0.0;
   
   for (int i=0; i<v.length()-1; i++) {
       sqnorm += pow(v(i),2.0);
   }   
   sqnorm += zweight*pow(v(v.length()-1),2.0);
   
   return sqnorm;
}
