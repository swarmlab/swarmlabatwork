#include "MinMaxDesign.h"

MinMaxDesign::MinMaxDesign(int _nsamples)
{
    if (nsamples <= 0) {
        cout << "Warning: Negative sample number in MinMaxDesign constructor." << endl;
        cout << "         Reverting to default (100)" << endl;
        nsamples = 100;
    }
    else
        nsamples = _nsamples;
}

MinMaxDesign::~MinMaxDesign()
{

}

/**
 * Returns a random subsample of X of size n which has minimum (amongst 
 * a set of NSAMPLES similar subsamples) maximum distance between points.
 */     
ivec MinMaxDesign::subsample(mat X, int n)
{
    if (n <= 0)
        cerr << "Invalid sample size in MinMaxDesign::subsample(...)" << endl;
    
    double minMaxDist = 0.0;
    ivec bestSubsample = randperm(X.rows());
    
    for (int i=0; i<nsamples; i++) {
        
    	// Generate random subsample
    	ivec irand = randperm(X.rows());
    	ivec irandn = irand(0,n-1);
        mat S = X.get_rows(irandn);
        
        // Compute max distance in this subsample
        double maxDist =  distanceMax(S);

        // Keep subsample if has min max distance
        if (maxDist < minMaxDist) {
        	bestSubsample = irandn;
        	minMaxDist = maxDist;
        }
    }
    
    return bestSubsample;
}

/**
 * Returns the maximum distance between any 2 points from X
 */
double MinMaxDesign::distanceMax(mat X)
{
   double distMax = 0.0;
   double d;
   
   for (int i=0; i<X.rows(); i++) {
       vec x1 = X.get_row(i);
       for (int j=i; j<X.rows(); j++) {
           d = norm(x1 - X.get_row(j));
           if (d>distMax) distMax = d;
       }
   }
   return distMax;
}
