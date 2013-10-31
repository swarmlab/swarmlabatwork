#include "MaxMinDesign.h"

MaxMinDesign::MaxMinDesign(int _nsamples)
{
    if (_nsamples <= 0) {
        cout << "Warning: Negative sample number in MaxMinDesign constructor." << endl;
        cout << "         Reverting to default (100)" << endl;
        nsamples = 100;
    }
    else
        nsamples = _nsamples;
}

MaxMinDesign::~MaxMinDesign()
{
    
}

/**
 * Returns a random subsample of X of size n which has minimum (amongst 
 * a set of NSAMPLES similar subsamples) maximum distance between points.
 */     
ivec MaxMinDesign::subsample(mat X, int n)
{
    if (n <= 0)
        cerr << "Invalid sample size in MaxMinDesign::subsample(...)" << endl;
    
    double maxMinDist = 0.0;
    ivec bestSubsample = randperm(X.rows());
    
    for (int i=0; i<nsamples; i++) {
        
    	// Generate random subsample
    	ivec irand = randperm(X.rows());
    	ivec irandn = irand(0,n-1);
        mat S = X.get_rows(irandn);
        
        // Compute max distance in this subsample
        double minDist =  distanceMin(S);

        // Keep subsample if has min max distance
        if (minDist > maxMinDist) {
        	bestSubsample = irandn;
        	maxMinDist = minDist;
        	cout << "New min distance: " << maxMinDist << endl;
        }
    }
    
    return bestSubsample;
}

/**
 * Returns the maximum distance between any 2 points from X
 */
double MaxMinDesign::distanceMin(mat X)
{
   double distMin = norm(X.get_row(1) - X.get_row(2));
   double d;
   
   for (int i=0; i<X.rows(); i++) {
       vec x1 = X.get_row(i);
       for (int j=0; j<i; j++) {
           d = norm(x1 - X.get_row(j));
           if (d<distMin) distMin = d;
       }
   }
   return distMin;
}
