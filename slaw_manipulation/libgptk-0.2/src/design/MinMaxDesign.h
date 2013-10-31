#ifndef MINMAXDESIGN_H_
#define MINMAXDESIGN_H_

#include "Design.h"

/**
 * MIN MAX DISTANCE RANDOM DESIGN
 * 
 * This returns a random subsample from a set of locations
 * in which the maximum distance between any two points
 * is minimised. The minimisation is achieved by taking the "best" 
 * subsample (with respect to the max distance between points) 
 * out of NSAMPLE similar random subsamples.
 */
class MinMaxDesign : public Design
{
public:
	MinMaxDesign(int nsamples = 100);   // Pass in the number of subsamples for minimisation
	~MinMaxDesign();
	ivec subsample(mat X, int sample_size);
	
private:
    
    // The number of samples used to determine the optimal sample
    int nsamples;
    
    // Returns the maximum distance between points from X
    double distanceMax(mat X);
    
    
    
};

#endif /*MINMAXDESIGN_H_*/
