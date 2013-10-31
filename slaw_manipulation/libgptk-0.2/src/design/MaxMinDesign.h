#ifndef MAXMINDESIGN_H_
#define MAXMINDESIGN_H_

#include "Design.h"

/**
 * MAX MIN DISTANCE RANDOM DESIGN
 * 
 * This returns a random subsample from a set of locations
 * in which the minimum distance between any two points
 * is maximised. The maximisation is achieved by taking the "best" 
 * subsample (with respect to the min distance between points) 
 * out of NSAMPLE similar random subsamples.
 */
class MaxMinDesign : public Design
{
public:
	MaxMinDesign(int nsamples = 100);   // Pass in the number of subsamples for minimisation
	virtual ~MaxMinDesign();
	ivec subsample(mat X, int sample_size);
	
private:
    
    // The number of samples used to determine the optimal sample
    int nsamples;
    
    // Returns the minimum distance between points from X
    double distanceMin(mat X);
    
};

#endif /*MAXMINDESIGN_H_*/
