#ifndef GreedyMaxMinDesign_H_
#define GreedyMaxMinDesign_H_

#include "Design.h"

/**
 * GREEDY MAX MIN DISTANCE RANDOM DESIGN
 * 
 * Subsample by adding points
 * 
 *  
 *  
 * 
 */
class GreedyMaxMinDesign : public Design
{
public:
	GreedyMaxMinDesign(double zweight = 3.0);
	virtual ~GreedyMaxMinDesign();
	ivec subsample(mat X, int sample_size);
	
private:
    
    // The number of samples used to determine the optimal sample
    int nsamples;
    
    // ~The relative weight of the Z component
    double zweight;
    
    // Returns the index of the point in X having maximal minimum distance
    // to the points in Y
    vec dist(mat X, vec y);
    
    // Compute the norm of vector v where the last component (the z-component) 
    // can be given a custom weight
    double weightedSquareNorm(vec v);
    
};

#endif /*GreedyMaxMinDesign_H_*/
