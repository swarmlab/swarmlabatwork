#ifndef DESIGN_H_
#define DESIGN_H_

#include <itpp/itbase.h>
#include <itppext/itppext.h>

using namespace std;
using namespace itpp;
using namespace itppext;

class Design
{
public:
	
    virtual ~Design() {};
    
    // Subsample sample_size locations from X 
	virtual ivec subsample(mat X, int sample_size) = 0;
};

#endif /*DESIGN_H_*/
