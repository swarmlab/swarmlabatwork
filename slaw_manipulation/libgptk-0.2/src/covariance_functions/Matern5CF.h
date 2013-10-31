#ifndef MATERN5CF_H_
#define MATERN5CF_H_

#include "StationaryCF.h"


/**
 * Isotropic Matern covariance function with nu=5/2.
 * 
 * (C) 2009 Remi Barillec <r.barillec@aston.ac.uk>
 */ 
class Matern5CF : public StationaryCF
{
public:
    Matern5CF(double variance, double lengthScale);
    virtual ~Matern5CF();

protected:
    virtual double correlation(double sqDist) const;
    virtual double correlationGradient(int parameterNumber, double sqDist) const;

};

#endif /*MATERN5CF_H_*/
    
