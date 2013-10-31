#ifndef MATERN3CF_H_
#define MATERN3CF_H_

#include "StationaryCF.h"

#include <itpp/itbase.h>
#include <itpp/stat/misc_stat.h>

/**
 * Isotropic Matern covariance function with nu=3/2.
 * 
 * (C) 2009 Remi Barillec <r.barillec@aston.ac.uk>
 */ 
class Matern3CF : public StationaryCF
{
public:
    Matern3CF(double variance, double lengthScale);
    virtual ~Matern3CF();

protected:
    virtual double correlation(double sqDist) const;
    virtual double correlationGradient(int parameterNumber, double sqDist) const;
};


#endif /*MATERN3CF_H_*/
