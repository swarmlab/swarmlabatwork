/*
 * demo_sine.cpp
 *
 *  Created on: 1 May 2010
 *      Author: Remi Barillec <r.barillec@aston.ac.uk>
 */
#include "demo_sine.h"

#define NUM_X 200           // X resolution
#define NUM_OBS 20          // Number of observations
#define NUM_ACTIVE 5        // Number of active points
#define NOISE_VAR 0.1      // Observation noise variance

int main(int argc, char* argv[]) {

    // Generate data from a sine
    vec X = linspace(0, 2*pi, 200);
    vec Y = sin(X);                          // True function

    // Randomly select a subset of the data as observations,
    // and add white noise
    ivec Iobs = randperm(NUM_X).left(NUM_OBS);
    vec Xobs = X(Iobs);
    mat XobsMat = mat(Xobs);      // PSGP requires a matrix of inputs (not a vector)
    vec Yobs = Y(Iobs) + sqrt(NOISE_VAR)*randn(NUM_OBS);   // Noisy observations

    // Covariance function: Gaussian + Nugget
    double range = 0.5;           // Initial range
    double sill = 1;              // Initial sill
    double nugget = NOISE_VAR;    // Nugget variance

    GaussianCF   gaussianCovFunc(range, sill);
    WhiteNoiseCF nuggetCovFunc(nugget);
    SumCF covFunc;
    covFunc.add(gaussianCovFunc);
    covFunc.add(nuggetCovFunc);

    // Initialise psgp
    PSGP psgp(XobsMat, Yobs, covFunc, NUM_ACTIVE);

    // Use a Gaussian likelihood model with fixed variance
    GaussianLikelihood gaussLik(nugget);

    // Compute the posterior (first guess), which projects the whole observed data
    // onto a subset of optimal active points.
    psgp.computePosterior(gaussLik);

    // Estimate parameters using Scaled Conjugate Gradients
    SCGModelTrainer scg(psgp);
    scg.Train(10);

    // Recompute the posterior and active points for the new parameters
    psgp.resetPosterior();
    psgp.computePosterior(gaussLik);


    // Make predictions - we use the Gaussian covariance function only
    // to make non-noisy prediction (no nugget)
    vec psgpmean = zeros(NUM_X);
    vec psgpvar  = zeros(NUM_X);
    psgp.makePredictions(psgpmean, psgpvar, mat(X), gaussianCovFunc);

    // Plot results
    GraphPlotter gplot = GraphPlotter();

    // Plot psgp mean and variance
    gplot.plotPoints(X, psgpmean, "psgp mean", LINE, BLUE);
    gplot.plotPoints(X, psgpmean + 2.0*sqrt(psgpvar), "error bar", LINE, CYAN);
    gplot.plotPoints(X, psgpmean - 2.0*sqrt(psgpvar), "error bar", LINE, CYAN);

    // Plot true function and observations
    gplot.plotPoints(X, Y, "true function", LINE, RED);
    gplot.plotPoints(Xobs, Yobs, "observations", CROSS, BLACK);

    // Plot active points
    vec activeX = (psgp.getActiveSetLocations()).get_col(0);
    vec activeY = Yobs(psgp.getActiveSetIndices());
    gplot.plotPoints(activeX, activeY, "active points", CIRCLE, BLUE);

    return 0;
}
