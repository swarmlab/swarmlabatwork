/**
 * This demonstration program illustrates the functioning of the 
 * PSGP algorithm. Noisy observations are sampled from a Gaussian Process
 * with known parameters. We then display the posterior distribution as 
 * the number of active points is increased.
 * 
 * (c) 2009, Remi Barillec <r.barillec@aston.ac.uk>  
 **/

#include "demo_active_set.h"

int main(void)
{
    vec Xtrn, Xtst, Ytrn, Ytst;
    vec psgpmean, psgpvar, gpmean, gpvar;
    
    GraphPlotter gplot = GraphPlotter();
    csvstream csv;                           // Input/output to CSV file
    stringstream filename;                   // Filename
    
    // True GP parameters
    double range  = 5.0;               // The range or length scale of the GP
    double sill   = 3.0;               // The sill or variance of the GP
    double nugget = 0.01;              // The noise variance
    double noisevar = 0.01;
    
    // Covariance function: Gaussian + Nugget
    GaussianCF   gaussianCovFunc(range, sill);            
    WhiteNoiseCF nuggetCovFunc(nugget);
    SumCF covFunc;
    covFunc.add(gaussianCovFunc);
    covFunc.add(nuggetCovFunc);

    vec initParams = covFunc.getParameters();
    
    //-------------------------------------------------------------------------
    // Generate some data from GP with known parameters
    int n_points = 400;
    Xtst = linspace(-40,40, n_points);
    mat K = zeros(Xtst.length(), Xtst.length());
    gaussianCovFunc.covariance(K, Xtst);              // Covariance of the inputs
    Ytst = (chol(K+0.0001*eye(n_points))).transpose()*randn(n_points);   // Outputs

    // Training set - use a random subsample of the data
    int n_train = 64;
    ivec itrn = to_ivec(linspace(0,Xtst.length()-1,n_train));
    Xtrn = Xtst(itrn);
    Ytrn = Ytst(itrn) + sqrt(noisevar)*randn(n_train);
    mat Xtrnmat = Xtrn;
    
    //-------------------------------------------------------------------------
    // Use a full GP to fit the data - for comparison with PSGP
    GaussianProcess gp(1, 1, Xtrnmat, Ytrn, covFunc);
    gpmean = zeros(n_points);
    gpvar = zeros(n_points);
    
    // Optimise the GP's parameters
    SCGModelTrainer scggp(gp);
    scggp.Train(15);

    // Compute the posterior GP
    gp.makePredictions(gpmean, gpvar, Xtst, gaussianCovFunc);
    
    //-------------------------------------------------------------------------
    // Initialise the PSGP
    int n_active = 8;    // Start with 8 active points

    covFunc.setParameters(initParams);
    PSGP psgp(Xtrnmat, Ytrn, covFunc, n_active, 1, 3);
    psgp.setGammaTolerance(1e-10);
    
    // Use a Gaussian likelihood model with fixed variance (set to 
    // the nugget variance) 
    GaussianLikelihood gaussLik(nugget);
    psgp.computePosterior(gaussLik);
    
    
    // Compute the PSGP posterior distribution under the Gaussian likelihood
    // for an increasing number of active points
    for (int i=0; i<4; i++) 
    {
        // Reset covariance function parameters
        covFunc.setParameters(initParams);

        // Recompute the posterior - the best active points are selected from
        // the 40 points in the training set
        for (int j=0; j<3; j++)
        {
            // Optimise the parameters
            SCGModelTrainer scg(psgp);
            // scg.setCheckGradient(true);         // Check the gradients
            scg.Train(5);

            // Recompute the posterior (including selection of active points)
            psgp.resetPosterior();
            psgp.computePosterior(gaussLik);
        }

                
        //---------------------------------------------------------------------
        cout << "PSGP with " << n_active << " active points" << endl;

        // Note that we are now using the covariange function without the
        // nugget term for prediction.
        psgpmean = zeros(n_points);
        psgpvar  = zeros(n_points);
        psgp.makePredictions(psgpmean, psgpvar, mat(Xtst), gaussianCovFunc);

        // Plot PSGP mean and error bars
        gplot.clearPlot();
        gplot.plotPoints(Xtst, psgpmean, "psgp mean", LINE, BLUE);
        gplot.plotPoints(Xtst, psgpmean + 2.0*sqrt(psgpvar), "error bar", LINE, CYAN);
        gplot.plotPoints(Xtst, psgpmean - 2.0*sqrt(psgpvar), "error bar", LINE, CYAN);

        // Plot full GP for comparison
        gplot.plotPoints(Xtst, gpmean, "gp mean", LINE, GREEN);
        gplot.plotPoints(Xtst, gpmean + 2.0*sqrt(gpvar), "gp var", LINE, GREEN);
        gplot.plotPoints(Xtst, gpmean - 2.0*sqrt(gpvar), "gp var", LINE, GREEN);
        
        // Plot observations and true function
        gplot.plotPoints(Xtrn, Ytrn, "training set", CROSS, RED);  
        gplot.plotPoints(Xtst, Ytst, "true function", LINE, RED);

        // Plot active points
        vec activeX = (psgp.getActiveSetLocations()).get_col(0);
        vec activeY = Ytrn(psgp.getActiveSetIndices());
        gplot.plotPoints(activeX, activeY, "active points", CIRCLE, BLUE);
        
        // Increase the number of active point for next iteration
        n_active *= 2;
        psgp.setActiveSetSize(n_active);

        cout << "Press a key to continue" << endl;
        getchar();
    }

    return 0;   
}
