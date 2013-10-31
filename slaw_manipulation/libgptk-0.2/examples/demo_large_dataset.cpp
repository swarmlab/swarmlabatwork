/**
 * This demonstration program illustrates the PSGP method in real
 * situation. Data from a simulated radioactive event is interpolated.
 * 
 * (c) 2009, Remi Barillec <r.barillec@aston.ac.uk>  
 **/

#include "demo_large_dataset.h"

int main(void)
{
    // Set random generator seed
    itpp::RNG_reset(123);

    int n_active = 400;
    time_t tstart = clock();
    run(n_active);
    cout << (clock() - tstart)/1e6 << " seconds elapsed" << endl;
    
    return 0;
}    

/**
 * Run the Scenario 1 interpolation exercise with n_active active points
 */
void run(int n_active)
{
    vec psgpmean, psgpvar;
    
    csvstream csv;                           // Input/output from/to CSV file
    
    
    //-------------------------------------------------------------------------
    // LOAD AND EXTRACT DATA
    mat data;
    string filename("INTAMAP_scenario1_rainevent_germany.csv");
    cout << "Loading data" << endl;
    if (csv.read(data, filename.c_str()))
    {
        cout << "Problem encountered while reading file " << filename << endl;
        cout << "Aborting." << endl;
        return;
    }
    
    // Use 80% of the data for training and 20% for testing (validation)
    cout << "Separating training/test set" << endl;
    int n_obs = data.rows();
    int n_train = (int) ceil(0.8*n_obs);
    
    ivec irand = itppext::randperm(n_obs);
    mat datarand = data.get_rows(irand);                    // Shuffle the data points
    
    mat data_trn = datarand.get_rows(0, n_train-1);
    mat data_tst = datarand.get_rows(n_train, n_obs-1);
    
    csv.write(data_tst, "Intamap_scenario1_germany_test.csv");
    csv.write(data_trn, "Intamap_scenario1_germany_train.csv");

    int n_test  = data_tst.rows();
    mat Xtrn = data_trn.get_cols(0, 1);         // Training inputs
    vec Ytrn = data_trn.get_col(2);             // Training outputs
    mat Xtst = data_tst.get_cols(0, 1);         // Testing inputs
    vec Ytst = data_tst.get_col(2);             // Testing outputs
    
    
    //-------------------------------------------------------------------------
    // COVARIANCE FUNCTION
    cout << "Finding sensible starting parameters." << endl;
    
    // A quick first guess for the covariance parameters
    double x1min = min(data.get_col(0));
    double x1max = max(data.get_col(0));
    double x2min = min(data.get_col(1));
    double x2max = max(data.get_col(1));
        
    double r1 = abs( x1max - x1min );
    double r2 = abs( x2max - x2min );
    // double range  = 0.25 * ((r1+r2) / 2.0);
    double range = 0.5*variance(data.get_col(0)) + 0.5*variance(data.get_col(0));
    double sill   = abs(variance(Ytrn));    
    double nugget = 0.1*sill;
    double offset = min(x1min, x2min); 
    		
    // Uniform grid for prediction
    int grid_size = 200;
    double dr = min(r1, r2)/grid_size;
    int n_grid1 = (int) ceil(r1/dr);
    int n_grid2 = (int) ceil(r2/dr);
    mat Xgrid(n_grid1*n_grid2, 2);
    vec x1grid = linspace(x1min, x1max, n_grid1);
    vec x2grid = linspace(x2min, x2max, n_grid2);

    int k=0;
    for (int i=0; i<n_grid1; i++) {
        for (int j=0; j<n_grid2; j++) {
            Xgrid(k,0) = x1grid(i);
            Xgrid(k,1) = x2grid(j);
            k++;
        }
    }

    csv.write(Xgrid, "Intamap_scenario1_germany_grid.csv");
    cout << "Making predictions on gridded data " << x1min << ":" << x1max << " x " << x2min << ":" << x2max << "(" << n_grid1 << "x" << n_grid2 << ")" << endl;
    
    
    // We use a mixture of several kernels for more flexibility
    cout << "Initialising covariance function" << endl;
    GaussianCF  kernel1(range, sill);
    Matern3CF   kernel2(range, sill);
    Matern5CF   kernel3(range, sill);
    NeuralNetCF kernel4(0.1*range, sill, offset);
    ExponentialCF kernel5(range, sill);
    ConstantCF  bias(mean(Ytrn));
    
    SumCF kernelCovFunc(kernel5);           // Put together the sum of various kernels
    kernelCovFunc.add(kernel4);
    // kernelCovFunc.addCovarianceFunction(kernel3);
    // kernelCovFunc.addCovarianceFunction(kernel4);
    // kernelCovFunc.add(kernel5);
    kernelCovFunc.add(bias);
    
    WhiteNoiseCF noise(nugget);                             // And add some white noise to make up the final
    SumCF covFunc(kernelCovFunc);           // covariance function
    covFunc.add(noise);
        
    covFunc.displayCovarianceParameters();
    
    //-------------------------------------------------------------------------
    // Initialise a PSGP for parameter estimation
    // Limit to n_active observations taken from maxi min design
    cout << "Initialising PSGP with " << n_active << " active points" << endl;
    
    MaxMinDesign design;
    ivec isub = design.subsample(Xtrn, n_active);
    mat Xtrn2 = Xtrn.get_rows(isub);
    vec Ytrn2 = Ytrn(isub);
    PSGP psgp_learn(Xtrn2, Ytrn2, covFunc, n_active, 1, 0);
    
    // Use a Gaussian likelihood model with fixed variance (set to 
    // a small percentage of the nugget variance) 
    GaussianLikelihood gaussLik(nugget);
    
    cout << "Compute posterior under Gaussian likelihood" << endl;
    psgp_learn.computePosterior(gaussLik);
    
    // Estimate parameters
    SCGModelTrainer scg(psgp_learn);
    
    int nparams = covFunc.getParameters().length();
    bvec optMask(nparams);
    // for (int i=0; i<nparams; i++) optMask(i) = true; 
    // optMask(nparams-1) = true;
    // scg.setOptimisationMask(optMask);
    scg.checkGradient();
    psgp_learn.setLikelihoodType(Approximate);
    for (int i=0; i<3; i++)
    {
        scg.Train(10);
        scg.checkGradient();
        psgp_learn.recomputePosterior();
    }
        
//    psgp_learn.setLikelihoodType(UpperBound);
//    for (int i=0; i<2; i++) 
//    {
//        scg.Train(10);
//        psgp_learn.recomputePosterior();
//    } 
        
    covFunc.displayCovarianceParameters();
    
    // Use a PSGP with all observations for prediction
    PSGP psgp(Xtrn, Ytrn, covFunc, n_active);
    psgp.computePosterior(gaussLik);
    
    // Prediction on the test set
    cout << "Making predictions on test data (" << n_test << " locations)" << endl;
    vec tstmean = zeros(n_test);
    vec tstvar  = zeros(n_test);
    psgp.makePredictions(tstmean, tstvar, Xtst, kernelCovFunc);
        
    // Prediction on a uniform grid (square grid cells)
    vec gridmean = zeros(n_grid1*n_grid2);
    vec gridvar  = zeros(n_grid1*n_grid2);
    
    cout << "Making predictions on gridded data " << x1min << ":" << x1max << " x " << x2min << ":" << x2max << "(" << n_grid1 << "x" << n_grid2 << ")" << endl;
    psgp.makePredictions(gridmean, gridvar, Xgrid, covFunc);
    
    
    //---------------------------------------------------------------------
    // Store data for Matlab plotting
    //-------------------------------------------------------------------------
    cout << "Saving results" << endl;
    // PSGP results on grid
    mat psgpgrid = zeros(4, n_grid1*n_grid2);
    psgpgrid.set_row(0, Xgrid.get_col(0));
    psgpgrid.set_row(1, Xgrid.get_col(1));
    psgpgrid.set_row(2, gridmean);
    psgpgrid.set_row(3, gridvar);
    csv.write(psgpgrid, "demo_large_psgp_grid.csv");
    
    // PSGP results on test set
    mat psgptest = zeros(5, n_test);
    psgptest.set_row(0, Xtst.get_col(0));
    psgptest.set_row(1, Xtst.get_col(1));
    psgptest.set_row(2, tstmean);
    psgptest.set_row(3, tstvar);
    psgptest.set_row(4, Ytst);
    csv.write(psgptest, "demo_large_psgp_test.csv");
    
    cout << "Done" << endl;
}


