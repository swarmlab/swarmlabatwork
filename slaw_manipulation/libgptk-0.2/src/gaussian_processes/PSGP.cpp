#include "PSGP.h"

/**
 * Constructor
 * 
 * Parameters
 * 
 * X             Matrix of inputs (locations)
 * Y             Vector outputs (observations)
 * nActivePoints Maximum number of active points 
 * _iterChancing Number of sweeps through the data with replacement 
 *               of active points (default: 1)
 * _iterFixed    Number of sweeps through the data with fixed active 
 *               set (default :2)
 */
PSGP::PSGP(mat& X, vec& Y, CovarianceFunction& cf, int nActivePoints, int _iterChanging, int _iterFixed) 
: ForwardModel(X.cols(), 1), Locations(X), Observations(Y), covFunc(cf)
{
    assert(Locations.rows() == Observations.size());

    maxActiveSet = nActivePoints;
    epsilonTolerance = 1e-6;
    gammaTolerance = 1e-3;
    
    
    iterChanging = _iterChanging;
    iterFixed = _iterFixed;

    nObs = Locations.rows();

    likelihoodType = Approximate;
    
    resetPosterior();
    
    // Which version of the implementation to use. Should be using V3 
    // as it is the fastest, but I have left the other versions for
    // backward comparison/debugging, should they be needed.
    algoVersion = ALGO_V3;
}


/**
 * Destructor
 */
PSGP::~PSGP()
{
}


/**
 * Specify the active set (by indices).
 * This resets the current active set and initialises the posterior for
 * the specified active set.
 */
void PSGP::setActiveSet(ivec activeIndexes, mat activeLocations) 
{
    assert(activeIndexes.length() == activeLocations.rows());
    
    resetPosterior();
    
    // Set active indexes and locations
    idxActiveSet = activeIndexes;
    ActiveSet = activeLocations;
    sizeActiveSet = activeIndexes.length();
    
    // Disable replacement of active set in posterior computation
    iterChanging = 0;
}


/**
 * Compute posterior with a single likelihood model
 */
void PSGP::computePosterior(const LikelihoodType& noiseModel)
{
    bool fixActiveSet = false;

    // Cycle several times through the data, first allowing the active
    // set to change (for iterChanging iterations) and then fixing it
    // (for iterFixed iterations)
    for(int cycle = 1; cycle <= (iterChanging + iterFixed); cycle++)
    {
        if(cycle > iterChanging) fixActiveSet = true;
        
        // Present observations in a random order
        ivec randObsIndex = itppext::randperm(nObs);
        
        for(int i=0; i<nObs; i++)	
        {
            cout << "\rProcessing observation: " << i+1 << "/" << nObs  << flush;
            processObservationEP(randObsIndex(i), noiseModel, fixActiveSet);
        }
        cout << endl;
    }
}


/**
 * Compute posterior with a different likelihood model for each observation
 */
void PSGP::computePosterior(const ivec& modelIndex, const Vec<LikelihoodType *> noiseModel)
{
    // Check if we have an index and a model per observation
    assert(nObs == modelIndex.length()); 

    bool fixActiveSet = false;

    // Cycle several times through the data, first allowing the active
    // set to change (for iterChanging iterations) and then fixing it
    // (for iterFixed iterations)
    for(int cycle = 1; cycle <= (iterChanging + iterFixed); cycle++)
    {
        if(cycle > iterChanging) fixActiveSet = true;

        // Present observations in a random order
        ivec randObsIndex = itppext::randperm(nObs);

        for(int iObs=0; iObs<nObs; iObs++)	
        {
            int iModel = modelIndex(randObsIndex(iObs));
            cout << "\rProcessing observation: " << iObs+1 << "/" << nObs << flush;
            // cout << " using likelihood model " << iModel << flush;
            
            assert(iModel < noiseModel.length() && iModel >= 0);
            
            processObservationEP(randObsIndex(iObs), *noiseModel(iModel), fixActiveSet);
        }
        cout << endl;
    }
}


/**
 * This is the core method, implementing the sparse EP algorithm
 * 
 * This algorithm follows Appendix G in in Lehel Csato's PhD thesis 
 * "Gaussian Processes - Iterative Sparse Approximations", 2002,
 * NCRG, Aston University.
 * 
 * Further reference numbers correspond to sections/equations in the same
 * document. 
 */
void PSGP::processObservationEP(const int iObs, const LikelihoodType &noiseModel, const bool fixActiveSet) 
{
    double sigmaLoc;                // Auto-covariance of location
    vec k = zeros(sizeActiveSet);   // Covariance between location and active set
    
    double r, q;                    // Online coefficients
    double cavityMean, cavityVar;   // Cavity mean and variance
    double gamma;                   // Mean of ??
    vec    eHat;                    // Variance of ??
    
    // Retrieve location and observation at index iObs
    vec loc    = Locations.get_row(iObs);
    double obs = Observations(iObs);
    
    // Remove previous contribution of observation iObs
    // Appendix G.(a)
    EP_removePreviousContribution(iObs);
    
    // Intermediate computations: covariances, cavity mean and variance
    // Appendix G.(c)
    EP_updateIntermediateComputations(cavityMean, cavityVar, sigmaLoc, k, gamma, eHat, loc);
    
    // Compute the updated q and r online coefficients for the specified 
    // likelihood model, using the updated alpha and C.
    // Appendix G.(b), Sec. 2.4, Eq. 2.42, 3.28
    double logEvidence = noiseModel.updateCoefficients(q, r, obs, cavityMean, cavityVar);
    
    // stabiliseCoefficients(q, r, cavityMean, cavityVar, 1e3, 1e-6);
    
    // Update EP parameters
    EP_updateEPParameters(iObs, q, r,  cavityMean, cavityVar, logEvidence);
            
    // Perform full or sparse update depending on geometry (gamma)
    // Appendix G.(e)
    if (gamma >= gammaTolerance*sigmaLoc && !fixActiveSet)
    {
        //----------------------------------------------
        // Full update
        //----------------------------------------------
        cout << " (full update)";
        
        if (sizeActiveSet < maxActiveSet)
        {
            // Add observation to active set and update parameters 
            addActivePoint(iObs, q, r, k, sigmaLoc, gamma, eHat);
        }
        else
        {
            switch(algoVersion) {
            
            case ALGO_V1:
            {
                // Add observation to active set
                addActivePoint(iObs, q, r, k, sigmaLoc, gamma, eHat);

                // Compute scores for each active point
                vec scores = scoreActivePoints(FullKL);

                // Remove active point with lowest score, and update alpha, C, Q and P
                int swapCandidate = min_index(scores);
                deleteActivePoint(swapCandidate);
                break;
            }
            case ALGO_V2:
                // Swap observation with one existing active point if
                // the swap results in an improved active set
                addActivePointAugmented_v1(iObs, q, r, k, sigmaLoc, gamma, eHat);
                break;
                
            default: // This also covers the case ALGO_V3
                // Swap observation with one existing active point if
                // the swap results in an improved active set
                addActivePointAugmented_v2(iObs, q, r, k, sigmaLoc, gamma, eHat);
            }
        }
        
    }
    else
    {
        //----------------------------------------------
        // Sparse update
        //----------------------------------------------
        P.set_row(iObs, eHat);
        
        vec s;
        if (sizeActiveSet>0) s = C*k;
        s += eHat;
        
        // Update GP parameters - same as full update case, but with scaling
        // Appendix G.(f)
        double eta = 1.0 / (1.0 + gamma*r);  // Scaling factor
        alpha += eta * s * q;
        C     += r * eta * outer_product(s, s, false);
    }
    
    
    // Remove unneeded active points based on geometry
    EP_removeCollapsedPoints();
}


/**
 * Substract contribution of observation iObs from previous iterations 
 * 
 * Appendix G.(a), Eq. 4.19 and 4.20
 */
void PSGP::EP_removePreviousContribution(int iObs) 
{
    if (varEP(iObs) > LAMBDA_TOLERANCE)   
    {
        vec p = P.get_row(iObs);
        vec Kp = KB * p;

        // Update alpha and C
        vec h  = C * Kp + p;
        double nu = varEP(iObs) / ( 1.0 - varEP(iObs) * dot(Kp,h) );
        alpha += h * nu * ( dot(alpha, Kp) - meanEP(iObs) );
        C += nu * outer_product(h,h);
    }
}


/**
 * Compute cavity mean and variance for specified input/location x
 * 
 * Appendix G.(c), Eq. 3.3
 */
void PSGP::EP_updateIntermediateComputations(double &cavityMean, double &cavityVar, double &sigmaLoc,
                                              vec &k, double &gamma, vec &eHat, vec loc) 
{
    assert(k.length() == sizeActiveSet);
    
    covFunc.covariance(sigmaLoc, loc);           // Auto-variance of location
    
    if(sizeActiveSet == 0)
    {
        cavityVar = sigmaLoc;
        cavityMean = 0.0;
        eHat = zeros(0);
        gamma = sigmaLoc;
    } 
    else 
    {
        covFunc.covariance(sigmaLoc, loc);           // Auto-variance of location
        covFunc.covariance(k, ActiveSet, loc);      // cov(location, active set)
        cavityVar = sigmaLoc + dot(k, C * k);
        cavityMean = dot(k, alpha);
        
        // This way of computing eHat is more robust - but much more 
        // computationaly expensive, as we already store Q = inv(KB)
        // eHat = backslash(KB, k);
        eHat = Q*k;
        gamma = sigmaLoc - dot(k, eHat);
    }
}


/**
 * Update the EP parameters (Eq. 4.18)
 */
void PSGP::EP_updateEPParameters(int iObs, double q, double r, double cavityMean, double cavityVar, 
                                 double logEvidence)
{
    double ratio = q / r;
    logZ(iObs)= logEvidence + (log(2.0 * pi) - log(abs(r)) - (q * ratio)) / 2.0;
    meanEP(iObs) = cavityMean - ratio;
    varEP(iObs) = -r / (1.0 + (r * cavityVar));
}


/**
 * Add a given location to the active set
 */
void PSGP::addActivePoint(int iObs, double q, double r, vec k, double sigmaLoc, double gamma, vec eHat)
{
    vec s;
    if (sizeActiveSet>0) s = C*k;
    
    // Increase size of active set
    sizeActiveSet++; 
   
    // Append index of observation to indexes in active set
    idxActiveSet.set_size(sizeActiveSet, true);
    idxActiveSet(sizeActiveSet - 1) = iObs;
    
    // Increase storage size for active set and store new observation
    // ActiveSet.set_size(sizeActiveSet, Locations.cols(), true);
    // ActiveSet.set_row(sizeActiveSet - 1, Locations.get_row(iObs));
    ActiveSet.append_row(Locations.get_row(iObs));

    // Increase size of C and alpha
    alpha.set_size(sizeActiveSet, true);
    C.set_size(sizeActiveSet, sizeActiveSet, true);
    
    // e is the unit vector for dimension sizeActiveSet
    vec e = zeros(nObs);
    e(iObs) = 1.0;
    
    // Update P matrix
    P.append_col(e);
    
    // Update KB matrix - add auto and cross covariances
    KB.append_col(k);
    KB.append_row( concat(k, sigmaLoc) );
    
    // update Q = inv(KB) matrix
    eHat.set_size(sizeActiveSet, true);
    eHat(sizeActiveSet - 1) = -1.0;
    Q.set_size(sizeActiveSet, sizeActiveSet, true);
    Q += outer_product(eHat, eHat) / gamma;
    
    // Update GP parameters
    // Appendix G.(f)
    s.set_size(sizeActiveSet, true);     // Increase size of s and append 1.0
    s(sizeActiveSet-1) = 1.0; 
    
    alpha += s * q;
    C     += r * outer_product(s, s, false);
}


/**
 * Swap current observation with an existing active point if
 * this results in an improved active set.
 * 
 * What we do is build an extended active set (of size maxActiveSet+1) and then remove
 * the active point with the lowest score to get back to size maxActiveSet. Instead of
 * adding/removing, we use temporary augmented matrices and only update the original
 * matrices if a swap is necessary. This avoids expensive memory operations. 
 */
void PSGP::addActivePointAugmented_v1(int iObs, double q, double r, vec k, double sigmaLoc, double gamma, vec eHat)
{
    assert(sizeActiveSet == maxActiveSet);
    
    // Initialise augmented matrices
    vec z = zeros(maxActiveSet+1);

    idxActiveSet_aug = concat(idxActiveSet, iObs);
    alpha_aug = concat(alpha, 0.0);
    
    ActiveSet_aug.set_submatrix(0, 0, ActiveSet);
    P_aug.set_submatrix(0, 0, P);
    Q_aug.set_submatrix(0, 0, Q);
    C_aug.set_submatrix(0, 0, C);
    KB_aug.set_submatrix(0, 0, KB);
    
    Q_aug.set_row(maxActiveSet, z);
    Q_aug.set_col(maxActiveSet, z);
    C_aug.set_row(maxActiveSet, z);
    C_aug.set_col(maxActiveSet, z);
            
    // Add current observation to augmented active set
    ActiveSet_aug.set_row(maxActiveSet, Locations.get_row(iObs));
    
    // e is the unit vector for dimension sizeActiveSet
    vec e = zeros(nObs);
    e(iObs) = 1.0;
    P_aug.set_col(maxActiveSet, e);
    
    // Update KB matrix - add auto and cross covariances
    vec newk = concat(k, sigmaLoc);
    KB_aug.set_col(maxActiveSet, newk);
    KB_aug.set_row(maxActiveSet, newk);
    
    // update Q = inv(KB) matrix
    vec eHat_aug = concat(eHat, -1.0);
    Q_aug += outer_product(eHat_aug, eHat_aug) / gamma;
    
    // Update GP parameters
    // Appendix G.(f)
    vec s = concat(C*k, 1.0);
    
    alpha_aug += s * q;
    C_aug     += r * outer_product(s, s, false);
    
    // Determine which point in the extended active set we need to remove
    // to get back to the maximum size. If this point is not the point we
    // we just added, swap it with the current observation.
    
    // Compute scores for each active point
    vec scores = scoreActivePoints(FullKL);
    
    // Remove active point with lowest score, and update alpha, C, Q and P
    int swapCandidate = min_index(scores);
    swapActivePoint_v1(swapCandidate);
}


/**
 * Swap current observation with an existing active point if
 * this results in an improved active set.
 * 
 * Typically, we:
 * - add a new observation to the active set (but store its effect 
 *   separately)
 * - remove one observation from the new, extended active set (to
 *   get back to the maximum size allowe)
 * - update the C and alpha to reflect the addition/removal of an 
 *   observation. However, we do it in such a way that there is minimal
 *   use of memory and no resizing of C and alpha.
 */
void PSGP::addActivePointAugmented_v2(int iObs, double q, double r, vec k, double sigmaLoc, double gamma, vec eHat)
{
    assert(sizeActiveSet == maxActiveSet);
    
    ActiveSet_new = Locations.get_row(iObs);
    idxActiveSet_new = iObs;
    
    P_new = zeros(nObs);
    P_new(iObs) = 1.0;
    
    alpha_new = q;
    
    vec s = C*k;
    C_new = r*s;
    c_new = r;

    Q_new = -eHat/gamma;
    q_new = 1.0/gamma;
    
    KB_new = k;
    kb_new = sigmaLoc;

    // Update parameters for current active set
    alpha += q*s;
    C     += r*outer_product(s,s,false);
    Q     += outer_product(eHat,eHat)/gamma;
    

    // Determine which point in the extended active set we need to remove
    // to get back to the maximum size. If this point is not the point we
    // we just added, swap it with the current observation.

    // Compute scores for each active point
    vec scores = scoreActivePoints(FullKL);

    // Remove active point with lowest score, and update alpha, C, Q and P
    int swapCandidate = min_index(scores);
    swapActivePoint_v2(swapCandidate);    
}


/**
 * Delete an active point from the active set
 *
 * @params:
 *  iObs    The index of the active point to be deleted
 */
void PSGP::deleteActivePoint(int iObs)
{
    // Elements for iObs (correspond to the * superscripts in Csato)
    double alpha_i = alpha(iObs);
    double c_i     = C(iObs, iObs);
    double q_i     = Q(iObs, iObs);
    vec    P_i     = P.get_col(iObs);

    // Covariance between iObs and other active points
    vec    C_i     = C.get_row(iObs);
    vec    Q_i     = Q.get_row(iObs);
    C_i.del(iObs);  // Delete cov(iObs, iObs), we only 
    Q_i.del(iObs);  // want the cross terms
    
    // Updated elements without iObs (correspond to the "r" superscripts in Csato)
    alpha.del(iObs);
    C.del_col(iObs);
    C.del_row(iObs);
    Q.del_col(iObs);
    Q.del_row(iObs);
    P.del_col(iObs);

    // Update new (reduced) elements
    alpha -= ( alpha_i / (c_i + q_i) ) * (Q_i + C_i);    // Eq. 3.19
    mat QQq = outer_product( Q_i, Q_i ) / q_i;
    C += QQq - outer_product( Q_i+C_i, Q_i+C_i ) / ( q_i+c_i );
    Q -= QQq;
    P -= outer_product( P_i, Q_i) / q_i;
    
    // Update Gram matrix
    KB.del_row(iObs);
    KB.del_col(iObs);
    
    // Update active set
    ActiveSet.del_row(iObs);
    idxActiveSet.del(iObs);
    sizeActiveSet--;
}


/**
 * Swap specified active point (iObs) with new active point 
 * (last index in augmented matrices)
 */
void PSGP::swapActivePoint_v1(int iDel)
{
    // Swap iObs with last active point in augmented matrices
    int iObs = maxActiveSet;
    
    if (iDel != maxActiveSet) 
    { 
        C_aug.swap_cols(iObs, iDel);
        C_aug.swap_rows(iObs, iDel);
        Q_aug.swap_cols(iObs, iDel);
        Q_aug.swap_rows(iObs, iDel);
        KB_aug.swap_cols(iObs, iDel);
        KB_aug.swap_rows(iObs, iDel);
        P_aug.swap_cols(iObs, iDel);
        double alpha_last = alpha_aug(iObs);
        alpha_aug(iObs) = alpha_aug(iDel);
        alpha_aug(iDel) = alpha_last;
        P.set_col(iDel, P_aug.get_col(iDel));
        
        // Update Gram matrix
        vec k_add = KB_aug.get_col(iDel);
        k_add.del(iObs);
        
        KB.set_col(iDel, k_add);
        KB.set_row(iDel, k_add);
        
        // Update active set
        ActiveSet.set_row(iDel, ActiveSet_aug.get_row(maxActiveSet));
        idxActiveSet(iDel) = idxActiveSet_aug(maxActiveSet);
    }
    
    alpha = alpha_aug(0,iObs-1);
    C = C_aug(0, iObs-1, 0, iObs-1);
    Q = Q_aug(0, iObs-1, 0, iObs-1);
        
    // Elements for point to be deleted (correspond to the * superscripts in Csato)
    double alpha_i = alpha_aug(iObs);
    
    double c_i     = C_aug(iObs, iObs);
    double q_i     = Q_aug(iObs, iObs);
    vec    P_i     = P_aug.get_col(iObs);
    
    // Covariance between element to be removed and other active points
    vec    C_i     = C_aug.get_row(iObs);
    vec    Q_i     = Q_aug.get_row(iObs);
    C_i.del(iObs);  // Delete cov(iObs, iObs), we only 
    Q_i.del(iObs);  // want the cross terms
    
    // Update new (reduced) elements
    alpha -= ( alpha_i / (c_i + q_i) ) * (Q_i + C_i);    // Eq. 3.19
    mat QQq = outer_product( Q_i, Q_i ) / q_i;
    C += QQq - outer_product( Q_i+C_i, Q_i+C_i ) / ( q_i+c_i );
    Q -= QQq;
    P -= outer_product( P_i, Q_i) / q_i;    
}

/**
 * Replaces given active point (iDel) with new active point,
 * for which data has been stored previously (in addActivePointAugmented)
 * in the vectors and matrices *_new.
 * 
 * We proceed by swapping the point to be deleted with the new active point,
 * and then perform the update as in Csato. The advantage of this method is
 * it does not requires (expensive) resizing of matrices.
 */   
void PSGP::swapActivePoint_v2(int iDel)
{
    double alpha_i, c_i, q_i;
    vec P_i, C_i, Q_i;
 
    assert(sizeActiveSet == maxActiveSet);
    
    if (iDel == maxActiveSet) 
    {
        alpha_i = alpha_new;
        q_i = q_new;
        c_i = c_new;

        P_i = P_new;
        Q_i = Q_new;
        C_i = C_new;
    }
    else
    {
        // Elements for point to be deleted (correspond to the * superscripts in Csato)
        alpha_i = alpha(iDel);
        c_i     = C(iDel, iDel);
        q_i     = Q(iDel, iDel);
        P_i     = P.get_col(iDel);
    
        // Covariance between element to be removed and other active points
        C_i     = C.get_row(iDel);
        Q_i     = Q.get_row(iDel);
    
        C_i(iDel) = C_new(iDel);   // Replace cov(iDel, iDel) with  
        Q_i(iDel) = Q_new(iDel);   // cov(iDel, new point)
    
        C_new(iDel) = c_new;       // cov(iDel, new point) is now
        Q_new(iDel) = q_new;       // cov(new point, new point)
    

        // Swap point to be removed with new point 
        alpha(iDel) = alpha_new;
                
        C.set_col(iDel, C_new);
        C.set_row(iDel, C_new);
        
        Q.set_col(iDel, Q_new);
        Q.set_row(iDel, Q_new);
        
        P.set_col(iDel, P_new);
        
        // Update Gram matrix
        KB_new(iDel) = kb_new;
        KB.set_col(iDel, KB_new);
        KB.set_row(iDel, KB_new);
        
        // Update active set
        ActiveSet.set_row(iDel, ActiveSet_new);
        idxActiveSet(iDel) = idxActiveSet_new;
    }
    
        
    // Update new (reduced) elements
    alpha -= ( alpha_i / (c_i + q_i) ) * (Q_i + C_i);    // Eq. 3.19
    mat QQq = outer_product( Q_i, Q_i ) / q_i;
    C += QQq - outer_product( Q_i+C_i, Q_i+C_i ) / ( q_i+c_i );
    Q -= QQq;
    P -= outer_product( P_i, Q_i) / q_i;
}


/**
 * Remove active points that might have become unnecessary (based on
 * geometry criterion)  
 */
void PSGP::EP_removeCollapsedPoints()
{
    while(sizeActiveSet > 0)
    {
        vec scores = scoreActivePoints(Geometric);
        int removalCandidate = min_index(scores);
        
        if(scores(removalCandidate) >= (gammaTolerance / 1000.0))
        {
            break;
        }
        deleteActivePoint(removalCandidate);
    }    
}


/**
 * Score active points according to scoring method
 */
vec PSGP::scoreActivePoints(ScoringMethod sm)
{
    vec diagC, diagS, term1, term2, term3;
    vec diagInvGram, a;

    // Extract relevant part from either augmented matrices or original ones
    switch (algoVersion) 
    {
    case ALGO_V1:
        a = alpha;
        diagC = diag(C);
        diagInvGram = diag(Q);
        break;
        
    case ALGO_V2:
        diagInvGram = diag(Q_aug);
        a = alpha_aug;
        diagC = diag(C_aug);
        break;
        
    default:   // This also covers the case ALGO_V3
        a = concat(alpha, alpha_new);
        diagC = concat(diag(C), c_new);
        diagInvGram = concat(diag(Q), q_new);
    }
    
    
    switch(sm)
    {
    case Geometric: 
        return(1.0 / diagInvGram);

    case MeanComponent : 
        return(elem_div(elem_mult(a,a), diagC + diagInvGram));

    case FullKL : // Lehel: Eq. 3.23 
    {
        switch (algoVersion) 
        {
        case ALGO_V1:
            // diagS = diag((P.transpose() * diag(varEP)) * P);
            diagS = zeros(P.cols());
            for (int i=0; i<P.cols(); i++) {
                diagS(i) = elem_mult_sum(varEP, elem_mult(P.get_col(i),P.get_col(i)));
            }
            break;

        case ALGO_V2:
            diagS = zeros(P_aug.cols());
            for (int i=0; i<P_aug.cols(); i++) {
                diagS(i) = elem_mult_sum(varEP, elem_mult(P_aug.get_col(i),P_aug.get_col(i)));
            }
            break;

        default:   // This also covers the case ALGO_V3
            diagS = zeros(P.cols()+1);
            for (int i=0; i<P.cols(); i++) {
                diagS(i) = elem_mult_sum(varEP, elem_mult(P.get_col(i),P.get_col(i)));
            } 
            diagS(P.cols()) = elem_mult_sum(varEP, elem_mult(P_new,P_new));
        }

        term1 = elem_div(elem_mult(a,a), diagC + diagInvGram);
        term2 = elem_div(diagS, diagInvGram);
        term3 = log(1.0 + elem_div(diagC, diagInvGram));
        return (term1 + term2 + term3);
    }
    
    default : 
        cerr << "Unknown scoring method" << endl;
        break;
    }

    return zeros(diagInvGram.length());
}


/**
 * Check update coefficients to ensure stability
 */
void PSGP::stabiliseCoefficients(double& q, double& r, double cavityMean, double cavityVar, double upperTolerance, double lowerTolerance)
{
    double sqrtPt = sqrt(cavityVar);
    double tu = -sqrtPt * r * sqrtPt; 
    bool mod = false;
    if(tu > upperTolerance)
    {
        tu = upperTolerance;
        cout << "Shrinking lambda" << endl;
        mod = true;
    }

    if(tu < lowerTolerance)
    {
        tu = lowerTolerance;
        cout << "(REV) Shrinking lambda" << endl;
        mod = true;
    }

    if(mod) {
        r = - (tu / sqrtPt) / tu;
        r = r + itpp::eps;
        r = r + r;
    }
}


/**
 * Make predictions
 */
void PSGP::makePredictions(vec& Mean, vec& Variance, const mat& Xpred, CovarianceFunction& cf) const
{
    assert(Mean.length() == Variance.length());
    assert(Xpred.rows() == Mean.length());

    // Predictive mean
    mat ktest(Xpred.rows(), sizeActiveSet); 
    cf.covariance(ktest, Xpred, ActiveSet);
    Mean = ktest*alpha;
    
    // Predictive variance
    vec kstar(Xpred.rows());
    cf.computeDiagonal(kstar, Xpred);
    Variance = kstar + sum(elem_mult((ktest * C), ktest), 2);
}


/**
 * Same as above, but using the current (stored) covariance function to make
 * the predictions.
 **/
void PSGP::makePredictions(vec& Mean, vec& Variance, const mat& Xpred) const
{
    makePredictions(Mean, Variance, Xpred, covFunc);
}


/**
 * Simulate from PSGP
 */
vec PSGP::simulate(const mat& Xpred, bool approx) const
{
    mat cov, vCov, kxbv(Xpred.rows(), sizeActiveSet);
    vec dCov, samp;

    covFunc.covariance(kxbv, Xpred, ActiveSet);

    if(approx)
    {
        cov = Q + C;
        vCov.set_size(sizeActiveSet, sizeActiveSet);
        dCov.set_size(sizeActiveSet);
        eig_sym(cov, dCov, vCov);
        vCov = kxbv * vCov;
        samp = randn(sizeActiveSet);
    }
    else
    {
        mat kxx(Xpred.rows(), Xpred.rows());
        covFunc.covariance(kxx, Xpred);
        cov = kxx + ((kxbv * C) * kxbv.transpose());
        eig_sym(cov, dCov, vCov);
        samp = randn(Xpred.rows());
    }

    dCov = sqrt(abs(dCov));

    vec a1 = kxbv * alpha;
    mat a2 = vCov * diag(dCov);

    return(a1 + (a2 * samp));
}


/**
 * Get covariance function parameters
 */
vec PSGP::getTransformedParameters() const
{
    return covFunc.getTransformedParameters();
}


/**
 * Set covariance function parameters
 */
void PSGP::setTransformedParameters(const vec p)
{
    covFunc.setTransformedParameters(p);
}


/**
 * Recompute posterior parameters
 */
void PSGP::recomputePosterior()
{
    cout << "Update posterior for new parameters" << endl;
    mat KBold = KB;
    mat Kplus(Observations.length(), sizeActiveSet);
    covFunc.covariance(KB, ActiveSet);
    covFunc.covariance(Kplus, Locations, ActiveSet);
    
    // RB: P should be transpose(inv(KB)*Kplus), not inv(KB)*Kplus (size is wrong otherwise)   
    mat Ptrans(P.cols(), P.rows());
    backslash(KB, Kplus.transpose(), Ptrans); // output is rightmost parameter
    P = Ptrans.transpose();

    mat varEPdiag = diag(varEP);
    mat projLam = P.transpose() * varEPdiag;
    mat UU = projLam * P;
    mat CC = UU * KB + eye(sizeActiveSet);
    alpha = backslash(CC, projLam * meanEP);
    C = -backslash(CC, UU);
    Q = computeInverseFromCholesky(KB);
}

/**
 * Reset posterior representation
 */
void PSGP::resetPosterior()
{
    KB.set_size(0, 0);
    Q.set_size(0, 0);
    C.set_size(0, 0);
    alpha.set_size(0);
    ActiveSet.set_size(0, getInputDimensions());
    idxActiveSet.set_size(0);
    sizeActiveSet = 0;
    P = zeros(Observations.length(), 0);
    
    KB_aug = zeros(maxActiveSet+1, maxActiveSet+1);
    Q_aug = zeros(maxActiveSet+1, maxActiveSet+1);
    C_aug = zeros(maxActiveSet+1, maxActiveSet+1);
    alpha_aug = zeros(maxActiveSet+1);
    ActiveSet_aug = zeros(maxActiveSet+1, getInputDimensions());
    idxActiveSet_aug = to_ivec(zeros(maxActiveSet+1));
    P_aug = zeros(Observations.length(), maxActiveSet+1);

    alpha_new = 0.0;
    kb_new = 0.0;
    c_new = 0.0;
    q_new = 0.0;
    idxActiveSet_new = -1;
    ActiveSet_new = zeros(getInputDimensions());
    C_new = zeros(maxActiveSet-1);
    KB_new = zeros(maxActiveSet-1);
    Q_new = zeros(maxActiveSet-1);
    P_new = zeros(Observations.length());


    varEP = zeros(Observations.length());
    meanEP = zeros(Observations.length());
    logZ = zeros(Observations.length());
    
}

/**
 * Objective function
 * 
 * RB: Can we possibly replace the switch() with an OO version? I.e.
 * have an Evidence class which is extended by EvidenceFull, 
 * EvidenceApproximate and EvidenceUpperBound? This would also solve the issue
 * of an unknown evidence model.
 */
double PSGP::objective() const
{
    double evidence;

    switch(likelihoodType)
    {
    case FullEvid : 
        evidence = compEvidence();
        break;
    
    
    case Approximate : 
        evidence = compEvidenceApproximate();
        break;
    
    case UpperBound :	
        evidence = compEvidenceUpperBound();
        break;
        
    default:	
        // RB: This really ought to throw an exception
        cerr << "Error in PSGP::objective: Unknown likelihood type." << endl;
        return 0.0;
    }
    return evidence;
}

/**
 * Gradient of the objective function
 *
 * RB: Can we possibly replace the switch() with an OO version? I.e.
 * have an Evidence class which is extended by EvidenceFull, 
 * EvidenceApproximate and EvidenceUpperBound? Would also solve the issue
 * of an unknown evidence model.
 
 */
vec PSGP::gradient() const
{
    vec g;

    switch(likelihoodType)
    {
    case FullEvid : 
        g = gradientEvidence();
        break;
        
    case Approximate : 
        g = gradientEvidenceApproximate();
        break;
        
    case UpperBound :	
        g = gradientEvidenceUpperBound();
        break;
        
    default : 
        // RB: This really ought to throw an exception
        g.set_size(covFunc.getNumberParameters());
    }

    return g;
}

/**
 * Full evidence for current covariance function
 */
double PSGP::compEvidence() const
{

    cvec es;
    mat KB_new(sizeActiveSet, sizeActiveSet);

    covFunc.covariance(KB_new, ActiveSet);

    double evid = sum(log(varEP));

    evid -= sum(elem_mult(pow(meanEP, 2.0), varEP));
    evid += 2.0 * sum(logZ);
    evid -= varEP.length() * log(2.0 * pi);
    mat Klp = P.transpose() * diag(varEP);
    mat Ksm = (Klp * P) * KB_new + eye(sizeActiveSet);
    vec Kall = Klp * meanEP;	
    mat Kinv= backslash(Ksm.transpose(), KB_new.transpose());

    evid += dot(Kall, Kinv.transpose() * Kall);

    if(eig(Ksm, es))
    {
        evid -= sum(log(real(es)));
    }
    else
    {
        cerr << "Problem computing evidence: eig_sum()" << endl;
    }

    return -evid / 2.0;
}


/**
 * Approximate evidence for current covariance function
 */
double PSGP::compEvidenceApproximate() const
{
    mat cholSigma(sizeActiveSet, sizeActiveSet);
    mat Sigma(sizeActiveSet, sizeActiveSet);
    
    covFunc.covariance(Sigma, ActiveSet);
    mat invSigma = computeInverseFromCholesky(Sigma);
    vec obsActiveSet = Observations(idxActiveSet);
    
    vec alpha = invSigma * obsActiveSet;

    double like1 = sum(log(diag(chol(Sigma))));
    double like2 = 0.5 * dot(obsActiveSet, alpha);

    return like1 + like2 + 0.5 * sizeActiveSet * log(2 * pi);
}


/**
 * Upper bound on the evidence for current covariance function
 */
double PSGP::compEvidenceUpperBound() const
{
    mat KB_new(sizeActiveSet, sizeActiveSet);
    covFunc.covariance(KB_new, ActiveSet);

    mat U(KB_new.rows(), KB_new.cols());
    if (!chol(KB_new, U)) { 
        cout << "** Error in Cholesky decomposition of KB_new" << endl;
        cout << "Current covariance function parameters:" << endl;
        covFunc.displayCovarianceParameters();
        cout << "** Try using the Approximate evidence instead of the UpperBound" << endl;
        cout << "Press a key to abort" << endl;
        getchar();
        exit(1);
    }
    
    double like1 = 2.0 * (sum(log(diag(U))));
    // double like2 = trace((eye(sizeActiveSet) + 
    //         (KB * (C + outer_product(alpha, alpha)))) * backslash(KB_new, KB));
    double like2 = trace( ( eye(sizeActiveSet) + 
                            KB * (C + outer_product(alpha, alpha))
                          ) * backslash(U,backslash(U.transpose(), KB)) 
                        );
    
    return 0.5*(like1 + like2 + sizeActiveSet * log(2 * pi));
}


/**
 * Gradient of full evidence
 */
vec PSGP::gradientEvidence() const
{
    vec grads = zeros(covFunc.getNumberParameters());
    return grads;

}


/**
 * Gradient of approximate evidence
 */
vec PSGP::gradientEvidenceApproximate() const
{
    vec grads(covFunc.getNumberParameters());

    mat cholSigma(sizeActiveSet, sizeActiveSet);
    mat Sigma(sizeActiveSet, sizeActiveSet);

    covFunc.covariance(Sigma, ActiveSet);
    cholSigma = computeCholesky(Sigma);
    mat invSigma = computeInverseFromCholesky(Sigma);
    // chol(Sigma, cholSigma);
    // mat invSigma = backslash( cholSigma, eye(sizeActiveSet) );
    // invSigma *= invSigma.transpose();
    
    vec obsActiveSet = Observations(idxActiveSet);
    vec alpha = invSigma * obsActiveSet;

    mat W = (invSigma - outer_product(alpha, alpha, false));

    mat partialDeriv(sizeActiveSet, sizeActiveSet);

    for(int i = 0; i < covFunc.getNumberParameters(); i++)
    {
        covFunc.covarianceGradient(partialDeriv, i, ActiveSet);
        grads(i) = elem_mult_sum(W, partialDeriv) / 2.0;
    }
    return grads; 
}


/**
 * Gradient of upper bound on evidence
 */
vec PSGP::gradientEvidenceUpperBound() const
{

    vec grads(covFunc.getNumberParameters());

    mat W = eye(sizeActiveSet);
    mat KB_new(sizeActiveSet, sizeActiveSet);
    covFunc.covariance(KB_new, ActiveSet);

    // RB: This gives the correct gradient for the length scale
    mat partialDeriv(sizeActiveSet, sizeActiveSet);
    mat U = backslash(KB_new,KB);

    W +=  KB * (C + outer_product(alpha, alpha));

    for(int i = 0; i < covFunc.getNumberParameters(); i++)
    {
        covFunc.covarianceGradient(partialDeriv, i, ActiveSet);
        mat V1 = backslash(KB_new,partialDeriv);
        mat V2 = W*backslash(KB_new,partialDeriv*U);

        grads(i) = trace(V1-V2);
    }

    return 0.5*grads;
}


/**
 * Set the likelihood type
 */
void PSGP::setLikelihoodType(LikelihoodCalculation lc)
{
    likelihoodType = lc;
}


/**
 * Display current parameters
 */
void PSGP::displayModelParameters() const
{
    cout << "Summary Sequential Gaussian Process" << endl;
    cout << "  Kernel Matrix size         : " << KB.rows() << " x " << KB.cols() << endl;
    cout << "  Inverse Kernel Matrix size : " << Q.rows() << " x " << Q.cols() << endl;
    cout << "  alpha size                 : " << alpha.size() << endl;
    cout << "  C size                     : " << C.rows() << " x " << C.cols() << endl;
    cout << "  Projection matrix size     : " << P.rows() << " x " << P.cols() << endl;
    cout << "  Lambda                     : " << varEP.size() << endl;
    cout << "  projection alpha           : " << meanEP.size() << endl;
    cout << "  log evidence vector        : " << logZ.size() << endl;
    cout << "  ----------------------------" << endl;
    cout << "  Predicion locations        : " << Locations.rows() << " x " << Locations.cols() << endl;
    cout << "  Observations               : " << Observations.size() << endl;
    cout << "  Active set size            : " << ActiveSet.rows() << " (max = " << maxActiveSet << ")" << endl;
    cout << "  Epsilon tolerance          : " << epsilonTolerance << endl;
    cout << "  Iterations Changing/Fixed  : " << iterChanging << "/" << iterFixed << endl;
}


/**
 * Compute Cholesky decomposition of matrix
 * 
 * RB: This belongs in a different class/library, really...
 * TODO: Move to ITPPExt
 */
mat PSGP::computeCholesky(const mat& iM) const 
{
    mat M = iM;
    assert(M.rows() == M.cols());

    const double ampl = 1.0e-10;
    const int maxAttempts = 10;

    mat cholFactor(M.rows(), M.cols());

    int l = 0;
    bool success = chol(M, cholFactor);
    if(success)
    {
        return cholFactor;
    }
    else
    {
        double noiseFactor = abs(ampl * (trace(M) / double(M.rows())));
        while(!success)
        {
            M = M + (noiseFactor * eye(M.rows()));

            if(l > maxAttempts)
            {
                cerr << "Unable to compute cholesky decomposition" << endl;
                break;
            }
            l++;
            noiseFactor = noiseFactor * 10;
            success = chol(M, cholFactor);
        }
        cout << "Matrix not positive definite.  After " << l << " attempts, " << noiseFactor << " added to the diagonal" << endl;
    }
    return cholFactor;

}


/**
 * Compute inverse of a square matrix using Cholesky decomposition
 * 
 * TODO: Move to ITPPExt
 */
mat PSGP::computeInverseFromCholesky(const mat& C) const
{
    mat cholFactor = computeCholesky(C);
    mat invChol = backslash(cholFactor, eye(cholFactor.rows()));
    return invChol * invChol.transpose();
}

