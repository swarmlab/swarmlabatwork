#ifndef ITPPEXT_H_
#define ITPPEXT_H_

/**
* Extension to IT++ library
*/
#include <vector>
#include <string>
#include <cassert>
#include <math.h>
#include <itpp/itbase.h>
#include <itpp/itstat.h>


using namespace std;
using namespace itpp;

namespace itppext 
{

vec ltr_vec(mat M);     // Vector of lower triangular elements
vec utr_vec(mat M);     // Vector of upper triangular elements
mat ltr_mat(vec v);     // Lower triangular matrix
mat utr_mat(vec v);     // Upper triangular matrix

double cond(mat M, int p=2); // Condition number for matrix p-norm (1 or 2)

ivec randperm(int n);  // Random permutation of numbers between 0 and N-1

vec min(vec u, vec v); // Minimum elements from 2 vectors of equal length

mat concat_cols(mat X, vec y); // Concatenate matrix and vector
mat concat_cols(mat X, mat Y); // Concatenate matrix and matrix

vec mean_rows(mat X);  // vector of column means
vec mean_cols(mat X);  // vector of row means
mat cov(mat X, vec &xmean); // covariance of rows, also returns the mean
mat cov(mat X); // covariance of rows

void normalise(mat &X);
void normalise(mat &X, vec &mean, vec &covdiag);
void denormalise(mat &X, vec mean, vec covdiag);

void tokenise(const string& str, vector<string>& tokens, const string& delimiters = " ", int stopAfter=0);

} // END OF NAMESPACE ITPPEXT


#endif /*ITPPEXT_H_*/
