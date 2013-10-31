#include "itppext.h"

namespace itppext {

/**
 * Returns the lower triangular elements (including diagonal) 
 * of a matrix, in row-major order.
 * 
 * @param M An NxN square matrix
 * @return The vector of N(N+1)/2 lower triangular elements of M
 * @see ltr_mat, utr_vec, utr_mat
 */   
vec ltr_vec(mat M) 
{
    int N = M.cols();
    assert(N==M.rows());
        
    int k = 0;
    vec v(N*(N+1)/2); 
    
    for (int i=0; i<N; i++) 
        for (int j=0; j<=i; j++) 
            v(k++) = M(i,j); 
    
    return v;
}

/**
 * Returns the upper triangular elements (including diagonal) 
 * of a matrix, in row-major order.
 * 
 * @param M An NxN square matrix
 * @return The vector of N(N+1)/2 upper triangular elements of M
 * @see utr_mat, ltr_vec, ltr_mat
 */   
vec utr_vec(mat M) 
{
    int N = M.cols();
    assert(N==M.rows());
    
    int k = 0;
    vec v(N*(N+1)/2); 
    
    for (int i=0; i<N; i++) 
        for (int j=i; j<N; j++) 
            v(k++) = M(i,j); 
    
    return v;
}

/**
 * Returns a lower triangular matrix where the elements are taken
 * from the argument vector in row-major order.
 * 
 * @param v A vector of N(N+1)/2 lower triangular elements
 * @return A NxN lower triangular matrix
 * @see ltr_vec, utr_vec, utr_mat
 */
mat ltr_mat(vec v)
{
    // Retrieve dimension of matrix
    int N =  (int) floor(sqrt(2*v.length()));
    
    assert(N*(N+1)/2 == v.length());
    
    mat M = zeros(N,N);
    int k = 0;
    
    for (int i=0; i<N; i++) 
            for (int j=0; j<=i; j++) 
                M(i,j) = v(k++);  
    
    return M;
}

/**
 * Returns an upper triangular matrix where the elements are taken
 * from the argument vector in row-major order.
 * 
 * @param v A vector of N(N+1)/2 upper triangular elements
 * @return A NxN upper triangular matrix
 * @see ltr_vec, utr_vec, utr_mat
 */
mat utr_mat(vec v)
{
    // Retrieve dimension of matrix
    int N =  (int) floor(sqrt(2*v.length()));
    
    assert(N*(N+1)/2 == v.length());
    
    mat M = zeros(N,N);
    int k = 0;
    
    for (int i=0; i<N; i++) 
            for (int j=i; j<N; j++) 
                M(i,j) = v(k++);  
    
    return M;
}

double cond(mat M, int p) 
{
    assert(M.rows() == M.cols());
    return itpp::norm(M,p)*itpp::norm(inv(M),p);
}

/**
 * Returns a random permutation of numbers between 0 and N-1
 */
ivec randperm(int n) 
{
	vec rndNums = randu(n);
	return sort_index(rndNums);
}

/**
 * Returns the vector of minimum elements from 2 vectors, i.e.
 * z(i) = min(u(i), v(i)).
 */
vec min(vec u, vec v) 
{
    assert(u.length() == v.length());
    
    vec z(u.length());
    
    for (int i=0; i<u.length(); i++) 
        z(i) = std::min(u(i), v(i));
    
    return z;
}

/**
 * Concatenation Z = [X y]
 */
mat concat_cols(mat X, vec y) {
    assert(X.rows()==y.length());
    
    mat Z(X.rows(), X.cols()+1);
    
    for (int i=0; i<X.cols(); i++) Z.set_col(i, X.get_col(i));
    Z.set_col(X.cols(), y);
    
    return Z;
}

/**
 * Concatenation Z = [X Y]
 */
mat concat_cols(mat X, mat Y) {
    assert(X.rows()==Y.rows());
    
    mat Z(X.rows(), X.cols()+Y.cols());
    
    for (int i=0; i<X.cols(); i++) Z.set_col(i, X.get_col(i));
    for (int i=X.cols(); i<X.cols()+Y.cols(); i++) Z.set_col(i, Y.get_col(i));
    
    return Z;
}

/**
 * Empirical arithmetic mean along rows
 */
vec mean_rows(mat X)
{
    int D = X.cols();
    
    vec m(D);
    for (int i=0; i<D; i++) m(i) = mean(X.get_col(i));
    return m;
}

/**
 * Empirical arithmetic mean along columns
 */
vec mean_cols(mat X)
{
    int N = X.rows();
    
    vec m(N);
    for (int i=0; i<N; i++) m(i) = mean(X.get_row(i));
    return m;
}


/**
 * Unbiased empirical covariance
 */
mat cov(mat X, vec &xmean)
{
    int N = X.rows();

    mat matxmean = mat(1,X.cols());
    matxmean.set_row(0, mean_rows(X));

    mat Xcentred = X-repmat(matxmean, N, 1);
    
    mat C = 1.0/(N-1) * Xcentred.transpose() * Xcentred;
    
    xmean = matxmean.get_row(0);
    return C;
}

mat cov(mat X)
{
    mat C;
    vec xmean;
    C = cov(X, xmean);
    return C;
}

/**
 * Normalises a data set comprising a set of inputs X and a set of outputs y.
 * The X and y arguments are overridden by their normalised versions. 
 * The mean and covariance of the original dataset are also returned.
 */
void normalise(mat &X, vec &Xmean, vec &Xcovdiag) 
{
    int N = X.rows();
    int D = X.cols(); 
    
    Xcovdiag  = diag(cov(X,Xmean));
    
    mat matXmean(1,D);
    
    matXmean.set_row(0, Xmean);
    
    mat Xcentred = X - repmat(matXmean, N, 1);
    mat Xsphered(N,D);
    
    for (int i=0; i<D; i++)
        Xsphered.set_col( i, 1.0/sqrt(Xcovdiag(i)) * Xcentred.get_col(i) );
    
    X = Xsphered;
}

void normalise(mat &X) 
{ 
    vec Xmean;
    vec Xcovdiag;
    normalise(X, Xmean, Xcovdiag);
}

void denormalise(mat &X, vec Xmean, vec Xcovdiag)
{
    int N = X.rows();
    int D = X.cols(); 

    mat matXmean(1,D);
    matXmean.set_row(0, Xmean);

    // Rescale
    mat Xdesphered(N,D);
    for (int i=0; i<D; i++)
        Xdesphered.set_col(i,sqrt(Xcovdiag(i))*X.get_col(i));
    
    // Add bias
    mat Xdecentred = Xdesphered + repmat(matXmean, N, 1);
    
    X = Xdecentred;    
}


/**
 * String tokeniser - split a string into a vector of substring identified
 * by a delimiter (space by default). Optionally, stop after a certain number
 * of tokens has been found and return the rest of the string as the last token
 * 
 * Adapted from: http://oopweb.com/CPP/Documents/CPPHOWTO/Volume/C++Programming-HOWTO-7.html
 */
void tokenise(const string& str, vector<string>& tokens, const string& delimiters, int stopAfter)
{
    // by default, do not stop after a given number of tokens (i.e. process 
    // the whole string)
    if (stopAfter == 0) stopAfter = str.length()+1; 
    
    // Find start and end of first token
    string::size_type tokenStart = str.find_first_not_of(delimiters, 0);       // First char
    string::size_type tokenEnd = str.find_first_of(delimiters, tokenStart);    // Following delimiter

    while ((string::npos != tokenEnd || string::npos != tokenStart) && tokens.size() < stopAfter)
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(tokenStart, tokenEnd - tokenStart));
        
        // Move to next token 
        tokenStart = str.find_first_not_of(delimiters, tokenEnd);
        tokenEnd   = str.find_first_of(delimiters, tokenStart);
    }
    
    // If we have stopped after a given number of tokens, add the rest of the
    // string as a final token 
    if (tokens.size() == stopAfter) {
        tokens.push_back(str.substr(tokenStart));
    }
}


} // END OF NAMESPACE ITPPEXT




