#include "header/polyfit.h"

polyFit::polyFit(QObject *parent) : QObject(parent)
{

}

// Adapted from https://github.com/codeplea/incbeta
double incbeta(double a, double b, double x) {
    if (x < 0.0 || x > 1.0) return 1.0/0.00000001;

    if (a<=0.) {
        std::cout << "Warning: a should be >0";
        return 0.;
    }

    if (b<=0.) {
        std::cout << "Warning: b should be >0";
        return 0.;
    }


    /*The continued fraction converges nicely for x < (a+1)/(a+b+2)*/
    if (x > (a+1.0)/(a+b+2.0)) {
        return (1.0-incbeta(b,a,1.0-x)); /*Use the fact that beta is symmetrical.*/
    }

    /*Find the first part before the continued fraction.*/
    const double lbeta_ab = lgamma(a)+lgamma(b)-lgamma(a+b);
    const double front = exp(log(x)*a+log(1.0-x)*b-lbeta_ab) / a;

    /*Use Lentz's algorithm to evaluate the continued fraction.*/
    double f = 1.0, c = 1.0, d = 0.0;

    int i, m;
    for (i = 0; i <= 200; ++i) {
        m = i/2;

        double numerator;
        if (i == 0) {
            numerator = 1.0; /*First numerator is 1.0.*/
        } else if (i % 2 == 0) {
            numerator = (m*(b-m)*x)/((a+2.0*m-1.0)*(a+2.0*m)); /*Even term.*/
        } else {
            numerator = -((a+m)*(a+b+m)*x)/((a+2.0*m)*(a+2.0*m+1)); /*Odd term.*/
        }

        /*Do an iteration of Lentz's algorithm.*/
        d = 1.0 + numerator * d;
        if (fabs(d) < TINY) d = TINY;
        d = 1.0 / d;

        c = 1.0 + numerator / c;
        if (fabs(c) < TINY) c = TINY;

        const double cd = c*d;
        f *= cd;

        /*Check for stop.*/
        if (fabs(1.0-cd) < STOP) {
            return front * (f-1.0);
        }
    }

    return 1.0/0.0000001; /*Needed more loops, did not converge.*/
}

double invincbeta(double y,double alpha, double beta) {

    if (y <= 0.) return 0.;
    else if (y >= 1.) return 1.;
    if (alpha<=0.) {
        std::cout << "Warning: alpha should be >0";
        return 0.;
    }

    if (beta<=0.) {
        std::cout << "Warning: beta should be >0";
        return 0.;
    }


    double x = 0.5;
    double a = 0;
    double b = 1;
    double precision = 1.e-8;
    double binit = y;
    double bcur = incbeta(alpha,beta,x);

    while (fabs(bcur-binit)>precision) {

        if ((bcur-binit)<0) {
            a = x;
        }
        else {
            b = x;
        }
        x = (a+b)*0.5;
        bcur = incbeta(alpha,beta,x);

        //std::cout << x << "\t" << bcur << "\n";


    }

    return x;


}



// Calculate the t value for a Student distribution
// Adapted from http://www.cplusplus.com/forum/beginner/216098/
// **************************************************************
double CalculateTValueStudent(const double nu, const double alpha) {

    double precision = 1.e-5;

    if (alpha<=0. || alpha >= 1.) return 0.;

    double x = invincbeta(2.*min(alpha,1.-alpha), 0.5*nu, 0.5);
    x = sqrt(nu*(1.-x)/x);
    return (alpha >= 0.5? x : -x);


}

// Cumulative distribution for Student-t
// **************************************************************
double cdfStudent(const double nu, const double t)
{
    double x = nu/(t*t+nu);

    return 1.-incbeta(0.5*nu,0.5,x);
}

// Cumulative distribution for Fisher F
// **************************************************************
double cdfFisher(const double df1, const double df2, const double x) {
    double y = df1*x/(df1*x+df2);
    return incbeta(0.5*df1,0.5*df2,y);
}

// Initialize a 2D array
// **************************************************************
double **Make2DArray(const size_t rows, const size_t cols) {

    double **array;

    array = new double*[rows];
    for(size_t i = 0; i < rows; i++) {
        array[i] = new double[cols];
    }

    for(size_t i = 0; i < rows; i++) {
        for(size_t j = 0; j < cols; j++) {
            array[i][j] = 0.;
        }
    }

    return array;

}

// Transpose a 2D array
// **************************************************************
double **MatTrans(double **array, const size_t rows, const size_t cols) {

    double **arrayT = Make2DArray(cols,rows);

    for(size_t i = 0; i < rows; i++) {
        for(size_t j = 0; j < cols; j++) {
            arrayT[j][i] = array[i][j];
        }
    }

    return arrayT;

}

// Perform the multiplication of matrix A[m1,m2] by B[m2,m3]
// **************************************************************
double **MatMul(const size_t m1, const size_t m2, const size_t m3, double **A, double **B) {

    double **array = Make2DArray(m1,m3);

    for (size_t i=0; i<m1; i++) {
        for (size_t j=0; j<m3; j++) {
            array[i][j]=0.;
            for (size_t m=0; m<m2; m++) {
                array[i][j]+=A[i][m]*B[m][j];
            }
        }
    }
    return array;

}

// Perform the multiplication of matrix A[m1,m2] by vector v[m2,1]
// **************************************************************
void MatVectMul(const size_t m1, const size_t m2, double **A, double *v, double *Av) {


    for (size_t i=0; i<m1; i++) {
        Av[i]=0.;
        for (size_t j=0; j<m2; j++) {
            Av[i]+=A[i][j]*v[j];
        }
    }


}


// Calculates the determinant of a matrix
// **************************************************************
double determinant(double **a, const size_t k) {

    double s = 1;
    double det = 0.;
    double **b = Make2DArray(k,k);
    size_t m;
    size_t n;

    if (k == 1) return (a[0][0]);

    for (size_t c=0; c<k; c++) {

        m = 0;
        n = 0;

        for (size_t i = 0; i < k; i++) {

            for (size_t j = 0; j < k; j++) {

                b[i][j] = 0;

                if (i != 0 && j != c) {

                    b[m][n] = a[i][j];
                    if (n < (k - 2)) {
                        n++;
                    }
                    else
                    {
                        n = 0;
                        m++;
                    }
                }
            }
        }

        det = det + s * (a[0][c] * determinant(b, k - 1));
        s = -1 * s;

    }

    return (det);

}


// Perform the
// **************************************************************
void transpose(double **num, double **fac, double **inverse, const size_t r) {

    double **b = Make2DArray(r,r);
    double deter;

    for (size_t i=0; i<r; i++) {
        for (size_t j=0; j<r; j++) {
            b[i][j] = fac[j][i];
        }
    }

    deter = determinant(num, r);

    for (size_t i=0; i<r; i++) {
        for (size_t j=0; j<r; j++) {
            inverse[i][j] = b[i][j] / deter;
        }
    }

}

// Calculates the cofactors
// **************************************************************
void cofactor(double **num, double **inverse, const size_t f)
{

    double **b = Make2DArray(f,f);
    double **fac = Make2DArray(f,f);

    size_t m;
    size_t n;

    for (size_t q=0; q<f; q++) {

        for (size_t p=0; p<f; p++) {

            m = 0;
            n = 0;

            for (size_t i=0; i<f; i++) {

                for (size_t j=0; j<f; j++) {

                    if (i != q && j != p) {

                        b[m][n] = num[i][j];

                        if (n < (f - 2)) {
                            n++;
                        }
                        else
                        {
                            n = 0;
                            m++;
                        }
                    }
                }
            }
            fac[q][p] = pow(-1, q + p) * determinant(b, f - 1);
        }
    }

    transpose(num, fac, inverse, f);

}


// Display a matrix
// **************************************************************
void displayMat(double **A, const size_t n, const size_t m)  {

    cout << "Matrix " << n << " x " << m << endl;
    for (size_t i=0; i<n; i++) {
        for (size_t j=0; j<m; j++)
            cout << A[i][j] << "\t";
        cout << endl;
    }
    cout << endl;

}

// Calculate the residual sum of squares (RSS)
// **************************************************************
double CalculateRSS(const double *x, const double *y, const double *a, double **Weights,
const bool fixed, const size_t N, const size_t n) {

    double r2 = 0.;
    double ri = 0.;
    for (size_t i=0; i<N; i++) {
        ri = y[i];
        for (size_t j=0; j<n; j++) {
            ri -= a[j]*pow(x[i],j);
        }
        r2 += ri*ri*Weights[i][i];
    }

    return r2;

}

// Calculate the total sum of squares (TSS)
// **************************************************************
double CalculateTSS(const double *x, const double *y, const double *a, double **Weights,
const bool fixed, const size_t N, const size_t n) {

    double r2 = 0.;
    double ri = 0.;
    double sumwy = 0.;
    double sumweights = 0.;
    size_t begin = 0;
    if (fixed) {
        for (size_t i=begin; i<N; i++) {
            r2+= y[i]*y[i]*Weights[i][i];
        }
    } else {


        for (size_t i=begin; i<N; i++) {
            sumwy += y[i]*Weights[i][i];
            sumweights += Weights[i][i];
        }

        for (size_t i=begin; i<N; i++) {
            ri = y[i]-sumwy/sumweights;
            r2 += ri*ri*Weights[i][i];
        }
    }

    return r2;

}

// Calculate coefficient R2 - COD
// **************************************************************
double CalculateR2COD(const double *x, const double *y, const double *a, double **Weights,
const bool fixed, const size_t N, const size_t n) {

    double RSS = CalculateRSS(x,y,a,Weights,fixed,N,n);
    double TSS = CalculateTSS(x,y,a,Weights,fixed,N,n);
    double R2 = 1.-RSS/TSS;

    return R2;

}

// Calculate the coefficient R2 - adjusted
// **************************************************************
double CalculateR2Adj(const double *x, const double *y, const double *a, double **Weights,
const bool fixed,const size_t N, const size_t n) {

    double RSS = CalculateRSS(x,y,a,Weights,fixed,N,n);
    double TSS = CalculateTSS(x,y,a,Weights,fixed,N,n);

    double dferr = N-n;
    double dftot = N-1;

    if (fixed) {
        dferr += 1.;
        dftot += 1.;
    }

    double R2Adj = 1.-(dftot)/(dferr)*RSS/TSS;

    return R2Adj;

}

// Perform the fit of data n data points (x,y) with a polynomial of order k
// **************************************************************
void PolyFit(const double *x, double *y, const size_t n, const size_t k, const bool fixedinter,
const double fixedinterval, double *beta, double **Weights, double **XTWXInv) {

    // Definition of variables
    // **************************************************************
    double **X = Make2DArray(n,k+1);           // [n,k+1]
    double **XT;                               // [k+1,n]
    double **XTW;                              // [k+1,n]
    double **XTWX;                             // [k+1,k+1]

    double *XTWY = new double[k+1];
    double *Y = new double[n];

    size_t begin = 0;
    if (fixedinter) begin = 1;

    // Initialize X
    // **************************************************************
    for (size_t i=0; i<n; i++) {
        for (size_t j=begin; j<(k+1); j++) {  // begin
          X[i][j]=pow(x[i],j);
        }
    }

    // Matrix calculations
    // **************************************************************
    XT = MatTrans(X, n, k+1);                 // Calculate XT
    XTW = MatMul(k+1,n,n,XT,Weights);         // Calculate XT*W
    XTWX = MatMul(k+1,n,k+1,XTW,X);           // Calculate (XTW)*X

    if (fixedinter) XTWX[0][0] = 1.;

    cofactor(XTWX, XTWXInv, k+1);             // Calculate (XTWX)^-1

    for (size_t m=0; m<n; m++) {
        if (fixedinter) {
            Y[m]= y[m]-fixedinterval;
        }
        else {
            Y[m] = y[m];
        }
    }
    MatVectMul(k+1,n,XTW,Y,XTWY);             // Calculate (XTW)*Y
    MatVectMul(k+1,k+1,XTWXInv,XTWY,beta);    // Calculate beta = (XTWXInv)*XTWY

    if (fixedinter) beta[0] = fixedinterval;

//    cout << "Matrix X" << endl;
//    displayMat(X,n,k+1);

//    cout << "Matrix XT" << endl;
//    displayMat(XT,k+1,n);

//    cout << "Matrix XTW" << endl;
//    displayMat(XTW,k+1,n);

//    cout << "Matrix XTWXInv" << endl;
//    displayMat(XTWXInv,k+1,k+1);


}



// Calculate the polynomial at a given x value
// **************************************************************
double calculatePoly(const double x, const double *a, const size_t n) {

    double poly = 0.;

    for (size_t i=0; i<n; i++) {
        poly += a[i]*pow(x,i);
    }

    return poly;

}

// Calculate and write the confidence bands in a file
// **************************************************************
void WriteCIBands(std::string filename, const double *x, const double *coefbeta, double **XTXInv,
const double tstudentval, const double SE, const size_t n, const size_t k) {


    double interval = (x[n-1]-x[0]);
    double x1,y0,y1,y2,y3,y4;
    double xstar[3];
    double xprod = 0.;

    ofstream output;
    output.open(filename.c_str());

    for (int i=0; i<101; i++) {
        x1 = x[0]+interval/100.*i;
        for (size_t j=0; j<k+1; j++) {
            xstar[j] = pow(x1,j);
        }

        xprod = 0.;
        for (size_t j=0; j<(k+1); j++) {
            for (size_t m=0; m<(k+1); m++) {
                xprod += xstar[m]*xstar[j]*XTXInv[j][m];
            }
        }

        y0 = calculatePoly(x1, coefbeta,k+1);
        y1 = y0 - tstudentval*SE*sqrt(xprod);
        y2 = y0 + tstudentval*SE*sqrt(xprod);
        y3 = y0 - tstudentval*SE*sqrt(1+xprod);
        y4 = y0 + tstudentval*SE*sqrt(1+xprod);

        output << x1 << "\t" << y0 << "\t" << y1 << "\t" << y2 << "\t";
        output << y3 << "\t" << y4 << endl;

    }


    output.close();


}

// Calculate the weights matrix
// **************************************************************
void CalculateWeights(const double *erry, double **Weights, const size_t n,
const int type) {


    for(size_t i = 0; i < n; i++) {

        switch (type) {
            case 0:
                Weights[i][i] = 1.;
            break;
            case 1:
                Weights[i][i] = erry[i];
            break;
            case 2:
                if (erry[i]>0.) {
                    Weights[i][i] = 1./(erry[i]*erry[i]);
                }
                else {
                    Weights[i][i] = 0.;
                }
            break;
        }

    }

}

// Calculate the standard error on the beta coefficients
// **************************************************************
void CalculateSERRBeta(const bool fixedinter, const double SE, size_t k, double *serbeta, double **XTWXInv) {

    size_t begin = 0;
    if (fixedinter) begin = 1;

    serbeta[0] = 0.;
    for (size_t i=begin; i<(k+1); i++) {
        serbeta[i] = SE*sqrt(XTWXInv[i][i]);
    }

}

// Display the polynomial
// **************************************************************
void DisplayPolynomial(const size_t k) {

    cout << "y = ";
    for (size_t i=0; i<(k+1); i++) {
        cout << "A" << i;
        if (i>0) cout << "X";
        if (i>1) cout << "^" << i;
        if (i<k) cout << " + ";
    }
    cout << endl << endl;

}

// Display the ANOVA test result
// **************************************************************
void DisplayANOVA(const size_t nstar, const size_t k, const double TSS, const double RSS) {

    double MSReg = (TSS-RSS)/(k);
    double MSE = RSS/(nstar-k);
    double FVal = MSReg/MSE;
    double pFVal = 1.-cdfFisher(k,nstar-k, FVal);

    cout << "ANOVA" << endl;
    cout << "\tDF\tSum squares\tMean square\tF value\tProb>F" << endl;
    cout << "Model\t" << k << "\t" << (TSS-RSS) << "\t" << (TSS-RSS)/k << "\t" << FVal << "\t" << pFVal << endl;
    cout << "Error\t" << nstar-k << "\t" << RSS << "\t" << RSS/(nstar-k) << endl;
    cout << "Total\t" << nstar << "\t" << TSS << endl << endl;

}


// Display the coefficients of the polynomial
// **************************************************************
void DisplayCoefs(const size_t k, const size_t nstar, const double tstudentval, const double *coefbeta, const double *serbeta) {

    double lcibeta;    // Low confidence interval coefficients
    double hcibeta;    // High confidence interval coefficients

    cout << "Polynomial coefficients" << endl;
    cout << "Coeff\tValue\tStdErr\tLowCI\tHighCI\tStudent-t\tProb>|t|" << endl;

    for (size_t i=0; i<(k+1); i++) {
        lcibeta = coefbeta[i]-tstudentval*serbeta[i];
        hcibeta = coefbeta[i]+tstudentval*serbeta[i];
        cout << "A" << i << "\t";
        cout << coefbeta[i] << "\t";
        cout << serbeta[i] << "\t";
        cout << lcibeta << "\t";
        cout << hcibeta << "\t";

        if (serbeta[i]>0) {
            cout << coefbeta[i]/serbeta[i] << "\t";
            cout << 1.-cdfStudent(nstar-k, coefbeta[i]/serbeta[i]);
        } else {
            cout << "-\t-";
        }

        cout << endl;
    }

}

// Display some statistics values
// **************************************************************
void DisplayStatistics(const size_t n, const size_t nstar, const size_t k, const double RSS, const double R2,
const double R2Adj, const double SE) {


    cout << endl;
    cout << "Statistics" << endl;
    cout << "Number of points: " << n << endl;
    cout << "Degrees of freedom: " << nstar-k << endl;
    cout << "Residual sum of squares: " << RSS << endl;
    cout << "R-square (COD): " << R2 << endl;
    cout << "Adj R-square: " << R2Adj << endl;
    cout << "RMSE: " << SE << endl << endl;


}


// Display the covariance and correlation matrix
// **************************************************************
void DisplayCovCorrMatrix(const size_t k, const double sigma, const bool fixed, double **XTWXInv) {


    double **CovMatrix = Make2DArray(k+1,k+1);
    double **CorrMatrix = Make2DArray(k+1,k+1);

    for (size_t i=0; i<k+1; i++) {
        for (size_t j=0; j<k+1; j++) {
            CovMatrix[i][j] = sigma*sigma*XTWXInv[i][j];
        }
    }

    if (fixed) CovMatrix[0][0] = 1.;

    for (size_t i=0; i<k+1; i++) {
        for (size_t j=0; j<k+1; j++) {
            CorrMatrix[i][j] = CovMatrix[i][j]/(sqrt(CovMatrix[i][i])*sqrt(CovMatrix[j][j]));
        }
    }


    cout << "Covariance matrix" << endl;
    displayMat(CovMatrix,k+1,k+1);

    cout << "Correlation matrix" << endl;
    displayMat(CorrMatrix,k+1,k+1);


}


// The main program
// **************************************************************
int polyFit::maini() {

    cout << "Polynomial fit!" << endl;

    // Input values
    // **************************************************************
    size_t k = 2;                                    // Polynomial order
    bool fixedinter = false;                         // Fixed the intercept (coefficient A0)
    int wtype = 0;                                   // Weight: 0 = none (default), 1 = sigma, 2 = 1/sigma^2
    double fixedinterval = 0.;                       // The fixed intercept value (if applicable)
    double alphaval = 0.05;                          // Critical apha value

    double x[] = {-1, 0, 0, 1, 1, 2, 2, 2, 3, 3};
    double y[] = {39, 40, 40, 41, 41, 42, 42, 43, 44, 45};
    double erry[] = {0,0,0,0,0,0,0,0,0,0};       // Data points (err on y) (if applicable)

    // Definition of other variables
    // **************************************************************
    size_t n = 0;                                    // Number of data points (adjusted later)
    size_t nstar = 0;                                // equal to n (fixed intercept) or (n-1) not fixed
    double coefbeta[3];                            // Coefficients of the polynomial
    double serbeta[3];                             // Standard error on coefficients
    double tstudentval = 0.;                         // Student t value
    double SE = 0.;                                  // Standard error

    double **XTWXInv;                                // Matrix XTWX Inverse [k+1,k+1]
    double **Weights;                                // Matrix Weights [n,n]


    // Initialize values
    // **************************************************************
    n = sizeof(x)/sizeof(double);
    nstar = n-1;
    if (fixedinter) nstar = n;

    cout << "Number of points: " << n << endl;
    cout << "Polynomial order: " << k << endl;
    if (fixedinter) {
        cout << "A0 is fixed!" << endl;
    } else {
        cout << "A0 is adjustable!" << endl;
    }

    if (k>nstar) {
        cout << "The polynomial order is too high. Max should be " << n << " for adjustable A0 ";
        cout << "and " << n-1 << " for fixed A0. ";
        cout << "Program stopped" << endl;
        return -1;
    }

    if (k==nstar) {
        cout << "The degree of freedom is equal to the number of points. ";
        cout << "The fit will be exact." << endl;
    }

    XTWXInv = Make2DArray(k+1,k+1);
    Weights = Make2DArray(n,n);

    // Build the weight matrix
    // **************************************************************
    CalculateWeights(erry, Weights, n, wtype);

    cout << "Weights" << endl;
    displayMat(Weights,n,n);

    if (determinant(Weights,n)==0.) {
        cout << "One or more points have 0 error. Review the errors on points or use no weighting. ";
        cout << "Program stopped" << endl;
        return -1;
    }

    // Calculate the coefficients of the fit
    // **************************************************************
    PolyFit(x,y,n,k,fixedinter,fixedinterval,coefbeta,Weights,XTWXInv);


    // Calculate related values
    // **************************************************************
    double RSS = CalculateRSS(x,y,coefbeta,Weights,std::fixed,n,k+1);
    double TSS = CalculateTSS(x,y,coefbeta,Weights,fixedinter,n,k+1);
    double R2 = CalculateR2COD(x,y,coefbeta,Weights,fixedinter,n,k+1);
    double R2Adj = CalculateR2Adj(x,y,coefbeta,Weights,fixedinter,n,k+1);

    if ((nstar-k)>0) {
        SE = sqrt(RSS/(nstar-k));
        tstudentval = fabs(CalculateTValueStudent(nstar-k, 1.-0.5*alphaval));
    }
    cout << "t-student value: " << tstudentval << endl << endl;

    // Calculate the standard errors on the coefficients
    // **************************************************************
    CalculateSERRBeta(fixedinter,SE,k,serbeta,XTWXInv);

    // Display polynomial
    // **************************************************************
    DisplayPolynomial(k);

    // Display polynomial coefficients
    // **************************************************************
    DisplayCoefs(k, nstar, tstudentval, coefbeta, serbeta);

    // Display statistics
    // **************************************************************
    DisplayStatistics(n,nstar,k,RSS,R2,R2Adj,SE);

    // Display ANOVA table
    // **************************************************************
    DisplayANOVA(nstar, k, TSS, RSS);

    // Write the prediction and confidence intervals
    // **************************************************************
    WriteCIBands("CIBands2.dat",x,coefbeta,XTWXInv,tstudentval,SE,n,k);

    // Display the covariance and correlation matrix
    // **************************************************************
    DisplayCovCorrMatrix(k, SE, fixedinter, XTWXInv);

    qDebug()<<"A1 ist:" <<coefbeta[1];
    qDebug()<<"A0 ist:" <<coefbeta[0];


}

QVector<double> polyFit::getSlope(QVector<double> xVec, QVector<double> yVec, double **Weights)
{
    // **************************************************************
    size_t k = 2;                                    // Polynomial order
    bool fixedinter = false;                         // Fixed the intercept (coefficient A0)
    double fixedinterval = 0.;                       // The fixed intercept value (if applicable)

    double* x = new double[xVec.size()]{0};
    double* y = new double[yVec.size()]{0};
    for(int i = 0; i<xVec.size(); i++){
        x[i] = xVec[i];
        y[i] = yVec[i];
    }

    // Definition of other variables
    // **************************************************************
    size_t n = 0;                                    // Number of data points (adjusted later)
    size_t nstar = 0;                                // equal to n (fixed intercept) or (n-1) not fixed
    double coefbeta[3]={0,0,0};                            // Coefficients of the polynomial


    double **XTWXInv;                                // Matrix XTWX Inverse [k+1,k+1]

    // Initialize values
    // **************************************************************
    n = xVec.size();
    nstar = n-1;
    if (fixedinter) nstar = n;

    if (k>nstar) {
        cout << "The polynomial order is too high. Max should be " << n << " for adjustable A0 ";
        cout << "and " << n-1 << " for fixed A0. ";
        cout << "Program stopped" << endl;
        return {-100,-100};
    }

    if (k==nstar) {
        cout << "The degree of freedom is equal to the number of points. ";
        cout << "The fit will be exact." << endl;
    }

    XTWXInv = Make2DArray(k+1,k+1);

    // Calculate the coefficients of the fit
    // **************************************************************
    PolyFit(x,y,n,k,fixedinter,fixedinterval,coefbeta,Weights,XTWXInv);

    return {coefbeta[1],coefbeta[0]};
}

void polyFit::cofactorQt(QVector<QVector<double> > &num, QVector<QVector<double> > &inverse, const size_t f)
{
    QVector<QVector<double> > b = makeQtMat(f,f);
    QVector<QVector<double> > fac = makeQtMat(f,f);

    size_t m;
    size_t n;

    for (size_t q=0; q<f; q++) {

        for (size_t p=0; p<f; p++) {

            m = 0;
            n = 0;

            for (size_t i=0; i<f; i++) {

                for (size_t j=0; j<f; j++) {

                    if (i != q && j != p) {

                        b[m][n] = num[i][j];

                        if (n < (f - 2)) {
                            n++;
                        }
                        else
                        {
                            n = 0;
                            m++;
                        }
                    }
                }
            }
            fac[q][p] = pow(-1, q + p) * determinantQt(b, f - 1);
        }
    }

    transposeQt(num, fac, inverse, f);

}

void polyFit::MatVectMulQt(const size_t m1, const size_t m2, QVector<QVector<double> > &A, QVector<double> &v, QVector<double> &Av)
{

    for (size_t i=0; i<m1; i++) {
        Av[i]=0.;
        for (size_t j=0; j<m2; j++) {
            Av[i]+=A[i][j]*v[j];
        }
    }
}

void polyFit::transposeQt(QVector<QVector<double> > &num, QVector<QVector<double> > &fac, QVector<QVector<double> > &inverse, const size_t r)
{
    QVector<QVector<double> > b = makeQtMat(r,r);
    double deter;

    for (size_t i=0; i<r; i++) {
        for (size_t j=0; j<r; j++) {
            b[i][j] = fac[j][i];
        }
    }

    deter = determinantQt(num, r);

    for (size_t i=0; i<r; i++) {
        for (size_t j=0; j<r; j++) {
            inverse[i][j] = b[i][j] / deter;
        }
    }
}

double polyFit::determinantQt(QVector<QVector<double> > a, const size_t k)
{
    double s = 1;
    double det = 0.;
    QVector<QVector<double> > b = makeQtMat(k,k);
    size_t m;
    size_t n;

    if (k == 1) return (a[0][0]);

    for (size_t c=0; c<k; c++) {

        m = 0;
        n = 0;

        for (size_t i = 0; i < k; i++) {

            for (size_t j = 0; j < k; j++) {

                b[i][j] = 0;

                if (i != 0 && j != c) {

                    b[m][n] = a[i][j];
                    if (n < (k - 2)) {
                        n++;
                    }
                    else
                    {
                        n = 0;
                        m++;
                    }
                }
            }
        }

        det = det + s * (a[0][c] * determinantQt(b, k - 1));
        s = -1 * s;

    }

    return (det);
}


QVector<QVector<double> > polyFit::MatMulQt(const size_t m1, const size_t m2, const size_t m3, QVector<QVector<double> > A, QVector<QVector<double> > B)
{
    QVector<QVector<double> > array = makeQtMat(m1,m3);

    for (size_t i=0; i<m1; i++) {
        for (size_t j=0; j<m3; j++) {
            array[i][j]=0.;
            for (size_t m=0; m<m2; m++) {
                array[i][j]+=A[i][m]*B[m][j];
            }
        }
    }
    return array;
}


QVector<QVector<double> > polyFit::MatTransQt(QVector<QVector<double> > array, int rows, int cols)
{
    QVector<QVector<double> > arrayT = makeQtMat(cols,rows);

    for(size_t i = 0; i < rows; i++) {
        for(size_t j = 0; j < cols; j++) {
            arrayT[j][i] = array[i][j];
        }
    }

    return arrayT;
}

void polyFit::polyFitQt(QVector<double> &x, QVector<double> &y, int n, int k, QVector<double> &beta, QVector<QVector<double> > &Weights, QVector<QVector<double> > &XTWXInv)
{
    // Definition of variables
    // **************************************************************
    QVector<QVector<double> > X = makeQtMat(n,k+1);           // [n,k+1]
    QVector<QVector<double> > XT;                               // [k+1,n]
    QVector<QVector<double> > XTW;                              // [k+1,n]
    QVector<QVector<double> > XTWX;                             // [k+1,k+1]

    QVector<double> XTWY = QVector<double>(k+1);
    QVector<double> Y = y;

    size_t begin = 0;


    // Initialize X
    // **************************************************************
    for (size_t i=0; i<n; i++) {
        for (size_t j=begin; j<(k+1); j++) {  // begin
          X[i][j]=pow(x[i],j);
        }
    }

    // Matrix calculations
    // **************************************************************
    XT = MatTransQt(X, n, k+1);                 // Calculate XT
    XTW = MatMulQt(k+1,n,n,XT,Weights);         // Calculate XT*W
    XTWX = MatMulQt(k+1,n,k+1,XTW,X);           // Calculate (XTW)*X

    cofactorQt(XTWX, XTWXInv, k+1);             // Calculate (XTWX)^-1

    MatVectMulQt(k+1,n,XTW,Y,XTWY);             // Calculate (XTW)*Y

    MatVectMulQt(k+1,k+1,XTWXInv,XTWY,beta);    // Calculate beta = (XTWXInv)*XTWY

}

QVector<QVector<double> > polyFit::makeQtMat(int rows, int cols)
{
    QVector<QVector<double>> matReturn = QVector<QVector<double>>(rows);
    for(int i = 0;i<rows;i++){
        matReturn[i]=QVector<double>(cols);
    }
    return matReturn;
}

void polyFit::displayPolyQt(size_t k)
{
    cout << "y = ";
    for (size_t i=0; i<(k+1); i++) {
        cout << "A" << i;
        if (i>0) cout << "X";
        if (i>1) cout << "^" << i;
        if (i<k) cout << " + ";
    }
    cout << endl << endl;

}

QVector<double> polyFit::getSlopeStatic(QVector<double> xVec, QVector<double> yVec, double **Weights)
{
    size_t k = 2;                                    // Polynomial order
    bool fixedinter = false;                         // Fixed the intercept (coefficient A0)
    double fixedinterval = 0.;                       // The fixed intercept value (if applicable)

    double* x = new double[xVec.size()]{0};
    double* y = new double[yVec.size()]{0};
    for(int i = 0; i<xVec.size(); i++){
        x[i] = xVec[i];
        y[i] = yVec[i];
    }

    // Definition of other variables
    // **************************************************************
    size_t n = 0;                                    // Number of data points (adjusted later)
    size_t nstar = 0;                                // equal to n (fixed intercept) or (n-1) not fixed
    double coefbeta[3]={0,0,0};                            // Coefficients of the polynomial


    double **XTWXInv;                                // Matrix XTWX Inverse [k+1,k+1]


    // Initialize values
    // **************************************************************
    n = xVec.size();
    nstar = n-1;
    if (fixedinter) nstar = n;

    if (k>nstar) {
        cout << "The polynomial order is too high. Max should be " << n << " for adjustable A0 ";
        cout << "and " << n-1 << " for fixed A0. ";
        cout << "Program stopped" << endl;
        return {-100,-100};
    }

    if (k==nstar) {
        cout << "The degree of freedom is equal to the number of points. ";
        cout << "The fit will be exact." << endl;
    }

    XTWXInv = Make2DArray(k+1,k+1);

    // **************************************************************
    PolyFit(x,y,n,k,fixedinter,fixedinterval,coefbeta,Weights,XTWXInv);

    return {coefbeta[1],coefbeta[0]};
}

QVector<double> polyFit::getSlopeStaticQt(QVector<double> xVec, QVector<double> yVec, QVector<QVector<double> > Weights, int order)
{
    //alles mit fixed Inter kann ich ignorieren!!
    size_t k = order;                                           //Poly-order
    size_t n = xVec.size();                                    // Number of data points (adjusted later)
    size_t nstar = n - 1;                                // equal to n (fixed intercept) or (n-1) not fixed
    QVector<double> coefbeta=QVector<double>(k+1);                            // Coefficients of the polynomial


    if (k>nstar) {
        qDebug() << "The polynomial order is too high. Max should be " << n << " for adjustable A0 ";
        qDebug() << "and " << n-1 << " for fixed A0. ";
        qDebug() << "Program stopped" << endl;
        return {-100,-100};
    }

    if (k==nstar) {
        qDebug() << "The degree of freedom is equal to the number of points. ";
        qDebug() << "The fit will be exact." << endl;
    }

    QVector<QVector<double>> XTWXInv = makeQtMat(k+1,k+1);

    polyFitQt(xVec,yVec,n,k,coefbeta,Weights,XTWXInv);

    return {coefbeta[1],coefbeta[0]};
}

