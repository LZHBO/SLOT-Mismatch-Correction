#ifndef POLYFIT_H
#define POLYFIT_H

#include <QObject>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <cmath>
#include <iomanip>
#include <QDebug>

using namespace std;

#define MAXIT 100
#define EPS 3.0e-7
#define FPMIN 1.0e-30
#define STOP 1.0e-8
#define TINY 1.0e-30



class polyFit : public QObject
{
    Q_OBJECT
public:
    explicit polyFit(QObject *parent = nullptr);
    int maini();
    QVector<double> getSlope(QVector<double> xVec, QVector<double> yVec, double **Weights);

    static void cofactorQt(QVector<QVector<double>> &num, QVector<QVector<double>> &inverse, const size_t f);

    static void MatVectMulQt(const size_t m1, const size_t m2, QVector<QVector<double>> &A, QVector<double> &v, QVector<double> &Av);

    static void transposeQt(QVector<QVector<double>> &num, QVector<QVector<double>> &fac, QVector<QVector<double>> &inverse, const size_t r);

    static double determinantQt(QVector<QVector<double>> a, const size_t k);

    static QVector<QVector<double>> MatMulQt(const size_t m1, const size_t m2, const size_t m3, QVector<QVector<double>> A, QVector<QVector<double>> B);

    static QVector<QVector<double>> MatTransQt(QVector<QVector<double>> array, int rows, int cols);

    static void polyFitQt(QVector<double> &x,QVector<double> &y,int n,int k,QVector<double> &beta, QVector<QVector<double>> &Weights,QVector<QVector<double>> &XTWXInv);

    static QVector<QVector<double>> makeQtMat(int rows, int cols);

    static QVector<double> getSlopeStatic(QVector<double> xVec, QVector<double> yVec, double **Weights);

    static QVector<double> getSlopeStaticQt(QVector<double> xVec, QVector<double> yVec, QVector<QVector<double>> Weights);

signals:

};

#endif // POLYFIT_H
