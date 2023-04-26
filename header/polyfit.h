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
    int maini(int argc, char *argv[]);
    double getSlope(QVector<double> xVec, QVector<double> yVec, double **Weights);

signals:

};

#endif // POLYFIT_H
