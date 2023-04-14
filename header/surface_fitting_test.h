#ifndef SURFACE_FITTING_TEST_H
#define SURFACE_FITTING_TEST_H

#include <QWidget>
#include <QPixmap>
#include <QDebug>
#include <QVector>
#include <QList>
#include <cmath>
#include <math.h>
#include <QtMath>
#include <QDir>
#include <QElapsedTimer>
#include <QRandomGenerator>
#include <QFileDialog>
#include <arrayfire.h>
#include "header/threadboi.h"
#include <QtConcurrent>
#include <QFuture>
#include <QFutureWatcher>



namespace Ui {
class surface_fitting_test;
}

class surface_fitting_test : public QWidget
{
    Q_OBJECT

    struct surfaceInfo{
        QImage thinnedOut;
        QVector<QVector<double>> entryPoints;
    };

public:
    explicit surface_fitting_test(QWidget *parent = nullptr);
    static int  propagateRaysMulti(QImage thinnedOutSurface);
    ~surface_fitting_test();
    static double riMediumMT;
    static double riSampleMT;
    static int slopeSigmaMT;

private slots:
    void on_pushButton_loadAndDisplay_clicked();

    void on_pushButton_getSlopeAt_clicked();

    void on_spinBox_aScan_valueChanged(int arg1);

    void on_spinBox_slopeSigma_valueChanged(int arg1);

    void on_pushButton_createSinogram_clicked();

    void on_doubleSpinBox_riMedium_valueChanged(double arg1);

    void on_doubleSpinBox_riSample_valueChanged(double arg1);

    void on_spinBox_rotateDisplayImageBy_valueChanged(int arg1);

    void on_pushButton_correctSinogram_clicked();

    void on_pushButton_testMath_clicked();

    void on_spinBox_arithMiddleSigma_valueChanged(int arg1);

    void on_pushButton_rotateSurfaceAndHisto_clicked();

    void on_pushButton_continousSimulation_clicked();

    void on_pushButton_newCorrection_clicked();

    void on_pushButton_loadAndDisplaySinogram_clicked();

    void on_pushButton_correctInputSinogram_clicked();

    void on_checkBox_createTransmission_toggled(bool checked);

    void on_checkBox_accountForReflection_stateChanged(int arg1);

    void on_pushButton_chooseSurfaceDirectory_clicked();

    void on_pushButton_ChooseSinogramDirectory_clicked();

    void on_pushButton_correctStack_clicked();

    void on_checkBox_useArrayFire_stateChanged(int arg1);

    void on_pushButton_varyingRiCorrection_clicked();

    void on_doubleSpinBox_varyingMediumRi_valueChanged(double arg1);

    void on_doubleSpinBox_mediumRiIncrement_valueChanged(double arg1);

    void on_checkBox_clockwise_stateChanged(int arg1);

    void on_pushButton_startThread_clicked();

    void on_pushButton_stopThread_clicked();

    void on_spinBox_nrOfProjections_valueChanged(int arg1);

    void on_checkBox_useMultiThreading_stateChanged(int arg1);


    void on_radioButton_pmt_toggled(bool checked);

    void on_radioButton_PD_toggled(bool checked);

private:
    Ui::surface_fitting_test *ui;
    threadBoi thready;
    QString inputPathSurface;
    QString inputPathHisto;
    QString inputPathSinogram;
    QFileInfoList fileListInfoSurface;
    QFileInfoList fileListInfoSinogram;
    QString nameRI;
    double riMedium;
    double riSample;
    double targetMediumRi;
    double riIncrement;
    int arithMiddleSigma;
    QImage inputSurface;
    QImage inputHisto;
    QImage inputSinogram;
    QImage bufferImg;
    QImage sinogramHisto;
    QImage transmissionHisto;
    QImage bScan;
    bool createTransmission = 0;
    bool accountForReflection = 0;
    bool useArrayFire = 0;
    bool useMultiThreading = 0;
    bool correctingPmtSinogram = true;
    signed rotateClockwise = 1;
    QImage rearrangedSinogramFails;
//    /**
//     * Punkte für die der korrigierte Wert auf der Rückseite wäre, erster Eintrag Nummer der Projektion, zweiter Eintrag AScan der Projektion
//     */
    QVector<QVector<int>> travelledOnBackside;
    double straightAngle;
    QVector<QImage> rotatedSurfacesThinnedOut;
    QVector<QImage> rotatedHistoImages;
    /**
     * [Number of tomogram], [x], [y, exit angle Abweichung von senkrechter Geraden in radiant, Länge bis Rückseite getroffen wird, Steigung der Oberfläche]
     */
    QVector<QVector<QVector<double>>>rotatedEntryPoints;
    int numberOfProjections;
    QString nrOfProjString;
    void displayImageLeft(QImage image);
    double getArithmicMiddle(QVector<int> vec, int base, int integral);
    //double getArithmicMiddleAF(QVector<int> vec, int base, int integral);
    QVector<int> fillVectorWithAscan(QImage image, int X);
    int getColor(QImage image, int x, int y);
    QImage setPixelColor8Bit(QImage image, int x, int y, int value, int bytesPerLine);
    QVector<double> getSlopeAtEntry(QImage image, int X, int sigma);
    QVector<double> getSlopeAtExit(QImage image, int X, int sigma);
    double getDeltaThetaForPartner(double slope, double n1, double n2);
    int getFirstValueFromTop(QImage image, int X);
    int getFirstValueFromBottom(QImage image, int X);
    QImage thinOutSurface(QImage image);
    QImage noiseBScan(QImage bScan, double noiseFactor);
    QImage correctExternalSinogram(QImage sinogram, QImage surface, QString surfacePath, double mediumRI, double sampleRI);
    void drawAndDisplaySlope(QImage image,int X, int Y, double slope, int width);
    /**
     * gibt Winkelabweichung von senkrechter Gerade in radiant wieder, input ist als Steigung deltaY/deltaX
     */
    double getExitAngle(double slope, double n1, double n2); //gibt die Winkelabweichung von der senkrechten Achse zurück
    double getExitAngleAtBack(double slope, double nSample, double nMedium, double gamma);
    double getValueForGaussDistribution(double gammaX, double gammaY, int x, int y, double sigma);
    QVector<double> rotateImagePointTo(QVector<double> inputPoint, QVector<double> centerPoint, double rotationAngle, bool clockwise);
    QVector<double> parameterizeFromPoints(double x1, double y1, double x2, double y2);
    QVector<double> parameterizeFromAngle(double x, double y, double angle);
    QVector<double> getIntersectionPoint(QVector<double> ray, QVector<double> surface);
    double linInterpolation(double x1, double x2, double y1, double y2, double x);
    /**
     * Gibt den interpolierten Grauwert eines Subpixels zurück, Q sind die Grauwerte und x1,x2,y1,y2 die Koordinaten der Nachbarn. x und y sind die Koordinaten des Subpixels
     */
    double bilinInterpolation(double q11, double q12, double q21, double q22, double x1, double x2, double y1, double y2, double x, double y);
    /**
     * Gibt die Koordinaten der benachbarten Bildpunkte, nur für positive Werte: x1, x2, y1, y2
     */
    QVector<int> getClosestNeighbours(double x, double y);
    double propagateRayThroughHisto(QImage histo, double entryX, double entryY, double angle, double length);
    QImage propagateOctRayThroughHisto(QImage histo, QImage bScan, double entryX, double entryY, double angle, double length, double mediumRi, double sampleRi);
    double bilinInterpolFromSinogram(QImage sino, double x, double gamma, int startProj, int deltaProj);
    double newBilinInterpolFromSinogram(QImage sino, double absX, double absProj);
    /**
     * rotatedEntryPoint: y entry, exit angle Abweichung von senkrechter Geraden in radiant, Länge bis Rückseite getroffen wird, Steigung der Oberfläche
     */
    double getTransmissionGrade(double riMedium, double riSample, QVector<double> rotatedEntryPoint);
    QVector<QImage> makeRotatedImageStack(QString path, int stackSize);
    void learningAF(int size);
    QVector<surfaceInfo> makeStructVector(QVector<QImage> imageList, QVector<QVector<QVector<double>>> pointList);
    static int propagateRayMultiThreaded(surfaceInfo &surfaceList);

    static int getFirstValueFromTopStatic(QImage image, int X);
    static int getFirstValueFromBottomStatic(QImage image, int X);
    static int getColorStatic(QImage image, int x, int y);
    static QVector<double> getSlopeAtEntryStatic(QImage image, int X, int sigma);
    static double getExitAngleStatic(double slope, double n1, double n2);
    static QVector<double> parameterizeFromPointsStatic(double x1, double y1, double x2, double y2);
    static QVector<double> parameterizeFromAngleStatic(double x, double y, double angle);
    static QVector<double> getIntersectionPointStatic(QVector<double> ray, QVector<double> surface);
    double calculateCameraPoint(double mediumRI, double yExit, double xExit, double angleExit);
    /**
     * [Entry Point: y, exit angle Abweichung von senkrechter Geraden in radiant, Länge bis Rückseite getroffen wird, Steigung der Oberfläche]
     */
    QVector<double> getBackExitPointAndAngle(QImage thinSurface, int xEntry, double mediaRI, double samplRi);
    QVector<double> generateRefractionPattern(QImage thinSurface, double mediaRI, double sampleRI);

public slots:
    void newNumber(QString name, int number, QString threadID);
    void fillInThinnedSurface(QImage surface, int i);
signals:
    void on_stop();
};

#endif // SURFACE_FITTING_TEST_H
