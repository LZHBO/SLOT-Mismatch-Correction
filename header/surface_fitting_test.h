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
#include <QFile>
#include <QTextStream>
#include <arrayfire.h>
#include "header/threadboi.h"
#include <QtConcurrent>
#include <QFuture>
#include <QFutureWatcher>
#include <header/refraction.h>
#include <header/polyfit.h>
#include <iostream>
#include <QTime>



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

    /**
     * Enthält alle Informationen über den Eingangspunkt des Strahls
     */
    struct entryPoint{
        /**
         * X-Koordinate des eingehendes Strahls
         */
        double xEntry = 0;
        /**
         * Y-Koordinate des eingehendes Strahls
         */
        double yEntry = 0;
        /**
         * Ablenkung des Eingrangstrahls in radiant von der Senkrechten
         */
        double gammaEntry = 0;
        /**
         * Länge bis der Strahl die Probenrückseite trifft
         */
        double lengthEntry = 0;
        /**
         * Steigung der Oberfläche am Eintrittspunkt
         */
        double slopeEntry = 0;
        /**
         * Steigung der Oberfläche am Austrittspunkt
         */
        double slopeExit = 0;
        /**
          * Y-Koordinate am Austrittspunkt aus der Probe
          */
        double yExit = 0;
        /**
          * X-Koordinate am Austrittspunkt aus der Probe
          */
        double xExit = 0;
        /**
          * X-Koordinaten-Verschiebung am Austrittspunkt aus der Probe im Verhältnix zum Eintrittspunkt
          */
        double xExitRelative = 0;
    };

    struct newSurfaceInfo{
        QImage thinnedOut;
        QImage rotSurface;
        QVector<entryPoint> ePoints;
    };



public:
    explicit surface_fitting_test(QWidget *parent = nullptr);
    static int  propagateRaysMulti(QImage thinnedOutSurface);
    ~surface_fitting_test();
    static double riMediumMT;
    static double riSampleMT;
    static int slopeSigmaMT;
    static int slopePxlNoMT;
    static bool usePoly;
    static int fitOrderMT;

private slots:
    void on_pushButton_loadAndDisplay_clicked();

    void on_spinBox_aScan_valueChanged(int arg1);

    void on_spinBox_slopeSigma_valueChanged(int arg1);

    void on_pushButton_createSinogram_clicked();

    void on_doubleSpinBox_riMedium_valueChanged(double arg1);

    void on_doubleSpinBox_riSample_valueChanged(double arg1);

    void on_spinBox_rotateDisplayImageBy_valueChanged(int arg1);

    void on_pushButton_testMath_clicked();

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

    void on_spinBox_nrOfProjections_valueChanged(int arg1);

    void on_checkBox_useMultiThreading_stateChanged(int arg1);

    void on_radioButton_pmt_toggled(bool checked);

    void on_radioButton_PD_toggled(bool checked);

    void on_pushButton_openOpenCV_clicked();

    void on_spinBox_ariMiddleSigma_valueChanged(int arg1);

    void on_checkBox_usePolyFit_stateChanged(int arg1);

    void on_spinBox_fitOrder_valueChanged(int arg1);

    void on_pushButton_choosePdSinogram_clicked();

    void on_pushButton_analyseCameraSlot_clicked();

    void on_doubleSpinBox_riRange_valueChanged(double arg1);

    void on_doubleSpinBox_searchIncriment_valueChanged(double arg1);

    void on_doubleSpinBox_riAcceptableOffset_valueChanged(double arg1);

    void on_doubleSpinBox_mmPerPixelinReco_valueChanged(double arg1);

    void on_pushButton_saveCameraSlot_clicked();

private:
    Ui::surface_fitting_test *ui;
    refraction *ref;
    polyFit *poly;
    threadBoi thready;
    QString inputPathSurface;
    QString inputPathHisto;
    QString inputPathSinogram;
    QFileInfoList fileListInfoSurface;
    QFileInfoList fileListInfoSinogramPMT;
    QFileInfoList fileListInfoSinogramPD;
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
    int firstIndexFromLeft = 0;
    int firstIndexFromRight = 0;
    int sampleSizePixel = 0;
    int scanPointsCCD = 0;
    double deltaPixel = 0;
    bool createTransmission = 0;
    bool accountForReflection = 0;
    bool useArrayFire = 1;
    bool useMultiThreading = 1;
    bool usePolyFit = 1;
    bool continuousSimulation = false;
    int fitOrder = 0;
    bool correctingPmtSinogram = true;
    signed rotateClockwise = 1;
    QImage rearrangedSinogramFails;
    double mmPerPixelOnSensor = 0.0031;
    double mmPerPixelInReco = 0.008765;
    double correctedSensorRatio;
    QVector<refraction::imageMoments> momentsList;
    QVector<entryPoint> surfacePointsRefraction;
    double riSearchRange = 0.02;
    double riSearchIncriment = 0.001;
    double riAcceptableOffset = 1.0;
//    /**
//     * Punkte für die der korrigierte Wert auf der Rückseite wäre, erster Eintrag Nummer der Projektion, zweiter Eintrag AScan der Projektion
//     */
    QVector<QVector<int>> travelledOnBackside;
    double straightAngle;
    QVector<QImage> rotatedSurfaces;
    QVector<QImage> rotatedSurfacesThinnedOut;
    QVector<QImage> rotatedHistoImages;
//    /**
//     * [Number of tomogram], [x], [y, exit angle Abweichung von senkrechter Geraden in radiant, Länge bis Rückseite getroffen wird, Steigung der Oberfläche]
//     */
//    QVector<QVector<QVector<double>>>rotatedEntryPoints;
    QVector<QVector<entryPoint>>newRotatedEntryPoints;
    int numberOfProjections;
    QString nrOfProjString;

    //Functions:
    /**
     * Diese Funktion projeziert das übergebene Image in das GUI
     */
    void displayImageLeft(QImage image);
    /**
     * Diese Funktion errechnet das arithmetische Mittel des übergebenen Vectors ab dem Punkt "base" mit der Anzahl an Folgepunktenbarpunkten "integral". Wenn man einen negativen Wert für integral übergibt, wird die Richtung, in die das A-Mittel bestimmt wird, getauscht. Zb in Funktionen, die die Rückseite (at back) des InputSurface betreffen
     */
    double getArithmicMiddle(QVector<int> vec, int base, int integral);
    /**
     * Nimmt die Grauwerte aus entlang eines senkrechten vectors (wie ein A-Scan im OCT...) an der horizontalen position X und packt sie in einen Vector
     */
    QVector<int> fillVectorWithAscan(QImage image, int X);
    /**
     * Gibt den Grauwert des bildes "image" an der pixel-position x,y. Das kann man bestimmt schlauer lösen
     */
    int getColor(QImage image, int x, int y);
    /**
     * Wird genutzt, um einen Grauwert direkt in ein QImage zu schreiben. Diese Funktion ist schneller als die entsprechende Funktion von Qt
     */
    QImage setPixelColor8Bit(QImage image, int x, int y, int value, int bytesPerLine);
    /**
     * Gibt die Steigung am Eingangspunkt X (horizontal) des InputSurface wieder. Die Anzahl an berücksichtigten Nachbarn ist sigma. Benutzt linearen Fit
     */
    QVector<double> getSlopeAtEntry(QImage image, int X, int sigma);
    /**
     * Gibt die Steigung am Ausgangspunkt X (horizontal) des InputSurface wieder. Die Anzahl an berücksichtigten Nachbarn ist sigma. Benutzt linearen Fit
     */
    QVector<double> getSlopeAtExit(QImage image, int X, int sigma);
    /**
     * Diese Funktion errechnet den gesuchten Partnerwinkel für die gegebene Steigung und die Brechungsindice 1 und 2. Siehe Dissertation Ole Hill Kapitel 4.1.3. Die "slope" ist die Steigung der Oberfläche.
     */
    double getDeltaThetaForPartner(double slope, double n1, double n2);
    /**
     * Sucht entlang eines "AScans" (vertikal)(Horizontale Koordinate X) beginnend an der Oberkante des Bildes nach dem ersten Wert, der ungleich Null ist und gibt den Index (Y position) dieses Pixels zurück
     */
    int getFirstValueFromTop(QImage image, int X);
    /**
     * Sucht entlang eines "AScans" (vertikal)(Horizontale Koordinate X) beginnend an der Unterkante des Bildes nach dem ersten Wert, der ungleich Null ist und gibt den Index (Y position) dieses Pixels zurück
     */
    int getFirstValueFromBottom(QImage image, int X);
    /**
     * Sucht den am weitest links liegenden Pixel, der ungleich Null ist und gibt den horizontalen Index dieses Pixels zurück
     */
    int getFirstIndexFromLeft(QImage surface);
    /**
     * Sucht den am weitest rechts liegenden Pixel, der ungleich Null ist und gibt den horizontalen Index dieses Pixels zurück
     */
    int getFirstIndexFromRight(QImage surface);
    /**
     * Wendet mehrere Funktionen an, um ein Bild des InputSurfaces zu erschaffen, dass nur noch eine dünne Außenkontur hat. Dabei wird das arithmetische Mittel angewendet.
     */
    QImage thinOutSurface(QImage image);
    /**
     * Erzeugt künstliches Rauschen auf den simulierten BScans. noiseFactor multipliziert den Zufallswert
     */
    QImage noiseBScan(QImage bScan, double noiseFactor);
    /**
     * Führt die komplette Korrektur des übergebenen Sinograms anhand der Surface Geometrie und Ray Propagation durch. Vgl. Dissertation Ole Hill Kapitel 4
     */
    QImage correctExternalSinogram(QImage sinogram, QImage surface, QString surfacePath, double mediumRI, double sampleRI);
    /**
     * Führt die komplette Korrektur des übergebenen Sinograms (PMT und PD) anhand der Surface Geometrie und Ray Propagation durch. Vgl. Dissertation Ole Hill Kapitel 4
     */
    bool correctExternalSinogramPDandPMT(QImage &sinogramPD, QImage &sinogramPMT, QImage &rearSinoPD, QImage &rearSinoPMT, QImage &surface, QString surfacePath, double mediumRI, double sampleRI);
    /**
     * Rechnet die Steigung an der Stelle an, die in der Gui gerade ausgewählt ist. Der Strahlengang wird simuliert und im großen Anzeigefenster der GUI präsentiert
     */
    void drawAndDisplaySlope(QImage image,int X, int Y, double slope, int width);
    /**
     * gibt Winkelabweichung von senkrechter Gerade in radiant wieder, input ist als Steigung deltaY/deltaX
     */
    double getExitAngle(double slope, double n1, double n2); //gibt die Winkelabweichung von der senkrechten Achse zurück
    /**
     * gibt Winkelabweichung von senkrechter Gerade in radiant hinter dem Austrittspunkt aus der Probe wieder, input ist als Steigung deltaY/deltaX, gamma ist die Winkelabweichung von der Senkrechten innerhalb der Probe
     */
    double getExitAngleAtBack(double slope, double nSample, double nMedium, double gamma);
    /**
     * Rechnet die Gaussverteilung eines Punktes gammaX/gammaY aus. Also den Wert an Punkt x/y wenn das Maximum auf gammaX/gammaY liegt. Subpixel und so
     */
    double getValueForGaussDistribution(double gammaX, double gammaY, int x, int y, double sigma);
    /**
     * Dreht den Punkt eines Bildes (inputPoint) um einen Winkel (rotationAngle) um den centerPoint des Bildes.
     */
    QVector<double> rotateImagePointTo(QVector<double> inputPoint, QVector<double> centerPoint, double rotationAngle, bool clockwise);
    /**
     * Erstellt einen parametrisierten Vector aus zwei Punkten
     */
    QVector<double> parameterizeFromPoints(double x1, double y1, double x2, double y2);
    /**
     * Erstellt einen parametrisierten Vector aus einem Punkt und einer Steigung
     */
    QVector<double> parameterizeFromAngle(double x, double y, double angle);
    /**
     * Standard Schnittpunkt von zwei Vectoren
     */
    QVector<double> getIntersectionPoint(QVector<double> ray, QVector<double> surface);
    /**
     * Lineare interpolation zwischen Punkt x1/y1 und x2/y2 an der Stelle x
     */
    double linInterpolation(double x1, double x2, double y1, double y2, double x);
    /**
     * Gibt den interpolierten Grauwert eines Subpixels zurück, Q sind die Grauwerte und x1,x2,y1,y2 die Koordinaten der Nachbarn. x und y sind die Koordinaten des Subpixels
     */
    double bilinInterpolation(double q11, double q12, double q21, double q22, double x1, double x2, double y1, double y2, double x, double y);
    /**
     * Gibt die Koordinaten der benachbarten Bildpunkte, nur für positive Werte: x1, x2, y1, y2
     */
    QVector<int> getClosestNeighbours(double x, double y);
    /**
     * addiert die Grauwerte entlang des Strahls, der am Punkt entryX/entryY unter dem winkel angle und für die Länge length durch das Bild histo propagiert.
     */
    double propagateRayThroughHisto(QImage histo, double entryX, double entryY, double angle, double length);
    /**
     * listet die Grauwerte entlang des Strahls, der am Punkt entryX/entryY unter dem winkel angle und für die Länge length durch das Bild histo propagiert, auf und packt diese Werte in "bscan"
     */
    QImage propagateOctRayThroughHisto(QImage histo, QImage bScan, double entryX, double entryY, double angle, double length, double mediumRi, double sampleRi);
    /**
     * Bilineare Interpolation der vier benachbahrten Grauwerte im Sinogram. Die Position im Sinogram is absX und absProj
     */
    double newBilinInterpolFromSinogram(QImage sino, double absX, double absProj);
    /**
     * rotatedEntryPoint: y entry, exit angle Abweichung von senkrechter Geraden in radiant, Länge bis Rückseite getroffen wird, Steigung der Oberfläche
     */
    double getTransmissionGrade(double riMedium, double riSample, entryPoint point);
    /**
     * Macht einen Stack aus rotierten Bildern des jeweiligen input bildes
     */
    QVector<QImage> makeRotatedImageStack(QString path, int stackSize);
    /**
     * Macht eine Liste aus Structs aus den Inputdaten
     */
    QVector<newSurfaceInfo> makeNewStructVector(QVector<QImage> imageListThinnedSurface, QVector<QImage> imageListRotSurface, QVector<QVector<entryPoint>> pointList);
    /**
     * Macht die Ray Propagation durch alle input Surfaces (meist ein Surface, das rotiert wurde in makeRotatedImageStack()
     */
    static int newPropagateRayMultiThreaded(newSurfaceInfo &surfaceList);
    /**
     * Fittet die Steigung an Punkt X (oben) mit der Anzahl an Nachbarn, die in der UI eingestellt ist. Order ist der Grad des Polynoms, Weighting macht die naheliegenden Nachbarn wichtiger
     */
    QVector<double> getPolySlopeAtEntryQt(QImage &surface, int X, int order, bool applyWeighting);
    /**
     * Fittet die Steigung an Punkt X (unten) mit der Anzahl an Nachbarn, die in der UI eingestellt ist. Order ist der Grad des Polynoms, Weighting macht die naheliegenden Nachbarn wichtiger
     */
    QVector<double> getPolySlopeAtExitQt(QImage &surface, int X, int order, bool applyWeighting);
    /**
     * Hier folgen viele static versionen von anderen  fuktionen.
     */
    static QVector<double> getPolySlopeAtEntryStatic(QImage &surface, int X, int order, bool applyWeighting);
    static int getFirstValueFromTopStatic(QImage &image, int X);
    static int getFirstValueFromBottomStatic(QImage &image, int X);
    static int getColorStatic(QImage &image, int x, int y);
    static QVector<int> fillVectorWithAscanStatic(QImage image, int X);
    static double getArithmicMiddleStatic(QVector<int> vec, int base, int integral);
    static QVector<double> getSlopeAtEntryStatic(QImage image, int X, int sigma);
    static double getExitAngleStatic(double slope, double n1, double n2);
    static QVector<double> parameterizeFromPointsStatic(double x1, double y1, double x2, double y2);
    static QVector<double> parameterizeFromAngleStatic(double x, double y, double angle);
    static QVector<double> getIntersectionPointStatic(QVector<double> ray, QVector<double> surface);
    /**
     * Berechnet den gesamten, relativen Offset in x-richtung auf der Kamera in mm, Input in mm. Vgl. Dissertation Ole Hill Kapitel 6
     */
    double calculateRelativCameraOffset(double mediumRI, double yExit, double xExit, double angleExit);
    /**
     * Bestimmt den Austrittspunkt und -winkel an der Rückseite der Probe. Vgl. Dissertation Ole Hill Kapitel 6
     */
    QVector<double> getBackExitPointAndAngle(QVector<entryPoint> surfacePoints, int xEntry, double mediaRI, double samplRi);
    /**
     * Simuliert eine "Camera SLOT"-Aufnahme anhand einer Liste von surfacePoints (quasi das InputSurface als Liste) und dem RI von Medium und Probe
     */
    QVector<double> generateRefractionPattern(QVector<entryPoint> surfacePoints, double mediaRI, double sampleRI);
    /**
     * Die Punkte der Camera-SLOT Messung, wie nicht durch die Probe gegangen sind, werden rausgeworfen
     */
    QVector<refraction::imageMoments> shortenMomentsList(QVector<refraction::imageMoments> moments);
    /**
     * Korrigiert den telezentrischen Fehler, der dadurch entsteht, dass die Strahlen im SLOT leider nicht perfekt parallel auf die Probe treffen.
     */
    void determineTeleError(QVector<refraction::imageMoments> moments);
    /**
     * Die Liste mit den Momenten wird so geändert, dass die Werte in mm sind und es sich um relative statt absolute Werte handelt. Es ist die Abweichung in mm jedes Momentes von dem Moment, wenn keine Refraktion geschehen wäre
     */
    QVector<refraction::imageMoments> makeListRelativeAndScaled(QVector<refraction::imageMoments> moments);
    /**
     * Berechnet alle Aussenpunkte und Steigungen, KEINE RAYPROPAGATION! alles auf Pixelebene der Rekonstruktion
     */
    QVector<entryPoint> calculateAllSlopes(QImage surface);
     /**
     * @brief Berechnet den RI der Probe anhand der relativen Verschiebung des Kamerapunktes vom erwarteten Punkt bei RI match. xEntry ist der RELATIVE Eintrittspunkt des entsprechenden Kamera-punktes
     * Relative Camera Offset ist die Ablenkung vom erwarteten Punkt. Vgl. Dissertation Ole Hill Kapitel 6
     */
    double determineRelativeCameraOffset(QVector<entryPoint> surfacePoints, int xEntry, double mediumRI, double sampleRI);
    /**
     * testet verschiedene SampleRI durch, bis die gemessene Ablenkung (xMoment) erreicht wird. Vgl. Dissertation Ole Hill Kapitel 6
     */
    QVector<double> getFittingSampleRI(QVector<entryPoint> surfacePoints,int xEntry, double relativeCameraOffset, double mediumRI, double expSampleRI, double riRange, double riIncriment, double excaptableOffset);
    /**
     * Speichert relevante paramater der simulation oder korrektur in dem angegenen pfad
     */
    void saveInfoFile(QString name, QString savepath);
public slots:
    void newNumber(QString name, int number, QString threadID);
    void fillInThinnedSurface(QImage surface, int i);
    void getMomentsList(QVector<refraction::imageMoments> moments);
signals:
    void on_stop();
};

#endif // SURFACE_FITTING_TEST_H
