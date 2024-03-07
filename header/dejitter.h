#ifndef DEJITTER_H
#define DEJITTER_H

#include <QWidget>
#include <QPixmap>
#include <QDebug>
#include <QVector>
#include <cmath>
#include <math.h>
#include <QtMath>
#include <QDir>
#include <QElapsedTimer>
#include <QRandomGenerator>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>

namespace Ui {
class dejitter;
}

class dejitter : public QWidget
{
    Q_OBJECT

public:
    explicit dejitter(QWidget *parent = nullptr);
    ~dejitter();

private slots:
    void on_pushButton_loadMap_clicked();

    void on_pushButton_testMath_clicked();

    void on_pushButton_loadMatchingPD_clicked();

    void on_pushButton_loadPdStack_clicked();

    void on_pushButton_loadPmtStack_clicked();

    void on_pushButton_dejitterStack_clicked();

    void on_pushButton_loadSinolation_clicked();

    void on_pushButton_dejitterStackSinolation_clicked();

private:

    Ui::dejitter *ui;
    QImage transmissionSinogram;
    QImage fluorescenceSinogram;
    QImage sinolation;
    QImage matchingSinogram;
    QVector<QVector<float>> curveCoordinates;
    QVector<QImage> transmissionStack;
    QVector<QImage> fluorescenceStack;
    QVector<float> offsetMap;
    QFileInfoList PDlist;
    QFileInfoList PMTlist;
    bool dejitterPD = 0;
    bool dejitterPMT = 0;

    /**
     * läd die txt datei mit den Koordinaten ein
     */
    void readFile(QString path);
    /**
     * Macht eine durchgehende Liste aus der input (list) in dem zwischen den genannten Punkten interpoliert wird
     */
    QVector<float> makeCurveFromList(QVector<QVector<float>> list, int height);
    /**
     * Fitted eine Liste an die linke Außenkante des simulierten Sinograms.
     */
    QVector<float> makeCurveFromSimulatedSinogramLeft(QImage sinolation);
    /**
     * Fitted eine Liste an die rechte Außenkante des simulierten Sinograms.
     */
    QVector<float> makeCurveFromSimulatedSinogramRight(QImage sinolation);
    /**
     * lineare Interpolation zwischen den Punkten x1/y1 und x2/y2 an Position y
     */
    float getPointAlongLine(float x1, float y1, float x2, float y2, int y);
    /**
     * Arithmetisches Mittel von base bis base + integral
     */
    float getArithmicMiddle(QVector<int> vec, int base, int integral);
    /**
     * Füllt einen Vector mit den horizontalen Grauwerten eines Bildes auf Höhe Y
     */
    QVector<int> getHorizontalColorVector(QImage image, int Y);
    /**
     * geht den Vektor vec vom ersten Wert (vec[0]) durch bis der erste Wert ungleich Null gefunden wird. Der index dessen wird zurückgegeben
     */
    int getIndexOfFirstValueLeft(QVector<int> vec);
    /**
     * geht den Vektor vec vom letzten Wert (vec[vec.size()-1]) durch bis der erste Wert ungleich Null gefunden wird. Der index dessen wird zurückgegeben
     */
    int getIndexOfFirstValueRight(QVector<int> vec);
    /**
     * gibt den Wert (Y Koordinate) der linken Kante im Sinogram auf Höhe Y an. Wendet arithmetisches Mittel an
     */
    float getBoarderCoordinateLeft(QImage sinogram, int Y);
    /**
     * gibt den Wert (Y Koordinate) der rechten Kante im Sinogram auf Höhe Y an. Wendet arithmetisches Mittel an
     */
    float getBoarderCoordinateRight(QImage sinogram, int Y);
    /**
     * Gibt den Grauwert des Pixels zurück
     */
    int getColor(QImage image, int x, int y);
    /** Positiver Offset bedeutet, dass die Zeile nach rechts verschoben werden soll
     * bewegt die ganze Zeile (row) im Bild image um einen Faktor offset
     */
    QImage moveVerticalPixelsBy(QImage image, int offset, int row);
    /**
     *
     */
    QImage dejitterSinogram(QImage sinogram, QVector<float> offsetList, bool pmt);
    /**
     *
     */
    void testfunction(int* x, int* y, QImage* image);

};

#endif // DEJITTER_H
