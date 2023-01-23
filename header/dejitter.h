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

private:

    Ui::dejitter *ui;
    QImage transmissionSinogram;
    QImage fluorescenceSinogram;
    QVector<QVector<float>> curveCoordinates;
    QVector<QImage> transmissionStack;
    QVector<QImage> fluorescenceStack;
    QVector<float> offsetMap;
    QFileInfoList PDlist;
    QFileInfoList PMTlist;
    bool dejitterPD = 0;
    bool dejitterPMT = 0;

    void readFile(QString path);

    QVector<float> makeCurveFromList(QVector<QVector<float>> list, int height);

    float getPointAlongLine(float x1, float y1, float x2, float y2, int y);

    float getArithmicMiddle(QVector<int> vec, int base, int integral);

    QVector<int> getVerticalColorVector(QImage image, int Y);

    int getIndexOfFirstValue(QVector<int> vec);

    float getBoarderCoordinate(QImage sinogram, int Y);

    int getColor(QImage image, int x, int y);
    /** Positiver Offset bedeutet, dass die Zeile nach rechts verschoben werden soll
     *
     */
    QImage moveVerticalPixelsBy(QImage image, int offset, int row);

    QImage dejitterSinogram(QImage sinogram, QVector<float> offsetList);

};

#endif // DEJITTER_H
