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

    void on_pushButton_loadSinograms_clicked();

    void on_pushButton_testMath_clicked();

private:

    Ui::dejitter *ui;
    QImage transmissionSinogram;
    QImage fluorescenceSinogram;
    QVector<QVector<float>> curveCoordinates;
    QVector<float> offsetList;
    QVector<QImage> transmissionStack;
    QVector<QImage> fluorescenceStack;

    void readFile();

    QVector<float> makeCurveFromList(QVector<QVector<float>> list, int height);

    float getPointAlongLine(float x1, float y1, float x2, float y2, int y);

    float getArithmicMiddle(QVector<int> vec, int base, int integral);

    QVector<int> getVerticalColor(QImage image, int Y);

    int getColor(QImage image, int x, int y);
    /** Positiver Offset bedeutet, dass die Zeile nach rechts verschoben werden soll
     *
     */
    QImage moveVerticalPixelsBy(QImage image, int offset, int row);

};

#endif // DEJITTER_H
