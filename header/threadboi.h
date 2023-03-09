#ifndef THREADBOI_H
#define THREADBOI_H

#include <QObject>
#include <QDebug>
#include <QThread>
#include <QtConcurrent>
#include <QFuture>
#include <QImage>
#include <QtMath>
#include <QElapsedTimer>
#include <QVector>
#include <QList>

class threadBoi : public QObject
{
    Q_OBJECT
public:
    explicit threadBoi(QObject *parent = nullptr);
    void start(QString name);
    void getRotatedSurfaces(QVector<QImage> rotatedSurfaces);
    static int thinOutSurfaceThreaded(QImage &surface);
    struct dings{
        QImage rotSurf;
        QVector<QVector<double>> rotPoints;
    };
    static int propagateRays(dings &pair);

signals:
    void on_number(QString name, int number, QString threadID);
    void thinnedOutSurface(QImage,int i);
public slots:
    void stop();

private:

    bool mStop;
    QVector<QImage> rotatedSurfacesInThreadBoi;
    QVector<QVector<QVector<double>>> rotatedEntryPointsInThreadBoi;
    static int getFirstValueFromTop(QImage image, int X);
    static int getFirstValueFromBottom(QImage image, int X);
    static int getColor(QImage image, int x, int y);
    static double getArithmicMiddle(QVector<int> vec, int base, int integral);
    static QVector<int> fillVectorWithAscan(QImage image, int X);

};

#endif // THREADBOI_H
