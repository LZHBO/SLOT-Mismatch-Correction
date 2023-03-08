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

class threadBoi : public QObject
{
    Q_OBJECT
public:
    explicit threadBoi(QObject *parent = nullptr);
    void start(QString name);
    void thinOutSurfaceThreaded(QImage surface, int i);

signals:
    void on_number(QString name, int number, QString threadID);
    void thinnedOutSurface(QImage,int i);
public slots:
    void stop();

private:
    bool mStop;
    int getFirstValueFromTop(QImage image, int X);
    int getFirstValueFromBottom(QImage image, int X);
    int getColor(QImage image, int x, int y);
    double getArithmicMiddle(QVector<int> vec, int base, int integral);
    QVector<int> fillVectorWithAscan(QImage image, int X);

};

#endif // THREADBOI_H
