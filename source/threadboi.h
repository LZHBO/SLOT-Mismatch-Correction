#ifndef THREADBOI_H
#define THREADBOI_H

#include <QObject>

class threadBoi : public QObject
{
    Q_OBJECT
public:
    explicit threadBoi(QObject *parent = nullptr);

signals:

};

#endif // THREADBOI_H
