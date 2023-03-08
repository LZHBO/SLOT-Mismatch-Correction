#include "header/threadboi.h"

threadBoi::threadBoi(QObject *parent) : QObject(parent)
{
    mStop =false;
}

void threadBoi::start(QString name)
{
    mStop = false;
    for(int i = 0; i<1000;i++){
        if(mStop) return;
        qDebug()<<"From Thread: " << QString::number(i);
        emit on_number(name,i,QThread::currentThread()->objectName());
        QThread::msleep(500);
    }
}

void threadBoi::thinOutSurfaceThreaded(QImage surface, int i)
{
    QImage output = QImage(surface.size(),QImage::Format_Grayscale8);
    output.fill(0);
    for(int x = 0;x<surface.width();x++){
        int y=getFirstValueFromTop(surface,x);
        if(y!=0){
            QVector<int> ascan = fillVectorWithAscan(surface,x);
            double middle = getArithmicMiddle(ascan,y,1);
            output.setPixelColor(x,std::round(middle),255);
        }
    }
    for(int x = 0;x<surface.width();x++){
        int y=getFirstValueFromBottom(surface,x);
        if(y!=0){
            QVector<int> ascan = fillVectorWithAscan(surface,x);
            double middle = getArithmicMiddle(ascan,y,-1);
            output.setPixelColor(x,std::round(middle),255);
        }
    }
    emit thinnedOutSurface(output,i);
    //return output;
}
void threadBoi::stop()
{
    mStop = true;
}

int threadBoi::getFirstValueFromTop(QImage image, int X)
{
    QElapsedTimer timer;
    timer.start();
    for(int y = 0;y<image.height();y++){
        if(getColor(image,X,y)!=0){
            //qDebug()<<"First Value from top nsec:"<<timer.nsecsElapsed();
            return y;
        }
    }
    return 0;
}

int threadBoi::getFirstValueFromBottom(QImage image, int X)
{
    for(int y = image.height()-1;y>=0;y--){
        if(getColor(image,X,y)!=0){
            return y;
        }
    }
    return 0;
}

int threadBoi::getColor(QImage image, int x, int y)
{
    QColor color = image.pixelColor(x,y);
    return color.green();
}

QVector<int> threadBoi::fillVectorWithAscan(QImage image, int X)
{
    QVector<int> vec = QVector<int>(image.height());
    vec.fill(0,image.height());

    for(int i = 0;i<image.height();i++){
     vec[i]=getColor(image,X,i);
    }
    return vec;
}

double threadBoi::getArithmicMiddle(QVector<int> vec, int base, int integral)
{
        if(base==0){
            //qDebug()<<"0 was passed to this function";
            return 0;
        }
        double sum = 0;
        double frac = 0;
        if(integral>0){
            for(int i=0;i<=integral;i++){
                sum = sum + (base + i)*vec[base+i];
                frac = frac + vec[base + i];
            }
        }else{
            for(int i=0;i>=integral;i--){
                sum = sum + (base + i)*vec[base+i];
                frac = frac + vec[base + i];
            }
        }
        return double (sum/frac);
}

