#include "header/dejitter.h"
#include "ui_dejitter.h"

dejitter::dejitter(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::dejitter)
{
    ui->setupUi(this);
}

dejitter::~dejitter()
{
    delete ui;
}

void dejitter::readFile()
{
    curveCoordinates = QVector<QVector<float>>(10);
    QFile file("C:/Users/o.hill/Pictures/oct_handling/surface_steepness/sinogram_PMT/sinogram_PMT0001_offset.txt");
    if(!file.open(QIODevice::ReadOnly)){
     qDebug()<<"Could not open file!";
     qDebug()<<file.errorString();
        return;
    }
    else {
        qDebug()<<"file opened!";
    }

    file.seek(0);
    QTextStream stream(&file);
    int i = 0;
    while (!stream.atEnd()){
        QString line = stream.readLine();
        QString one = line.split("\t").first();
        QString two = line.split("\t").last();
        qDebug()<<one;
        qDebug()<<two;
        curveCoordinates[i]={one.toFloat(),two.toFloat()};
        i++;
    }
    curveCoordinates.resize(i);
    qDebug()<<curveCoordinates;
    file.close();
}

QVector<float> dejitter::makeCurveFromList(QVector<QVector<float>> list, int height)
{
    QVector<float> curve = QVector<float>(height);
    int i = 0;
    int k;
    while(i<=list[0][1]){
            curve[i]=list[0][0];
            i++;
        }
    for(k = 0;k<list.size()-1;k++)
        while(i<=list[k+1][1]){
            curve[i]=getPointAlongLine(list[k][0],list[k][1],list[k+1][0],list[k+1][1],i);
            i++;
        }
    while(i<height){
        curve[i]=list[k][0];
        i++;
    }
    return curve;
}

float dejitter::getPointAlongLine(float x1, float y1, float x2, float y2, int y)
{
    float m = (x2-x1)/(y2-y1);
    float b = x1 - m*y1;
    return m*y + b;
}

float dejitter::getArithmicMiddle(QVector<int> vec, int base, int integral)
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

QVector<int> dejitter::getVerticalColor(QImage image, int Y)
{
    QVector<int> vec = QVector<int>(image.width());
    vec.fill(0,image.width());

    for(int i = 0;i<image.width();i++){
     vec[i]=getColor(image,i,Y);
    }
    return vec;
}

int dejitter::getColor(QImage image, int x, int y)
{
    QColor color = image.pixelColor(x,y);
    return color.green();
}

QImage dejitter::moveVerticalPixelsBy(QImage image, int offset, int row)
{
    if(offset>0){
        for(int x = image.width()-1; x >= 0 ; x--){
            quint16 *dstImage = (quint16*)(image.bits()+row*image.bytesPerLine());
            if(x<offset){
                dstImage[x]=0;
            }else{
                dstImage[x]=dstImage[x-offset];
            }
        }
    }else{
        for(int x = 0; x < image.width(); x++){
            quint16 *dstImage = (quint16*)(image.bits()+row*image.bytesPerLine());
            if(x>image.width()+offset){
                dstImage[x]=0;
            }else{
                dstImage[x]=dstImage[x-offset];
            }
        }
    }
    return image;
}

void dejitter::on_pushButton_loadMap_clicked()
{
    readFile();
}


void dejitter::on_pushButton_loadSinograms_clicked()
{

}

void dejitter::on_pushButton_testMath_clicked()
{
    QImage looser;
    looser.load("C:/Users/o.hill/Pictures/oct_handling/surface_steepness/sinogram_PMT/sinogram_PMT0001");
    readFile();
    QVector<float> offset = makeCurveFromList(curveCoordinates,looser.height());
    for(int y = 0; y<looser.height(); y++){
        looser = moveVerticalPixelsBy(looser,int(offset[y]),y);
    }
    looser.save("C:/Users/o.hill/Pictures/oct_handling/surface_steepness/sinogram_PMT/sinogram_PMT0001_offset.png");
    looser.save("C:/Users/o.hill/Pictures/oct_handling/surface_steepness/sinogram_PMT/sinogram_PMT0001_offset.tif");
}
