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

void dejitter::readFile(QString path)
{
    curveCoordinates = QVector<QVector<float>>(1000);
    QFile file(path);
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


QVector<int> dejitter::getVerticalColorVector(QImage image, int Y)
{
    QVector<int> vec = QVector<int>(image.width());
    vec.fill(0,image.width());

    for(int i = 0;i<image.width();i++){
     vec[i]=getColor(image,i,Y);
    }
    return vec;
}

int dejitter::getIndexOfFirstValue(QVector<int> vec)
{
    for(int index = 10; index < vec.size(); index++){
        if(vec[index]>10){
            return index;
        }
    }
    return 0;
}

float dejitter::getBoarderCoordinate(QImage sinogram, int Y)
{
    auto vector = getVerticalColorVector(sinogram,Y);
    int index = getIndexOfFirstValue(vector);
    float boarder = getArithmicMiddle(vector,index,7);
    return boarder;
}

int dejitter::getColor(QImage image, int x, int y)
{
    QColor color = image.pixelColor(x,y);
    return color.green();
}

QImage dejitter::moveVerticalPixelsBy(QImage image, int offset, int row)
{
    if(offset==0){
        return image;
    }else if(offset>0){
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
            quint16*dstImage = (quint16*)(image.bits()+row*image.bytesPerLine());
            if(x>image.width()+offset){
                dstImage[x]=0;
            }else{
                dstImage[x]=dstImage[x-offset];
            }
        }
    }
    return image;
}

QImage dejitter::dejitterSinogram(QImage sinogram, QVector<float> offsetList)
{
    QImage dejiSino = sinogram;
    dejiSino.fill(0);
    for(int y = 0; y<sinogram.height();y++){
        quint16 *dstDeji = (quint16*)(dejiSino.bits()+y*dejiSino.bytesPerLine());
        quint16 *dstSinogram = (quint16*)(sinogram.bits()+y*sinogram.bytesPerLine());
        if(offsetList[y]==0){
            for(int x = 0; x<sinogram.width();x++){
                dstDeji[x]=dstSinogram[x];
            }
        }else if(offsetList[y]>1){
            float factorX1=int(offsetList[y])+1-offsetList[y];
            float factorX2=offsetList[y]-int(offsetList[y]);
            for(int x = 0; x < dejiSino.width()-offsetList[y]; x++){
                dstDeji[x+int(offsetList[y])]=dstSinogram[x]*factorX1+dstDeji[x+int(offsetList[y])];
                dstDeji[x+1+int(offsetList[y])]=dstSinogram[x]*factorX2+dstDeji[x+1+int(offsetList[y])];
            }
        }else{
            float factorX1 = std::abs(offsetList[y]-int(offsetList[y]));
            float factorX2 = std::abs(int(offsetList[y])-1-offsetList[y]);
            for(int x = -int(offsetList[y])+1;x<dejiSino.width();x++){
                dstDeji[x+int(offsetList[y])-1]=dstSinogram[x]*factorX1+dstDeji[x+int(offsetList[y])-1];
                dstDeji[x+int(offsetList[y])]=dstSinogram[x]*factorX2+dstDeji[x+int(offsetList[y])];
            }
        }
        }
    return dejiSino;
    }


void dejitter::on_pushButton_loadMap_clicked()
{
    QString inputPathMap = ui->lineEdit_dejitterMap->text();
    inputPathMap.replace("\\","/",Qt::CaseSensitivity());
    readFile(inputPathMap);
}

void dejitter::on_pushButton_testMath_clicked()
{
    QImage looser;
    looser.load("C:/Users/o.hill/Pictures/oct_handling/surface_steepness/sinogram_PMT/sinogram_PMT0001");
    readFile("C:/Users/o.hill/Pictures/oct_handling/surface_steepness/sinogram_PMT/sinogram_PMT0001");
    QVector<float> offset = makeCurveFromList(curveCoordinates,looser.height());
    for(int y = 0; y<looser.height(); y++){
        looser = moveVerticalPixelsBy(looser,int(offset[y]),y);
    }
    looser.save("C:/Users/o.hill/Pictures/oct_handling/surface_steepness/sinogram_PMT/sinogram_PMT0001_offset.png");
    looser.save("C:/Users/o.hill/Pictures/oct_handling/surface_steepness/sinogram_PMT/sinogram_PMT0001_offset.tif");
}

void dejitter::on_pushButton_loadMatchingPD_clicked()
{
    QString inputPathMatchingSinogram = ui->lineEdit_matchingSinogram->text();
    inputPathMatchingSinogram.replace("\\","/",Qt::CaseSensitivity());
    QImage matchingSinogram;
    if(!matchingSinogram.load(inputPathMatchingSinogram)){
        qDebug()<<"Failed to load Matching Sinogram. Check file path!";
    }
    QVector<float> offsetSource = makeCurveFromList(curveCoordinates,matchingSinogram.height());
    offsetMap = QVector<float>(matchingSinogram.height());
    for(int y = 0;y<matchingSinogram.height();y++){
        float boarder = getBoarderCoordinate(matchingSinogram,y);
        offsetMap[y]=offsetSource[y]-boarder;
    }
    qDebug()<<offsetMap;
//    for(int y = 0; y<matchingSinogram.height(); y++){
//        matchingSinogram = moveVerticalPixelsBy(matchingSinogram,offsetMap[y],y);
//    }
//    matchingSinogram = dejitterSinogram(matchingSinogram,offsetMap);
//    matchingSinogram.save("C:/Users/o.hill/Pictures/oct_handling/surface_steepness/31-test-4-move.png");
}


void dejitter::on_pushButton_loadPdStack_clicked()
{
    const QString folderpathSurfaceStack = QFileDialog::getExistingDirectory(this,tr("Surface Folder"),"C:/Users/o.hill/Pictures/oct_handling/surface_steepness/");
    QDir dir(folderpathSurfaceStack);
    PDlist = dir.entryInfoList();
    PDlist.removeFirst();
    PDlist.removeFirst();
    dejitterPD = true;
}


void dejitter::on_pushButton_loadPmtStack_clicked()
{
    const QString folderpathSurfaceStack = QFileDialog::getExistingDirectory(this,tr("Folder"),"C:/Users/o.hill/Pictures/oct_handling/surface_steepness/");
    QDir dir(folderpathSurfaceStack);
    PMTlist = dir.entryInfoList();
    PMTlist.removeFirst();
    PMTlist.removeFirst();
    dejitterPMT = true;
}


void dejitter::on_pushButton_dejitterStack_clicked()
{
    const QString folderpathSave = QFileDialog::getExistingDirectory(this,tr("Surface Folder"),"C:/Users/o.hill/Pictures/oct_handling/surface_steepness/");
    QDir saveDir(folderpathSave);
    QString savePath = saveDir.absolutePath();
    savePath.append("/");
    qDebug()<<savePath;
    if(dejitterPD==true){
        for(int i = 0; i<PDlist.size();i++){
            QImage bufSinogram;
            bufSinogram.load(PDlist[i].absoluteFilePath());
            bufSinogram = dejitterSinogram(bufSinogram,offsetMap);
            QString name = "korrigiertesPD_Sinogram_";
            name.append(QString::number(i)).append(".png");
            bufSinogram.save(savePath + name);
            qDebug()<<"PD Sinogram Nr"<<i<<"korrigiert.";
        }
    }
    if(dejitterPMT==true){
        for(int i = 0; i<PMTlist.size();i++){
            QImage bufSinogram;
            bufSinogram.load(PMTlist[i].absoluteFilePath());
            bufSinogram = dejitterSinogram(bufSinogram,offsetMap);
            QString name = "korrigiertesPMT_Sinogram_";
            name.append(QString::number(i)).append(".png");
            bufSinogram.save(savePath + name);
            qDebug()<<"PMT Sinogram Nr"<<i<<"korrigiert.";
        }
    }
}

