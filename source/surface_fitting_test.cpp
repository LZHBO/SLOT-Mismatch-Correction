#include "header/surface_fitting_test.h"
#include "ui_surface_fitting_test.h"

double surface_fitting_test::riMediumMT = 1;
double surface_fitting_test::riSampleMT = 1;
int surface_fitting_test::slopeSigmaMT = 1;

surface_fitting_test::surface_fitting_test(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::surface_fitting_test)
{
    ui->setupUi(this);
}

surface_fitting_test::~surface_fitting_test()
{
    emit on_stop();
    delete ui;
}

void surface_fitting_test::on_pushButton_loadAndDisplay_clicked()
{
    arithMiddleSigma = ui->spinBox_arithMiddleSigma->value();
    inputPathSurface = ui->lineEdit_inputSurface->text();
    inputPathSurface.replace("\\","/",Qt::CaseSensitivity());
    if(inputSurface.load(inputPathSurface)==0){
        qDebug()<<"Failed to load input surface, check path";
    };
    inputSurface.convertTo(QImage::Format_Grayscale8,Qt::AutoColor);
    inputPathHisto = ui->lineEdit_inputHisto->text();
    inputPathHisto.replace("\\","/",Qt::CaseSensitivity());
    if(inputHisto.load(inputPathHisto)==0){
        qDebug()<<"Failed to load input histogram, check path";
    };
    inputHisto.convertTo(QImage::Format_Grayscale8,Qt::AutoColor);
    bufferImg = QImage(inputSurface.size(),QImage::Format_Grayscale8);
    bufferImg = thinOutSurface(inputSurface);
    displayImageLeft(inputSurface);
    riMedium = ui->doubleSpinBox_riMedium->value();
    riSample = ui->doubleSpinBox_riSample->value();
    numberOfProjections = ui->spinBox_nrOfProjections->value();
}

void surface_fitting_test::on_pushButton_loadAndDisplaySinogram_clicked()
{
    arithMiddleSigma = ui->spinBox_arithMiddleSigma->value();
    inputPathSurface = ui->lineEdit_inputSurface->text();
    inputPathSurface.replace("\\","/",Qt::CaseSensitivity());
    if(inputSurface.load(inputPathSurface)==0){
        qDebug()<<"Failed to load input surface, check path";
    };
    inputSurface.convertTo(QImage::Format_Grayscale8,Qt::AutoColor);
    inputPathSinogram = ui->lineEdit_inputSinogram->text();
    inputPathSinogram.replace("\\","/",Qt::CaseSensitivity());
    if(inputSinogram.load(inputPathSinogram)==0){
        qDebug()<<"Failed to load input sinogram, check path";
    };
    //inputHisto.convertTo(QImage::Format_Grayscale8,Qt::AutoColor);
    auto sinogramFormat = inputSinogram.pixelFormat();
    qDebug()<<"Sinogram Bits per Pixel:"<<sinogramFormat.bitsPerPixel();
    qDebug()<<"Sinogram Color Model (3 is grayscale)"<<sinogramFormat.colorModel();
    if(inputSinogram.width()!=inputSurface.width()){
        qDebug()<<"Sinogram and Surface don't match!!";
    }
    bufferImg = QImage(inputSurface.size(),QImage::Format_Grayscale8);
    bufferImg = thinOutSurface(inputSurface);
    displayImageLeft(inputSurface);
    numberOfProjections = inputSinogram.height()-2;
    riMedium = ui->doubleSpinBox_riMedium->value();
    riSample = ui->doubleSpinBox_riSample->value();
    targetMediumRi = ui->doubleSpinBox_varyingMediumRi->value();
    riIncrement = ui->doubleSpinBox_mediumRiIncrement->value();
}

void surface_fitting_test::displayImageLeft(QImage image)
{
    QPixmap displayLeft = QPixmap::fromImage(image);
    if(ui->checkBox_fitToLabel->isChecked()){
        ui->label_leftDisplay->setPixmap(displayLeft.scaled(ui->label_leftDisplay->width(),ui->label_leftDisplay->height(),Qt::KeepAspectRatio));
    }else{
        ui->label_leftDisplay->setPixmap(displayLeft.scaled(displayLeft.size()));
    }
}

double surface_fitting_test::getArithmicMiddle(QVector<int> vec, int base, int integral)
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

//double surface_fitting_test::getArithmicMiddleAF(QVector<int> vec, int base, int integral)
//{
//    QElapsedTimer timer;
//    timer.start();
//    if(base==0){
//        //qDebug()<<"0 was passed to this function";
//        return 0;
//    }
//    af::array baseAF = range(af::dim4(integral));
//    qDebug()<<timer.elapsed();
//    baseAF = baseAF.operator+=(base); //das sollte die Base zu jedem Element des Arrays hinzufügen...
//    af_print(baseAF);
//    vec.remove(base + integral-1,1+vec.size()-base-integral);
//    vec.remove(0,base-1);
//    qDebug()<<vec;
//    int slorp[4];
//    for(int i = 0; i< integral;i++){
//        slorp[i]=vec[i];
//    }
//    af::array vecAF(4,1,slorp);
//    af_print(vecAF);
//    vecAF = vecAF.operator*=(baseAF);
//    af_print(vecAF);
//    qDebug()<<timer.elapsed();
//    return 0;
//}

QVector<int> surface_fitting_test::fillVectorWithAscan(QImage image, int X)
{
    QVector<int> vec = QVector<int>(image.height());
    vec.fill(0,image.height());

    for(int i = 0;i<image.height();i++){
     vec[i]=getColor(image,X,i);
    }
    return vec;
}

int surface_fitting_test::getColor(QImage image, int x, int y)
{
    QColor color = image.pixelColor(x,y);
    return color.green();
}

QImage surface_fitting_test::setPixelColor8Bit(QImage image, int x, int y, int value, int bytesPerLine)
{
    quint8 *dstColor = (quint8*)(image.bits()+y*bytesPerLine);
    dstColor[x] = value;
    return image;
}

QVector<double> surface_fitting_test::getSlopeAtEntry(QImage image, int X, int sigma)
{
    if(getFirstValueFromTop(image,X)==0){
        //qDebug()<<"Keine Oberfläche im AScan gefunden";
        QVector<double> d{999,999};
        return d; //keine Oberfläche im Ascan
    }
    double Sxx = 0;
    double Sxy = 0;
    double yQuer = 0;
    double xQuer = 0;
    double b = 0;
    int sumX = 0;
    int fracX = 0;
    for(int x = X-sigma;x<=X+sigma;x++){
        if(getFirstValueFromTop(image,x)!=0){
            sumX = sumX + x;
            fracX = fracX + 1;
        }
    }
    xQuer = sumX/fracX;
    int sumY = 0;
    int fracY = 0;
    for(int x = X-sigma;x<=X+sigma;x++){
        if(getFirstValueFromTop(image,x)!=0){
            sumY = sumY + getFirstValueFromTop(image,x);
            fracY=fracY+1;
            Sxx = Sxx + (x-xQuer)*(x-xQuer);
        }
    }
    yQuer = sumY/fracY;
    for(int x = X-sigma;x<=X+sigma;x++){
        if(getFirstValueFromTop(image,x)!=0){
            Sxy = Sxy + (x-xQuer)*(getFirstValueFromTop(image,x)-yQuer);
        }
    }
    b = yQuer - (Sxy/Sxx)*xQuer;
    double yReturn = b + (Sxy/Sxx)*X;
    //QVector<double> d{yReturn,Sxy/Sxx};
    //return d; // Vorzeichenwechsel scheint falsch herum, weil der Nullpunkt von Bildern oben links ist
    return {yReturn,Sxy/Sxx};
}

QVector<double> surface_fitting_test::getSlopeAtExit(QImage image, int X, int sigma)  // quasi real-time
{
    if(getFirstValueFromBottom(image,X)==0){
        QVector<double> d{999,999};
        return d; //keine Oberfläche im Ascan
    }
    double Sxx = 0;
    double Sxy = 0;
    double yQuer = 0;
    double xQuer = 0;
    double b = 0;
    int sumX = 0;
    int fracX = 0;
    for(int x = X-sigma;x<=X+sigma;x++){
        if(getFirstValueFromBottom(image,x)!=0){
            sumX = sumX + x;
            fracX = fracX + 1;
        }
    }
    xQuer = sumX/fracX;
    int sumY = 0;
    int fracY = 0;
    for(int x = X-sigma;x<=X+sigma;x++){
        if(getFirstValueFromBottom(image,x)!=0){
            sumY = sumY + getFirstValueFromBottom(image,x);
            fracY=fracY+1;
            Sxx = Sxx + (x-xQuer)*(x-xQuer);
        }
    }
    yQuer = sumY/fracY;
    for(int x = X-sigma;x<=X+sigma;x++){
        if(getFirstValueFromBottom(image,x)!=0){
            Sxy = Sxy + (x-xQuer)*(getFirstValueFromBottom(image,x)-yQuer);
        }
    }
    b = yQuer - (Sxy/Sxx)*xQuer;
    double yReturn = b + (Sxy/Sxx)*X;
    //QVector<double> d{yReturn,Sxy/Sxx};
    //return d; // Vorzeichenwechsel scheint falsch herum, weil der Nullpunkt von Bildern oben links ist
    return {yReturn,Sxy/Sxx};
}

double surface_fitting_test::getDeltaThetaForPartner(double slope, double n1, double n2)
{
    if(n1==n2||slope==0){
        return 0.0;
    }else if((n2*qSin(qAtan(slope)))/n1<-1){
        return 999;
    }else if((n2*qSin(qAtan(slope)))/n1>1){
        return 999;
    }else{
        return qAsin(n2/n1*qSin(qAtan(slope)))-qAtan(slope);
    }
}
int surface_fitting_test::getFirstValueFromTop(QImage image, int X)
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

int surface_fitting_test::getFirstValueFromBottom(QImage image, int X)
{
    for(int y = image.height()-1;y>=0;y--){
        if(getColor(image,X,y)!=0){
            return y;
        }
    }
    return 0;
}

QImage surface_fitting_test::thinOutSurface(QImage image)
{
//    QElapsedTimer timer,grandTimer;
//    timer.start();
//    grandTimer.start();
    QImage output = QImage(image.size(),QImage::Format_Grayscale8);
    output.fill(0);
    for(int x = 0;x<image.width();x++){
        int y=getFirstValueFromTop(image,x);
        //qDebug()<<"getFirstValueFromTop"<<timer.nsecsElapsed();
        //timer.restart();
        if(y!=0){
            QVector<int> ascan = fillVectorWithAscan(image,x);
            //qDebug()<<"Vector mit AScan gefüllt"<<timer.nsecsElapsed();
            //timer.restart();
            double middle = getArithmicMiddle(ascan,y,arithMiddleSigma);
            //qDebug()<<"getArithmicMiddle"<<timer.nsecsElapsed();
            //timer.restart();
            output.setPixelColor(x,std::round(middle),255);
            //output = setPixelColor8Bit(output,x,std::round(middle),255,bytesPerLine);
            //qDebug()<<"setPixelColorQBit"<<timer.nsecsElapsed();
            //timer.restart();
        }
    }
    for(int x = 0;x<image.width();x++){
        int y=getFirstValueFromBottom(image,x);
        if(y!=0){
            QVector<int> ascan = fillVectorWithAscan(image,x);
            double middle = getArithmicMiddle(ascan,y,-arithMiddleSigma);
            //output = setPixelColor8Bit(output,x,std::round(middle),255,bytesPerLine);
            output.setPixelColor(x,std::round(middle),255);
        }
    }
    //qDebug()<<"Ein Surface thinned out"<<grandTimer.elapsed();
    return output;
}

QImage surface_fitting_test::noiseBScan(QImage bScan, double noiseFactor)
{
    for(int x = 0; x < bScan.width();x++){
        for(int y = 0; y < bScan.height();y++){
            int col = getColor(bScan,x,y);
            int random = QRandomGenerator::global()->bounded(30);
            col = col + std::round(random*noiseFactor);
            if(col>255){
                col = 255;
            }
            QColor gray = QColor(col,col,col,255);
            bScan.setPixelColor(x,y,gray);
        }
    }
    return bScan;
}

QImage surface_fitting_test::correctExternalSinogram(QImage sinogram, QImage surface, QString surfacePath, double mediumRI, double sampleRI)  //Beim Übergang von der letzten zur ersten Proj geht noch was schief... Grauwerte haben einen scheinbar festen Offset
{
    riMediumMT = mediumRI;
    riSampleMT = sampleRI;
    QElapsedTimer externalCorrectionTimer;
    externalCorrectionTimer.start();
    int slopeSigma = ui->spinBox_slopeSigma->value();
    slopeSigmaMT = ui->spinBox_slopeSigma->value();
    int numberOfAngles = sinogram.height()-2;
    rotatedSurfacesThinnedOut = QVector<QImage>(numberOfAngles);
    rotatedEntryPoints = QVector<QVector<QVector<double>>>(numberOfAngles);
    newRotatedEntryPoints = QVector<QVector<entryPoint>>(numberOfAngles);
    QVector<surfaceInfo> surfaceList;
    QVector<newSurfaceInfo> newSurfaceList;
    if(useArrayFire){
        rotatedSurfacesThinnedOut = makeRotatedImageStack(surfacePath, numberOfAngles);
    }
    if(useMultiThreading){
        QtConcurrent::blockingMap(rotatedSurfacesThinnedOut,&threadBoi::thinOutSurfaceThreaded);
        for(int k = 0;k<numberOfAngles;k++){
            rotatedEntryPoints[k]=QVector<QVector<double>>(rotatedSurfacesThinnedOut[0].width());
            newRotatedEntryPoints[k]=QVector<entryPoint>(rotatedSurfacesThinnedOut[0].width());
        }
        qDebug()<<"Struct wird erstellt"<<externalCorrectionTimer.restart();
        surfaceList = makeStructVector(rotatedSurfacesThinnedOut,rotatedEntryPoints);
        newSurfaceList = makeNewStructVector(rotatedSurfacesThinnedOut,newRotatedEntryPoints);
        qDebug()<<"Struct wurde erstellt"<<externalCorrectionTimer.restart();
        QtConcurrent::blockingMap(surfaceList,&surface_fitting_test::propagateRayMultiThreaded);
        QtConcurrent::blockingMap(newSurfaceList,&surface_fitting_test::newPropagateRayMultiThreaded);
        qDebug()<<"Strahlen wurden MT propagiert"<<externalCorrectionTimer.restart();
        for(int k=0; k<surfaceList.size();k++){
            rotatedEntryPoints[k] = surfaceList[k].entryPoints;
        }
        for(int k=0; k<newSurfaceList.size();k++){
            newRotatedEntryPoints[k] = newSurfaceList[k].ePoints;
        }
        qDebug()<<"Werte wurden kopiert"<<externalCorrectionTimer.restart();
    }else{
    qDebug()<<"Single Threading gestartet für ray propagation"<<externalCorrectionTimer.restart();
    for(int k = 0;k<numberOfAngles;k++){
        if(!useArrayFire){
            QVector<double>rotatedPoint=QVector<double>(2);
            double rotateBy = (2*M_PI/double(numberOfAngles))*k;
            rotatedSurfacesThinnedOut[k] = QImage(surface.size(),QImage::Format_Grayscale8);
            rotatedSurfacesThinnedOut[k].fill(0);
            for(int x = 0;x<surface.width();x++){
                for(int y = 0;y<surface.height();y++){
                    if(getColor(surface,x,y)!=0){
                        rotatedPoint=rotateImagePointTo({double(x),double(y)},{double(surface.width()/2.0),double(surface.height()/2.0)},rotateClockwise*rotateBy,true);
                        int xRound = std::round(rotatedPoint[0]);
                        int yRound = std::round(rotatedPoint[1]);
                        for(int xGauss = xRound-2;xGauss<=xRound+2;xGauss++){
                            for(int yGauss = yRound-2;yGauss<=yRound+2;yGauss++){
                                int newInt = getColor(rotatedSurfacesThinnedOut[k],xGauss,yGauss) + int(200.0*getValueForGaussDistribution(rotatedPoint[0],rotatedPoint[1],xGauss,yGauss,1));
                                if(newInt>255){
                                    newInt=255;
                                    qDebug()<<"Grauwert zu hoch";
                                }
                                quint8 *dstRotSurface = (quint8*)(rotatedSurfacesThinnedOut[k].bits()+yGauss*rotatedSurfacesThinnedOut[k].bytesPerLine());
                                dstRotSurface[xGauss] = newInt;
                            }
                        }
                    }
                }
            }
        }
        if(!useMultiThreading){
            rotatedSurfacesThinnedOut[k]=thinOutSurface(rotatedSurfacesThinnedOut[k]);
        }
        rotatedEntryPoints[k] = QVector<QVector<double>>(surface.width());
        newRotatedEntryPoints[k] = QVector<entryPoint>(surface.width());
        rotatedEntryPoints[k][0]={0,0,0,0};
        newRotatedEntryPoints[k][0]={0,0,0,0,0};
        newRotatedEntryPoints[k][0].xEntry = 0;
        //Strahl durch die "Projektion" propagieren
        for(int x = 1;x<surface.width();x++){
            rotatedEntryPoints[k][x]={0,0};
            newRotatedEntryPoints[k][x].xEntry = x;
            if(getFirstValueFromTop(rotatedSurfacesThinnedOut[k],x-1)!=0 && getFirstValueFromTop(rotatedSurfacesThinnedOut[k],x)!=0 && getFirstValueFromTop(rotatedSurfacesThinnedOut[k],x+1)!=0){ //dadurch werden die äußersten Punkte ignoriert
                QVector<double>bufVec = getSlopeAtEntry(rotatedSurfacesThinnedOut[k],x,slopeSigma);
                double exitAngle = getExitAngle(bufVec[1],mediumRI,sampleRI);
                rotatedEntryPoints[k][x] = {bufVec[0],exitAngle};
                rotatedEntryPoints[k][x].append(0);
                newRotatedEntryPoints[k][x].gammaEntry = exitAngle;
                newRotatedEntryPoints[k][x].slopeEntry = bufVec[1];
                newRotatedEntryPoints[k][x].yEntry = bufVec[0];
                if(exitAngle>=0&&exitAngle<1.0){  //Strahl wird nach rechts abgelenkt
                    int X=x;
                    bool success = false;
                    while(success == false && getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X+1)>0){
                        QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X), X+1, getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X+1));
                        QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                        QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                        if(intersection[0]>=X&&intersection[0]<X+1){
                            rotatedEntryPoints[k][x][2] = intersection[2];
                            newRotatedEntryPoints[k][x].lengthEntry = intersection[2];
                            success = true;
                        }else{
                            X++;
                        }
                    }
                    if(success == false){
                        //qDebug()<<"Strahl wurde nach rechts abgelenkt und fand keine Rückseite bei: "<<k<<x;
                        if(getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X)!=getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X)){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X), X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[1]>=getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X)&&intersection[1]<=getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X)){
                                //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                                rotatedEntryPoints[k][x][2] = intersection[2];
                                newRotatedEntryPoints[k][x].lengthEntry = intersection[2];
                                success = true;
                            }
                        }
                        while(success == false && X>=x){
                            QVector<double> surfaceVector = parameterizeFromPoints(X-1, getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X-1), X, getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[0]>=X&&intersection[0]<X+1){
                                rotatedEntryPoints[k][x][2] = intersection[2];
                                newRotatedEntryPoints[k][x].lengthEntry = intersection[2];
                                //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                                success = true;
                            }else{
                                X--;
                            }
                        }
                    }
                }else if(exitAngle<0&&exitAngle>-1.0){  //Strahl wird nach links abgelenkt
                    int X=x;
                    bool success = false;
                    while(success == false && getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X-1) > 0){
                        QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X), X-1, getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X-1));
                        QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                        QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                        if(intersection[0]<=X&&intersection[0]>X-1){
                            rotatedEntryPoints[k][x][2] = intersection[2];
                            newRotatedEntryPoints[k][x].lengthEntry = intersection[2];
                            success = true;
                        }else{
                            X--;
                        }
                    }
                    if(success == false){
                        //qDebug()<<"Strahl wurde nach links abgelenkt und fand keine Rückseite bei: "<<k<<x;
                        if(getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X)!=getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X)){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X), X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[1]>=getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X)&&intersection[1]<=getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X)){
                                //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                                rotatedEntryPoints[k][x][2] = intersection[2];
                                newRotatedEntryPoints[k][x].lengthEntry = intersection[2];
                                success = true;
                            }
                        }
                        while(success == false && X<=x){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X), X+1, getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X+1));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[0]>=X&&intersection[0]<X+1){
                                rotatedEntryPoints[k][x][2] = intersection[2];
                                newRotatedEntryPoints[k][x].lengthEntry = intersection[2];
                                //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                                success = true;
                            }else{
                                X++;
                            }
                        }
                    }
                }else{
                    //qDebug()<<"Strahl wurde total reflektiert!"<<k<<x;
                }

                rotatedEntryPoints[k][x].append(bufVec[1]);
                // Wert in ein Sinogram malen
                if(rotatedEntryPoints[k][x][2]<0){
                    rotatedEntryPoints[k][x][2]=0;
                    qDebug()<<"irgendwas schiefgelaufen beim rayPropagate";
                }
                if(newRotatedEntryPoints[k][x].lengthEntry < 0){
                    newRotatedEntryPoints[k][x].lengthEntry = 0;
                }

            }else{
                rotatedEntryPoints[k][x] = {0,0,0,0};
            }
        }
        qDebug()<<"Rays propagated for Input Surface "<<k<<"of "<<numberOfAngles<<", time: "<<externalCorrectionTimer.elapsed();
    }
    }
    QElapsedTimer reTimer;
    reTimer.start();
    QImage rearrangedExternalSinogram = QImage(sinogram.size(),QImage::Format_Grayscale16);
    if(correctingPmtSinogram){
        rearrangedExternalSinogram.fill(0);
    }else{
        rearrangedExternalSinogram.fill(36000);
    }
    QVector<QVector<int>> travelledOnBackside;
    for(int p = 0; p<numberOfAngles;p++){
        quint16 *dstArrSino = (quint16*)(rearrangedExternalSinogram.bits()+p*rearrangedExternalSinogram.bytesPerLine());
        for(int x = 0; x<surface.width();x++){
            if(newRotatedEntryPoints[p][x].yEntry!=0){
                if(!correctingPmtSinogram){
                    quint16 *dstSinoStraight = (quint16*)(sinogram.bits()+p*sinogram.bytesPerLine());
                    dstArrSino[x]=dstSinoStraight[x];
                }
                if(newRotatedEntryPoints[p][x].gammaEntry==0){                                                  //bei senkrechten Strahlen wird nichts verschoben!
                    quint16 *dstSinoStraight = (quint16*)(sinogram.bits()+p*sinogram.bytesPerLine());
                    dstArrSino[x]=dstSinoStraight[x];
                }else{
                    double deltaTheta = getDeltaThetaForPartner(newRotatedEntryPoints[p][x].slopeEntry,mediumRI,sampleRI);
                    if(deltaTheta!=999){
                        double deltaProj = deltaTheta*double(numberOfAngles)/(2*M_PI);
                        QVector<double> buf = rotateImagePointTo({double(x),newRotatedEntryPoints[p][x].yEntry},{double(surface.width()/2),double(surface.height()/2)},deltaTheta,true);
                        double absNewX = buf[0];
                        long double value = newBilinInterpolFromSinogram(sinogram,absNewX,double(p)+rotateClockwise*deltaProj);
                        if(accountForReflection==true){
                            int newP = std::round(p+deltaProj);
                            newP = (newP+numberOfAngles)%numberOfAngles;
                            value = value/getTransmissionGrade(mediumRI,sampleRI,newRotatedEntryPoints[newP][std::round(absNewX)]); // muss noch auf neues Struct umgeschrieben werden ... hier müsste eigentlich der Winkel vom Partner angegeben werden!! Deswegen zu hell
                        }
                        if(value<0){
                            qDebug()<<"Wert war unter Null bei: "<<p<<x;
                            value=0;
                        }else if(value>65000){
                            value=65000;
                            qDebug()<<"Wert aus Sinograminterpolation war zu hoch";
                        }
                        //qDebug()<<"crash after line 462";
                        dstArrSino[x] = std::round(value);
                    }else{
                        travelledOnBackside.append({p,x});
                        //rearrangedSinogramFails.setPixelColor(x,p,Qt::red);
                    }
                }

            }
        }
    } //Am Ende noch die letzten beiden Zeilen aus der ersten kopieren
    quint16 *dstArrSinoBot = (quint16*)(rearrangedExternalSinogram.bits()+numberOfAngles*rearrangedExternalSinogram.bytesPerLine());
    quint16 *dstArrSinoTop = (quint16*)(rearrangedExternalSinogram.bits());
    for(int x = 0;x<sinogram.width();x++){
        dstArrSinoBot[x] = dstArrSinoTop[x];
    }
    quint16 *dstArrSinoBot2 = (quint16*)(rearrangedExternalSinogram.bits()+(numberOfAngles+1)*rearrangedExternalSinogram.bytesPerLine());
    for(int x = 0;x<sinogram.width();x++){
        dstArrSinoBot2[x] = dstArrSinoTop[x];
    }
    for(int r = 0;r<travelledOnBackside.size();r++){ // Wenn Punkte bei Korrektur auf die Rückseite gewandert sind, wird hier der Wert "von der anderen Seite" kopiert
        int fromX = sinogram.width()-travelledOnBackside[r][1]-1;
        quint16 *dstArrSinoSource = (quint16*)(rearrangedExternalSinogram.bits()+((travelledOnBackside[r][0]+numberOfAngles/2)%numberOfAngles)*rearrangedExternalSinogram.bytesPerLine());
        quint16 *dstArrSinoFailed = (quint16*)(rearrangedExternalSinogram.bits()+(travelledOnBackside[r][0])*rearrangedExternalSinogram.bytesPerLine());
        if(dstArrSinoSource[fromX]!=0){
            //rearrangedSinogramFails.setPixelColor(travelledOnBackside[r][1],travelledOnBackside[r][0],Qt::yellow);
            dstArrSinoFailed[travelledOnBackside[r][1]] = dstArrSinoSource[fromX];
        }
    }

    rotatedEntryPoints.resize(0);
    newRotatedEntryPoints.resize(0);
    rotatedSurfacesThinnedOut.resize(0);
    //rearrangedSinogramFails.save(inputPathSinogram+"\\"+nameRI2+nrOfProjString2+"_sinogramRearrangedFailsFromExternal.png");
    //rearrangedExternalSinogram.save(inputPathSinogram+"\\"+nameRI2+nrOfProjString2+"_sinogramRearrangedFromExternal.png"); Umstellung von save auf return!
    //rearrangedSinogramFails.fill(0);
    qDebug()<<"Sinogram fertig sortiert!"<<externalCorrectionTimer.elapsed();
    return rearrangedExternalSinogram;
}

void surface_fitting_test::drawAndDisplaySlope(QImage image, int X, int Y, double slope, int width)
{
    QImage drawn = image;
    for(int x = X-width/2;x<=X+width/2;x++){
        int y = std::round(Y + (x-X)*slope);
        drawn.setPixelColor(x,y,Qt::green);
    }
    double gamma = getExitAngle(slope,riMedium,riSample);
    qDebug()<<gamma;
    for(int y = 0;y<150;y++){
        drawn.setPixelColor(X+std::round(y*qTan(gamma)),y+Y,Qt::red);
    }
    displayImageLeft(drawn);
}

double surface_fitting_test::getExitAngle(double slope, double n1, double n2) //quasi real-time
{
    if(n1==n2||slope==0){
        return 0.0;
    }else if((n1*qSin(qAtan(slope)))/n2<-1){
        return -1.0;
    }else if((n1*qSin(qAtan(slope)))/n2>1){
        return 1.0;
    }else{
        return -(qAtan(slope) - qAsin((n1*qSin(qAtan(slope)))/n2));
    }
}

double surface_fitting_test::getExitAngleAtBack(double slope, double nSample, double nMedium, double gamma)
{
    if(nSample==nMedium){
        return gamma;
    }else if((nSample*qSin(qAtan(slope)+gamma))/nMedium<-1){
        return -1.0;
    }else if((nSample*qSin(qAtan(slope)+gamma))/nMedium>1){
        return 1.0;
    }else{
        return qAsin((nSample*qSin(gamma + qAtan(slope)))/nMedium) - qAtan(slope);
    }
}

double surface_fitting_test::getValueForGaussDistribution(double gammaX, double gammaY, int x, int y, double sigma)
{
    //double value = (1/(2*M_PI*pow(sigma,2)))*exp(((-1)/(2*pow(sigma,2)))*(pow(x-gammaX,2)+pow(y-gammaY,2)));
    //return value;
    return (1/(2*M_PI*pow(sigma,2)))*exp(((-1)/(2*pow(sigma,2)))*(pow(x-gammaX,2)+pow(y-gammaY,2)));
}

QVector<double> surface_fitting_test::rotateImagePointTo(QVector<double> inputPoint, QVector<double> centerPoint, double rotationAngle, bool clockwise)
{
    QVector<double> exitPoint = {0,0};
    QVector<double> translationVector = {0,0};
    QVector<QVector<double>> rotationMatrix = {{0,0},{0,0}};
    if(clockwise){
        rotationMatrix[0][0]=qCos(rotationAngle);
        rotationMatrix[0][1]=-qSin(rotationAngle);
        rotationMatrix[1][0]=qSin(rotationAngle);
        rotationMatrix[1][1]=qCos(rotationAngle);
    }
    else{
        rotationMatrix[0][0]=qCos(double(2*M_PI - rotationAngle));
        rotationMatrix[0][1]=-qSin(double(2*M_PI - rotationAngle));
        rotationMatrix[1][0]=qSin(double(2*M_PI - rotationAngle));
        rotationMatrix[1][1]=qCos(double(2*M_PI - rotationAngle));
    }
    translationVector[0] = centerPoint[0]-(rotationMatrix[0][0]*centerPoint[0]+rotationMatrix[0][1]*centerPoint[1]);
    translationVector[1] = centerPoint[1]-(rotationMatrix[1][0]*centerPoint[0]+rotationMatrix[1][1]*centerPoint[1]);
    exitPoint[0] = (rotationMatrix[0][0]*inputPoint[0]+rotationMatrix[0][1]*inputPoint[1]) + translationVector[0];
    exitPoint[1] = (rotationMatrix[1][0]*inputPoint[0]+rotationMatrix[1][1]*inputPoint[1]) + translationVector[1];
    return exitPoint;
}


QVector<double> surface_fitting_test::parameterizeFromPoints(double x1, double y1, double x2, double y2)
{
    QVector<double> fuPa = QVector<double>(4);
    if(x1==x2){
        fuPa[0] = x1;
        fuPa[1] = y1;
        fuPa[2] = 0;
        fuPa[3] = 1;
        return fuPa;
    }
    fuPa[0] = x1;
    fuPa[1] = y1;
    fuPa[2] = qCos(qAtan((y2-y1)/(x2-x1)));
    fuPa[3] = qSin(qAtan((y2-y1)/(x2-x1)));
    return fuPa;
}

QVector<double> surface_fitting_test::parameterizeFromAngle(double x, double y, double angle)
{
    QVector<double> fuPa = QVector<double>(4);
    fuPa[0] = x;
    fuPa[1] = y;
    fuPa[2] = qSin(angle);
    fuPa[3] = qCos(angle);
    return fuPa;
}

QVector<double> surface_fitting_test::getIntersectionPoint(QVector<double> ray, QVector<double> surface)
{
    QVector<double> intersection = QVector<double>{0,0,0};
    double x1 = ray[0];
    double x2 = surface[0];
    double y1 = ray[1];
    double y2 = surface[1];
    double mX1 = ray[2];
    double mX2 = surface[2];
    double mY1 = ray[3];
    double mY2 = surface[3];
    if(mX2==0){
        double t = (y1-y2+(mY1*(x2-x1))/mX1)/(mY2-(mY1*mX2)/mX1);
        intersection[0] = x2+mX2*t;
        intersection[1] = y2+mY2*t;
        intersection[2] = sqrt(pow(intersection[0]-x1,2)+pow(intersection[1]-y1,2));
        return intersection;
    }
    double t = (y2-y1+(mY2*(x1-x2))/mX2)/(mY1-(mY2*mX1)/mX2);
    intersection[0] = x1+mX1*t;
    intersection[1] = y1+mY1*t;
    intersection[2] = sqrt(pow(intersection[0]-x1,2)+pow(intersection[1]-y1,2));
    return intersection;
}

double surface_fitting_test::linInterpolation(double x1, double x2, double y1, double y2, double x)
{
//    double result = y1+(x2-x)*(y2-y1)/(x2-x1);
//    return result;
    return y1+(x2-x)*(y2-y1)/(x2-x1);
}

double surface_fitting_test::bilinInterpolation(double q11, double q12, double q21, double q22, double x1, double x2, double y1, double y2, double x, double y)
{
    double x2x1, y2y1, x2x, y2y, yy1, xx1;
    x2x1 = x2 -x1;
    y2y1 = y2 -y1;
    x2x = x2 -x;
    y2y = y2 -y;
    yy1 = y - y1;
    xx1 = x - x1;
    return 1.0/(x2x1*y2y1)*(q11*x2x*y2y + q21*xx1*y2y + q12*x2x*yy1 + q22*xx1*yy1);
}

QVector<int> surface_fitting_test::getClosestNeighbours(double x, double y)
{
    QVector<int> closest = QVector<int>(4);
    closest[0] = int(x);
    closest[1] = closest[0]+1;
    closest[2] = int(y);
    closest[3] = closest[2]+1;
    return closest;
}


double surface_fitting_test::propagateRayThroughHisto(QImage histo, double entryX, double entryY, double angle, double length)
{
    if(length <=0.1){
        return 0;
    }
    double gray = 0.0;
    QVector<int> neighbours=QVector<int>(4); //kann man leicht erweitern, sodass die grauwerte einzeln gespeichert werden-> BScan
    for(int t=0; t<length;t++){
        neighbours = getClosestNeighbours(entryX+qSin(angle)*t,entryY+qCos(angle)*t);
        gray = gray + bilinInterpolation(getColor(histo,neighbours[0],neighbours[2]),getColor(histo,neighbours[0],neighbours[3]),getColor(histo,neighbours[1],neighbours[2]),getColor(histo,neighbours[1],neighbours[3]),neighbours[0],neighbours[1],neighbours[2],neighbours[3],entryX+qSin(angle)*t,entryY+qCos(angle)*t);
    }
    neighbours = getClosestNeighbours(entryX+qSin(angle)*length,entryY+qCos(angle)*length);
    gray = gray + (length-int(length))*bilinInterpolation(getColor(histo,neighbours[0],neighbours[2]),getColor(histo,neighbours[0],neighbours[3]),getColor(histo,neighbours[1],neighbours[2]),getColor(histo,neighbours[1],neighbours[3]),neighbours[0],neighbours[1],neighbours[2],neighbours[3],entryX+qSin(angle)*length,entryY+qCos(angle)*length);
    return gray;
}

QImage surface_fitting_test::propagateOctRayThroughHisto(QImage histo, QImage bScan, double entryX, double entryY, double angle, double length, double mediumRi, double sampleRi)
{
    if(length <=0){
        return bScan;
    }
    double colorDouble;
    int colorInt;
    double colorBuffer = 1;
    QColor gray;
    QVector<int> neighbours=QVector<int>(4);
    for(int t=0; t<length*sampleRi+1;t++){
        neighbours = getClosestNeighbours(entryX+qSin(angle)*(double(t)/sampleRi),entryY+qCos(angle)*(double(t)/sampleRi));
        colorDouble = bilinInterpolation(getColor(histo,neighbours[0],neighbours[2]),getColor(histo,neighbours[0],neighbours[3]),getColor(histo,neighbours[1],neighbours[2]),getColor(histo,neighbours[1],neighbours[3]),neighbours[0],neighbours[1],neighbours[2],neighbours[3],entryX+qSin(angle)*(double(t)/sampleRi),entryY+qCos(angle)*(double(t)/sampleRi));
        if(colorDouble<0){
            colorDouble=0;
        }
        colorInt=std::round((2/(sqrt(colorBuffer))-1)*colorDouble);
        if(colorInt<0){
            colorInt =0;
        }
        colorBuffer = colorBuffer+colorDouble/20000.0;
        gray=QColor(colorInt,colorInt,colorInt,255);
        bScan.setPixelColor(entryX,std::round(entryY*mediumRi+t),gray);
    }
    return bScan;
}

double surface_fitting_test::bilinInterpolFromSinogram(QImage sino, double x, double gamma, int startProj, int deltaProj)
{
    int x1 = int(x);
    int x2 = x1+1;
    double y = startProj + (double(numberOfProjections)/360.0)*qRadiansToDegrees(gamma);
    if(y>numberOfProjections-1){
        y=y-numberOfProjections+1;
    }else if(y<0){
        y=y+numberOfProjections-1;
    }
    int y1 = int(y);
    int y2 = y1 +1;
    if(gamma>0&&startProj+deltaProj!=y2){
        y2=startProj+deltaProj;
    }else if(gamma<0&&startProj+deltaProj!=y1){
        y1=startProj+deltaProj;
    }

    quint16 *dstSinoY1 = (quint16*)(sino.bits()+y1*sino.bytesPerLine());
    double q11 = dstSinoY1[x1];
    double q21 = dstSinoY1[x2];
    quint16 *dstSinoY2 = (quint16*)(sino.bits()+y2*sino.bytesPerLine());
    double q12 = dstSinoY2[x1];
    double q22 = dstSinoY2[x2];
    //double result = bilinInterpolation(q11,q12,q21,q22,x1,x2,y1,y2,x,y);
    //return result;
    return bilinInterpolation(q11,q12,q21,q22,x1,x2,y1,y2,x,y);
}

double surface_fitting_test::newBilinInterpolFromSinogram(QImage sino, double absX, double absProj)
{
    int x1 = int(absX);
    int x2 = x1+1;
    if(absProj>numberOfProjections-1){
        absProj=absProj-numberOfProjections+1;
    }else if(absProj<0){
        absProj=absProj+numberOfProjections-1;
    }
    int y1 = int(absProj);
    int y2 = y1 + 1;
    quint16 *dstSinoY1 = (quint16*)(sino.bits()+y1*sino.bytesPerLine());
    double q11 = dstSinoY1[x1];
    double q21 = dstSinoY1[x2];
    quint16 *dstSinoY2 = (quint16*)(sino.bits()+y2*sino.bytesPerLine());
    double q12 = dstSinoY2[x1];
    double q22 = dstSinoY2[x2];
    return bilinInterpolation(q11,q12,q21,q22,x1,x2,y1,y2,absX,absProj);
}

double surface_fitting_test::getTransmissionGrade(double riMedium, double riSample, entryPoint point)
{
    if(riMedium==riSample){
        return 1.0;
    }else if(point.yEntry==0){
        return 10000.0;
    }
    else if(point.slopeEntry==0){ //Wenn die Oberfläche horizontal ist, muss diese Formel verwendet werden
        return 1.0 - pow((riMedium-riSample)/(riMedium+riSample),2);
    }else{
        double alpha = qAtan(point.slopeEntry);
        double beta = alpha - point.gammaEntry;
        double tp = 1.0 - pow((qTan(alpha-beta)/qTan(alpha+beta)),2);
        double ts = 1.0 - pow((-qSin(alpha-beta)/qSin(alpha+beta)),2);
        if((tp+ts)/2.0<0.5){
            return 0.5;
        }
        return (tp+ts)/2.0;
    }
}

QVector<QImage> surface_fitting_test::makeRotatedImageStack(QString path, int stackSize)
{
    QElapsedTimer reTimer;
    reTimer.start();
    QVector<QImage> rotatedStack = QVector<QImage>(stackSize);
    qDebug()<<path;
    qDebug()<<"rotation Started"<<reTimer.elapsed();
    if(path.endsWith(".png")){
        path.remove(path.length()-4,4);
    }
    qDebug()<<path;
    QString inputCopyPath = path;
    inputCopyPath.append(".png");
    std::string str = inputCopyPath.toStdString();
    const char* p = str.c_str();
    qDebug()<<"Dings zu const char converted"<<reTimer.elapsed();
    af::array pic = af::loadImage(p,false);
    qDebug()<<"image loaded as array"<<reTimer.elapsed();
    for(int k = 0; k<stackSize;k++){
        af::array rot = af::rotate(pic,-rotateClockwise*float(k)/float(stackSize)*2*M_PI,true,AF_INTERP_BILINEAR);
        QString saveString = path;
        saveString.append("_buffer.png");
        std::string saveStr = saveString.toStdString();
        const char* q = saveStr.c_str();
        af::saveImage(q,rot);
        QImage saveImage;
        saveImage.load(saveString);
        rotatedStack[k]=saveImage;
    }

    qDebug()<<"Rotation mit AF fertig." <<reTimer.elapsed();
    return rotatedStack;
}

void surface_fitting_test::learningAF(int size)
{
    af::array a = af::randu(100, f32);  //This works
    float *host_a = a.host<float>();
    af::freeHost(host_a);


    QVector<int> ascan = fillVectorWithAscan(inputHisto,300);  //Starting with a QVector<int>
    std::vector<int> stdVec = ascan.toStdVector();              //Conversion to std::vector<int>
    int toArray[1000];
    std::copy(stdVec.begin(), stdVec.end(), toArray);           //"Conversion" to std::array
    af::array ascanAF=af::array(1,inputHisto.height(),toArray); //Creating af::array (this works!)
    af::array nonZero = af::where(ascanAF);
    af::array bufArrayAF = nonZero.as(f32);
    float *host_nonZero = bufArrayAF.host<float>();
    qDebug()<<host_nonZero[2];
    std::vector<float> jo = std::vector<float>(*host_nonZero);
    QVector<float> qVec = QVector<float>::fromStdVector(jo);
    qDebug()<<qVec;
    af::freeHost(host_nonZero);
}

QVector<surface_fitting_test::surfaceInfo> surface_fitting_test::makeStructVector(QVector<QImage> imageList, QVector<QVector<QVector<double> > > pointList)
{
    QVector<surfaceInfo> returnList(imageList.size());
    for(int k=0; k<imageList.size();k++){
        returnList[k].thinnedOut = imageList[k];
        returnList[k].entryPoints = pointList[k];
    }
    return returnList;
}

QVector<surface_fitting_test::newSurfaceInfo> surface_fitting_test::makeNewStructVector(QVector<QImage> imageList, QVector<QVector<entryPoint> > pointList)
{
    QVector<newSurfaceInfo> returnList(imageList.size());
    for(int k=0; k<imageList.size();k++){
        returnList[k].thinnedOut = imageList[k];
        returnList[k].ePoints = pointList[k];
    }
    return returnList;
}

int surface_fitting_test::propagateRayMultiThreaded(surfaceInfo &surfaceList)
{
    surfaceList.entryPoints[0]={0,0,0,0}; //Nullpointer :(
    for(int x = 1;x<surfaceList.thinnedOut.width();x++){
        surfaceList.entryPoints[x]={0,0};
           if(getFirstValueFromTopStatic(surfaceList.thinnedOut,x-1)!=0 && getFirstValueFromTopStatic(surfaceList.thinnedOut,x)!=0 && getFirstValueFromTopStatic(surfaceList.thinnedOut,x+1)!=0){ //dadurch werden die äußersten Punkte ignoriert
               QVector<double>bufVec = getSlopeAtEntryStatic(surfaceList.thinnedOut,x,slopeSigmaMT);
               double exitAngle = getExitAngleStatic(bufVec[1],riMediumMT,riSampleMT);
               surfaceList.entryPoints[x] = {bufVec[0],exitAngle};
               surfaceList.entryPoints[x].append(0);
               if(exitAngle>=0&&exitAngle<1.0){  //Strahl wird nach rechts abgelenkt
                   int X=x;
                   bool success = false;
                   while(success == false && getFirstValueFromBottomStatic(surfaceList.thinnedOut,X+1)>0){
                       QVector<double> surfaceVector = parameterizeFromPointsStatic(X, getFirstValueFromBottomStatic(surfaceList.thinnedOut,X), X+1, getFirstValueFromBottomStatic(surfaceList.thinnedOut,X+1));
                       QVector<double> rayVector = parameterizeFromAngleStatic(x,bufVec[0],exitAngle);
                       QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                       if(intersection[0]>=X&&intersection[0]<X+1){
                           surfaceList.entryPoints[x][2] = intersection[2];
                           success = true;
                       }else{
                           X++;
                       }
                   }
                   if(success == false){
                       //qDebug()<<"Strahl wurde nach rechts abgelenkt und fand keine Rückseite bei: "<<k<<x;
                       if(getFirstValueFromTopStatic(surfaceList.thinnedOut,X)!=getFirstValueFromBottomStatic(surfaceList.thinnedOut,X)){
                           QVector<double> surfaceVector = parameterizeFromPointsStatic(X, getFirstValueFromTopStatic(surfaceList.thinnedOut,X), X, getFirstValueFromBottomStatic(surfaceList.thinnedOut,X));
                           QVector<double> rayVector = parameterizeFromAngleStatic(x,bufVec[0],exitAngle);
                           QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                           if(intersection[1]>=getFirstValueFromTopStatic(surfaceList.thinnedOut,X)&&intersection[1]<=getFirstValueFromBottomStatic(surfaceList.thinnedOut,X)){
                               //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                               surfaceList.entryPoints[x][2] = intersection[2];
                               success = true;
                           }
                       }
                       while(success == false && X>=x){
                           QVector<double> surfaceVector = parameterizeFromPointsStatic(X-1, getFirstValueFromTopStatic(surfaceList.thinnedOut,X-1), X, getFirstValueFromTopStatic(surfaceList.thinnedOut,X));
                           QVector<double> rayVector = parameterizeFromAngleStatic(x,bufVec[0],exitAngle);
                           QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                           if(intersection[0]>=X&&intersection[0]<X+1){
                               surfaceList.entryPoints[x][2] = intersection[2];
                               //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                               success = true;
                           }else{
                               X--;
                           }
                       }
                   }
               }else if(exitAngle<0&&exitAngle>-1.0){  //Strahl wird nach links abgelenkt
                   int X=x;
                   bool success = false;
                   while(success == false && getFirstValueFromBottomStatic(surfaceList.thinnedOut,X-1) > 0){
                       QVector<double> surfaceVector = parameterizeFromPointsStatic(X, getFirstValueFromBottomStatic(surfaceList.thinnedOut,X), X-1, getFirstValueFromBottomStatic(surfaceList.thinnedOut,X-1));
                       QVector<double> rayVector = parameterizeFromAngleStatic(x,bufVec[0],exitAngle);
                       QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                       if(intersection[0]<=X&&intersection[0]>X-1){
                           surfaceList.entryPoints[x][2] = intersection[2];
                           success = true;
                       }else{
                           X--;
                       }
                   }
                   if(success == false){
                       //qDebug()<<"Strahl wurde nach links abgelenkt und fand keine Rückseite bei: "<<k<<x;
                       if(getFirstValueFromTopStatic(surfaceList.thinnedOut,X)!=getFirstValueFromBottomStatic(surfaceList.thinnedOut,X)){
                           QVector<double> surfaceVector = parameterizeFromPointsStatic(X, getFirstValueFromTopStatic(surfaceList.thinnedOut,X), X, getFirstValueFromBottomStatic(surfaceList.thinnedOut,X));
                           QVector<double> rayVector = parameterizeFromAngleStatic(x,bufVec[0],exitAngle);
                           QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                           if(intersection[1]>=getFirstValueFromTopStatic(surfaceList.thinnedOut,X)&&intersection[1]<=getFirstValueFromBottomStatic(surfaceList.thinnedOut,X)){
                               //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                               surfaceList.entryPoints[x][2] = intersection[2];
                               success = true;
                           }
                       }
                       while(success == false && X<=x){
                           QVector<double> surfaceVector = parameterizeFromPointsStatic(X, getFirstValueFromTopStatic(surfaceList.thinnedOut,X), X+1, getFirstValueFromTopStatic(surfaceList.thinnedOut,X+1));
                           QVector<double> rayVector = parameterizeFromAngleStatic(x,bufVec[0],exitAngle);
                           QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                           if(intersection[0]>=X&&intersection[0]<X+1){
                               surfaceList.entryPoints[x][2] = intersection[2];
                               //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                               success = true;
                           }else{
                               X++;
                           }
                       }
                   }
               }else{
                   //qDebug()<<"Strahl wurde total reflektiert!"<<k<<x;
               }

               surfaceList.entryPoints[x].append(bufVec[1]);
               // Wert in ein Sinogram malen
               if(surfaceList.entryPoints[x][2]<0){
                   surfaceList.entryPoints[x][2]=0;
                   qDebug()<<"irgendwas schiefgelaufen beim rayPropagate";
               }

           }else{
               surfaceList.entryPoints[x] = {0,0,0,0};
           }
       }
    return 1;
}

int surface_fitting_test::newPropagateRayMultiThreaded(newSurfaceInfo &surfaceList)
{
    surfaceList.ePoints[0]={0,0,0,0,0}; //Nullpointer :(
    for(int x = 1;x<surfaceList.thinnedOut.width();x++){
        //surfaceList.entryPoints[x]={0,0};
           if(getFirstValueFromTopStatic(surfaceList.thinnedOut,x-1)!=0 && getFirstValueFromTopStatic(surfaceList.thinnedOut,x)!=0 && getFirstValueFromTopStatic(surfaceList.thinnedOut,x+1)!=0){ //dadurch werden die äußersten Punkte ignoriert
               QVector<double>bufVec = getSlopeAtEntryStatic(surfaceList.thinnedOut,x,slopeSigmaMT);
               double exitAngle = getExitAngleStatic(bufVec[1],riMediumMT,riSampleMT);
               surfaceList.ePoints[x].gammaEntry = exitAngle;
               surfaceList.ePoints[x].yEntry = bufVec[0];
               surfaceList.ePoints[x].slopeEntry = bufVec[1];
               surfaceList.ePoints[x].lengthEntry = 0;
               surfaceList.ePoints[x].xEntry = x;
               if(exitAngle>=0&&exitAngle<1.0){  //Strahl wird nach rechts abgelenkt
                   int X=x;
                   bool success = false;
                   while(success == false && getFirstValueFromBottomStatic(surfaceList.thinnedOut,X+1)>0){
                       QVector<double> surfaceVector = parameterizeFromPointsStatic(X, getFirstValueFromBottomStatic(surfaceList.thinnedOut,X), X+1, getFirstValueFromBottomStatic(surfaceList.thinnedOut,X+1));
                       QVector<double> rayVector = parameterizeFromAngleStatic(x,bufVec[0],exitAngle);
                       QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                       if(intersection[0]>=X&&intersection[0]<X+1){
                           surfaceList.ePoints[x].lengthEntry = intersection[2];
                           success = true;
                       }else{
                           X++;
                       }
                   }
                   if(success == false){
                       //qDebug()<<"Strahl wurde nach rechts abgelenkt und fand keine Rückseite bei: "<<k<<x;
                       if(getFirstValueFromTopStatic(surfaceList.thinnedOut,X)!=getFirstValueFromBottomStatic(surfaceList.thinnedOut,X)){
                           QVector<double> surfaceVector = parameterizeFromPointsStatic(X, getFirstValueFromTopStatic(surfaceList.thinnedOut,X), X, getFirstValueFromBottomStatic(surfaceList.thinnedOut,X));
                           QVector<double> rayVector = parameterizeFromAngleStatic(x,bufVec[0],exitAngle);
                           QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                           if(intersection[1]>=getFirstValueFromTopStatic(surfaceList.thinnedOut,X)&&intersection[1]<=getFirstValueFromBottomStatic(surfaceList.thinnedOut,X)){
                               //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                               surfaceList.ePoints[x].lengthEntry = intersection[2];
                               success = true;
                           }
                       }
                       while(success == false && X>=x){
                           QVector<double> surfaceVector = parameterizeFromPointsStatic(X-1, getFirstValueFromTopStatic(surfaceList.thinnedOut,X-1), X, getFirstValueFromTopStatic(surfaceList.thinnedOut,X));
                           QVector<double> rayVector = parameterizeFromAngleStatic(x,bufVec[0],exitAngle);
                           QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                           if(intersection[0]>=X&&intersection[0]<X+1){
                               surfaceList.ePoints[x].lengthEntry = intersection[2];
                               //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                               success = true;
                           }else{
                               X--;
                           }
                       }
                   }
               }else if(exitAngle<0&&exitAngle>-1.0){  //Strahl wird nach links abgelenkt
                   int X=x;
                   bool success = false;
                   while(success == false && getFirstValueFromBottomStatic(surfaceList.thinnedOut,X-1) > 0){
                       QVector<double> surfaceVector = parameterizeFromPointsStatic(X, getFirstValueFromBottomStatic(surfaceList.thinnedOut,X), X-1, getFirstValueFromBottomStatic(surfaceList.thinnedOut,X-1));
                       QVector<double> rayVector = parameterizeFromAngleStatic(x,bufVec[0],exitAngle);
                       QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                       if(intersection[0]<=X&&intersection[0]>X-1){
                           surfaceList.ePoints[x].lengthEntry = intersection[2];
                           success = true;
                       }else{
                           X--;
                       }
                   }
                   if(success == false){
                       //qDebug()<<"Strahl wurde nach links abgelenkt und fand keine Rückseite bei: "<<k<<x;
                       if(getFirstValueFromTopStatic(surfaceList.thinnedOut,X)!=getFirstValueFromBottomStatic(surfaceList.thinnedOut,X)){
                           QVector<double> surfaceVector = parameterizeFromPointsStatic(X, getFirstValueFromTopStatic(surfaceList.thinnedOut,X), X, getFirstValueFromBottomStatic(surfaceList.thinnedOut,X));
                           QVector<double> rayVector = parameterizeFromAngleStatic(x,bufVec[0],exitAngle);
                           QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                           if(intersection[1]>=getFirstValueFromTopStatic(surfaceList.thinnedOut,X)&&intersection[1]<=getFirstValueFromBottomStatic(surfaceList.thinnedOut,X)){
                               //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                               surfaceList.ePoints[x].lengthEntry = intersection[2];
                               success = true;
                           }
                       }
                       while(success == false && X<=x){
                           QVector<double> surfaceVector = parameterizeFromPointsStatic(X, getFirstValueFromTopStatic(surfaceList.thinnedOut,X), X+1, getFirstValueFromTopStatic(surfaceList.thinnedOut,X+1));
                           QVector<double> rayVector = parameterizeFromAngleStatic(x,bufVec[0],exitAngle);
                           QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                           if(intersection[0]>=X&&intersection[0]<X+1){
                               surfaceList.ePoints[x].lengthEntry = intersection[2];
                               //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                               success = true;
                           }else{
                               X++;
                           }
                       }
                   }
               }else{
                   //qDebug()<<"Strahl wurde total reflektiert!"<<k<<x;
               }

               if(surfaceList.ePoints[x].lengthEntry<0){
                   surfaceList.ePoints[x].lengthEntry=0;
                   qDebug()<<"irgendwas schiefgelaufen beim rayPropagate";
               }

           }else{
               surfaceList.ePoints[x] = {0,0,0,0,0};
               surfaceList.ePoints[x].xEntry = x;
           }
       }
    return 1;
}

int surface_fitting_test::getFirstValueFromTopStatic(QImage image, int X)
{
    for(int y = 0;y<image.height();y++){
        if(getColorStatic(image,X,y)!=0){
            return y;
        }
    }
    return 0;
}

int surface_fitting_test::getFirstValueFromBottomStatic(QImage image, int X)
{
    for(int y = image.height()-1;y>=0;y--){
        if(getColorStatic(image,X,y)!=0){
            return y;
        }
    }
    return 0;
}

int surface_fitting_test::getColorStatic(QImage image, int x, int y)
{
    QColor color = image.pixelColor(x,y);
    return color.green();
}

QVector<double> surface_fitting_test::getSlopeAtEntryStatic(QImage image, int X, int sigma)
{
    if(getFirstValueFromTopStatic(image,X)==0){
        //qDebug()<<"Keine Oberfläche im AScan gefunden";
        QVector<double> d{999,999};
        return d; //keine Oberfläche im Ascan
    }
    double Sxx = 0;
    double Sxy = 0;
    double yQuer = 0;
    double xQuer = 0;
    double b = 0;
    int sumX = 0;
    int fracX = 0;
    for(int x = X-sigma;x<=X+sigma;x++){
        if(getFirstValueFromTopStatic(image,x)!=0){
            sumX = sumX + x;
            fracX = fracX + 1;
        }
    }
    xQuer = sumX/fracX;
    int sumY = 0;
    int fracY = 0;
    for(int x = X-sigma;x<=X+sigma;x++){
        if(getFirstValueFromTopStatic(image,x)!=0){
            sumY = sumY + getFirstValueFromTopStatic(image,x);
            fracY=fracY+1;
            Sxx = Sxx + (x-xQuer)*(x-xQuer);
        }
    }
    yQuer = sumY/fracY;
    for(int x = X-sigma;x<=X+sigma;x++){
        if(getFirstValueFromTopStatic(image,x)!=0){
            Sxy = Sxy + (x-xQuer)*(getFirstValueFromTopStatic(image,x)-yQuer);
        }
    }
    b = yQuer - (Sxy/Sxx)*xQuer;
    double yReturn = b + (Sxy/Sxx)*X;
    return {yReturn,Sxy/Sxx};
}

double surface_fitting_test::getExitAngleStatic(double slope, double n1, double n2)
{
    if(n1==n2||slope==0){
        return 0.0;
    }else if((n1*qSin(qAtan(slope)))/n2<-1){
        return -1.0;
    }else if((n1*qSin(qAtan(slope)))/n2>1){
        return 1.0;
    }else{
        return -(qAtan(slope) - qAsin((n1*qSin(qAtan(slope)))/n2));
    }
}

QVector<double> surface_fitting_test::parameterizeFromPointsStatic(double x1, double y1, double x2, double y2)
{
    QVector<double> fuPa = QVector<double>(4);
    if(x1==x2){
        fuPa[0] = x1;
        fuPa[1] = y1;
        fuPa[2] = 0;
        fuPa[3] = 1;
        return fuPa;
    }
    fuPa[0] = x1;
    fuPa[1] = y1;
    fuPa[2] = qCos(qAtan((y2-y1)/(x2-x1)));
    fuPa[3] = qSin(qAtan((y2-y1)/(x2-x1)));
    return fuPa;
}

QVector<double> surface_fitting_test::parameterizeFromAngleStatic(double x, double y, double angle)
{
    QVector<double> fuPa = QVector<double>(4);
    fuPa[0] = x;
    fuPa[1] = y;
    fuPa[2] = qSin(angle);
    fuPa[3] = qCos(angle);
    return fuPa;
}

QVector<double> surface_fitting_test::getIntersectionPointStatic(QVector<double> ray, QVector<double> surface)
{
    QVector<double> intersection = QVector<double>{0,0,0};
    double x1 = ray[0];
    double x2 = surface[0];
    double y1 = ray[1];
    double y2 = surface[1];
    double mX1 = ray[2];
    double mX2 = surface[2];
    double mY1 = ray[3];
    double mY2 = surface[3];
    if(mX2==0){
        double t = (y1-y2+(mY1*(x2-x1))/mX1)/(mY2-(mY1*mX2)/mX1);
        intersection[0] = x2+mX2*t;
        intersection[1] = y2+mY2*t;
        intersection[2] = sqrt(pow(intersection[0]-x1,2)+pow(intersection[1]-y1,2));
        return intersection;
    }
    double t = (y2-y1+(mY2*(x1-x2))/mX2)/(mY1-(mY2*mX1)/mX2);
    intersection[0] = x1+mX1*t;
    intersection[1] = y1+mY1*t;
    intersection[2] = sqrt(pow(intersection[0]-x1,2)+pow(intersection[1]-y1,2));
    return intersection;
}

double surface_fitting_test::calculateCameraPoint(double mediumRI, double yExit, double xExit, double angleExit)
{
    //hier muss noch irgendwie ein Faktor rein um von Pixeln auf mm zu kommen
    if(angleExit == -1){
        return -99;
    }else if(angleExit ==1){
        return 99;
    }
    double radiusGlassCuvette = 9.0;
    double n2 = 1.523;
    double n3 = 1;
    double y1 = radiusGlassCuvette-yExit;
    double y2 = 2.0;
    double y3 = 73.0;
    double x1 = y1*qTan(angleExit);
    //qDebug()<<"Versatz auf Innenseite: "<<x1;
    double x2 = y2*qTan(qAsin(qSin(angleExit)*mediumRI/n2));
    //qDebug()<<"Versatz innerhalb Glas: "<<x2;
    double x3 = y3*qTan(qAsin(qSin(angleExit)*mediumRI/n3));
    //qDebug()<<"Winkel hinter GlassCuvete (total): "<<qRadiansToDegrees(qAsin(qSin(angleExit)*mediumRI/n3));
    //qDebug()<<"Versatz in X: "<<x1+x2+x3;
    return xExit + x1 + x2 + x3;
}

QVector<double> surface_fitting_test::getBackExitPointAndAngle(QImage thinSurface, int xEntry, double mediaRI, double samplRi)
{
    struct entryPoint myPoint;
    myPoint.xEntry = xEntry;
    if(getFirstValueFromTop(thinSurface,xEntry-1)!=0 && getFirstValueFromTop(thinSurface,xEntry)!=0 && getFirstValueFromTop(thinSurface,xEntry+1)!=0){ //dadurch werden die äußersten Punkte ignoriert
        QVector<double>bufVec = getSlopeAtEntry(thinSurface,xEntry,ui->spinBox_slopeSigma->value());
        double exitAngle = getExitAngle(bufVec[1],mediaRI,samplRi);
//        qDebug()<<"Entry Slope ist: "<<bufVec[1];
//        qDebug()<<"Exit Angle in Probe: "<<exitAngle;
        myPoint.gammaEntry = exitAngle;
        myPoint.yEntry = bufVec[0];
        if(exitAngle>=0&&exitAngle<1.0){  //Strahl wird nach rechts abgelenkt
            int X=xEntry;
            bool success = false;
            while(success == false && getFirstValueFromBottom(thinSurface,X+1)>0){
                QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromBottom(thinSurface,X), X+1, getFirstValueFromBottom(thinSurface,X+1));
                QVector<double> rayVector = parameterizeFromAngle(xEntry,bufVec[0],exitAngle);
                QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                if(intersection[0]>=X&&intersection[0]<X+1){
                    myPoint.lengthEntry = intersection[2];
                    success = true;
                }else{
                    X++;
                }
            }
            if(success == false){
                //qDebug()<<"Strahl wurde nach rechts abgelenkt und fand keine Rückseite bei: "<<q<<x;
                if(getFirstValueFromTop(thinSurface,X)!=getFirstValueFromBottom(thinSurface,X)){
                    QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromTop(thinSurface,X), X, getFirstValueFromBottom(thinSurface,X));
                    QVector<double> rayVector = parameterizeFromAngle(xEntry,bufVec[0],exitAngle);
                    QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                    if(intersection[1]>=getFirstValueFromTop(thinSurface,X)&&intersection[1]<=getFirstValueFromBottom(thinSurface,X)){
                        //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                        myPoint.lengthEntry = intersection[2];
                        success = true;
                    }
                }
                while(success == false && X>=xEntry){
                    QVector<double> surfaceVector = parameterizeFromPoints(X-1, getFirstValueFromTop(thinSurface,X-1), X, getFirstValueFromTop(thinSurface,X));
                    QVector<double> rayVector = parameterizeFromAngle(xEntry,bufVec[0],exitAngle);
                    QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                    if(intersection[0]>=X&&intersection[0]<X+1){
                        myPoint.lengthEntry = intersection[2];
                        //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                        success = true;
                    }else{
                        X--;
                    }
                }
            }
        }else if(exitAngle<0&&exitAngle>-1.0){  //Strahl wird nach links abgelenkt
            int X=xEntry;
            bool success = false;
            while(success == false && getFirstValueFromBottom(thinSurface,X-1) > 0){
                QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromBottom(thinSurface,X), X-1, getFirstValueFromBottom(thinSurface,X-1));
                QVector<double> rayVector = parameterizeFromAngle(xEntry,bufVec[0],exitAngle);
                QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                if(intersection[0]<=X&&intersection[0]>X-1){
                    myPoint.lengthEntry = intersection[2];
                    success = true;
                }else{
                    X--;
                }
            }
            if(success == false){
                //qDebug()<<"Strahl wurde nach links abgelenkt und fand keine Rückseite bei: "<<q<<x;
                if(getFirstValueFromTop(thinSurface,X)!=getFirstValueFromBottom(thinSurface,X)){
                    QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromTop(thinSurface,X), X, getFirstValueFromBottom(thinSurface,X));
                    QVector<double> rayVector = parameterizeFromAngle(xEntry,bufVec[0],exitAngle);
                    QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                    if(intersection[1]>=getFirstValueFromTop(thinSurface,X)&&intersection[1]<=getFirstValueFromBottom(thinSurface,X)){
                        //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                        myPoint.lengthEntry = intersection[2];
                        success = true;
                    }
                }
                while(success == false && X<=xEntry){
                    QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromTop(thinSurface,X), X+1, getFirstValueFromTop(thinSurface,X+1));
                    QVector<double> rayVector = parameterizeFromAngle(xEntry,bufVec[0],exitAngle);
                    QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                    if(intersection[0]>=X&&intersection[0]<X+1){
                        myPoint.lengthEntry = intersection[2];
                        //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                        success = true;
                    }else{
                        X++;
                    }
                }
            }
        }else if(exitAngle==1){
            //qDebug()<<"Strahl wurde totalreflektiert!";
            return {double(xEntry),0,1};
        }else{
            //qDebug()<<"Strahl wurde totalreflektiert!";
            return {double(xEntry),0,-1};
        }

//    qDebug()<<"Strahl an Eingang "<<xEntry<<" simuliert.";
//    qDebug()<<"Ablenkung am Eingang betrug "<<entryPoint[1];
    double exitPositionX = xEntry + myPoint.lengthEntry*qSin(myPoint.gammaEntry);
    //qDebug()<<"Exit Position X betrug: "<<exitPositionX;
    QVector<double> exitSlope = getSlopeAtExit(thinSurface,std::round(exitPositionX),ui->spinBox_slopeSigma->value());
    //qDebug()<<"Exit Slope ist: "<<exitSlope[1];
    double backExitAngle = getExitAngleAtBack(exitSlope[1],samplRi,mediaRI,myPoint.gammaEntry);
    //qDebug()<<"Winkel hinter der Oberfläche: "<<backExitAngle;
    qDebug()<<"test, ob mein struct funzt: neu (ablenkung): "<<myPoint.gammaEntry;
    return {exitPositionX, exitSlope[0], backExitAngle};
}else{
        return {double(xEntry),0,0};
    }
}
QVector<double> surface_fitting_test::generateRefractionPattern(QImage thinSurface, double mediaRI, double sampleRI)
{
    QVector<double> pattern = QVector<double>(thinSurface.width());
    for(int x = 0; x<thinSurface.width(); x++){
        QVector<double> exit = getBackExitPointAndAngle(thinSurface,x,mediaRI,sampleRI);
        pattern[x]=calculateCameraPoint(mediaRI,mmPerPixelInReco*exit[1],mmPerPixelInReco*exit[0],exit[2]);
        if(exit[2]==-1||exit[2]==1){
            qDebug()<<"Strahl wurde totalreflektiert beim pattern-gen";
        }
    }
    qDebug()<<pattern;
    return pattern;
}

void surface_fitting_test::determineTeleError(QVector<QVector<double> > moments)
{
    int scanPoints = moments.size();
    qDebug()<<scanPoints;
    //Ablenkung in Probe und Ablenkung auf Sensor vergleichen
    double relativeScanMovementInSample = scanPoints*mmPerPixelInReco;
    qDebug()<<"Der Laser wurde in der Probe um "<<relativeScanMovementInSample<<" in x-Richtung bewegt";
    double relativeScanMovementOnSensor = (moments[scanPoints-1][0] - moments[0][0])*mmPerPixelOnSensor;
    qDebug()<<"Der Laser wurde in der auf dem Sensor um "<<relativeScanMovementOnSensor<<" in x-Richtung bewegt";
    correctedSensorRatio = relativeScanMovementInSample/std::abs(moments[scanPoints-1][0] - moments[0][0]);
    qDebug()<<"Echte Ratio aufm Sensor: "<<mmPerPixelOnSensor;
    qDebug()<<"Korrigierte Ratio aufm Sensor: "<<correctedSensorRatio;
}

QVector<QVector<double> > surface_fitting_test::makeListRelativeAndScaled(QVector<QVector<double> > moments)
{
    double offset = moments[0][0];
    for(int x = 0; x<moments.size();x++){
        moments[x][0] = (moments[x][0] - offset)*correctedSensorRatio;
        qDebug()<<"Skalierter und relativierter Versatz für Punkt "<<x<<"ist: "<<moments[x][0];
    }
    return moments;
}

double surface_fitting_test::getFittingSampleRI(QImage thinSurface, int xEntry, double xCameraPoint, double mediaRI, double expectedSampleRI, double riRange, double riIncriment, double exceptableOffset)
{
    //double mmPerPixel = 0.01;
    double currTestRi = expectedSampleRI - riRange;
    double smallestDeviation = 100;
    double fittingRi = 333;
    if(getFirstValueFromTop(thinSurface,xEntry)==0){
        return 999;
    }
    QVector<double> entrySlopeVec = getSlopeAtEntry(thinSurface,xEntry,ui->spinBox_slopeSigma->value());
    if(entrySlopeVec[1]==0){
        QVector<double> exitSlopeVec = getSlopeAtExit(thinSurface,xEntry,ui->spinBox_slopeSigma->value());
        if(exitSlopeVec[1]==0){
            return 777;
        }
    }
    while(currTestRi <=expectedSampleRI+riRange){
        QVector<double> exit = getBackExitPointAndAngle(thinSurface,xEntry,mediaRI,currTestRi);
        //qDebug()<<"Back Exit Point: " << xEntry<<currTestRi <<exit;
        if(exit[2]>-1&&exit[2]<1){
            double cameraPoint =calculateCameraPoint(mediaRI,mmPerPixelInReco*exit[1],mmPerPixelInReco*exit[0],exit[2]);
            if(std::abs(cameraPoint-xCameraPoint)<smallestDeviation){
                fittingRi = currTestRi;
                smallestDeviation = std::abs(cameraPoint-xCameraPoint);
                //qDebug()<<"Aktuell kleinste Abweichung ist: "<<std::abs(cameraPoint-xCameraPoint);
            }
        }
        currTestRi = currTestRi + riIncriment;
    }
    if(smallestDeviation<exceptableOffset){
        //qDebug()<<"Smallest Deviation von "<<xEntry<<smallestDeviation;
        return fittingRi;
    }
    return 333;
}


void surface_fitting_test::newNumber(QString name, int number, QString threadID)
{
    qDebug()<<"From dialog: " <<name << number << threadID;
    ui->lineEdit_threadedOutput->setText(name + " " + QString::number(number));
}

void surface_fitting_test::fillInThinnedSurface(QImage surface, int i)
{
    rotatedSurfacesThinnedOut[i]=surface;
    qDebug()<<"Thinned out surface in Liste eingetragen!!"<<i;
}

void surface_fitting_test::getMomentsList(QVector<QVector<double> > moments)
{
    momentsList = moments;
    qDebug()<<"Liste der Momente erhalten. Erster Wert: "<<momentsList[0];
}


void surface_fitting_test::on_pushButton_getSlopeAt_clicked()
{
    QVector<double> slope{0,0};
    slope = getSlopeAtEntry(bufferImg,ui->spinBox_aScan->value(),ui->spinBox_slopeSigma->value());
    if(slope[1]!=999){
        qDebug()<<"slope at"<<ui->spinBox_aScan->value()<<"is: "<< slope;
        drawAndDisplaySlope(bufferImg,ui->spinBox_aScan->value(),slope[0],slope[1],20);
    }
}


void surface_fitting_test::on_spinBox_aScan_valueChanged(int arg1)
{
    QVector<double> slope{0,0};
    slope = getSlopeAtEntry(bufferImg,arg1,ui->spinBox_slopeSigma->value());
    if(slope[1]!=999){
        qDebug()<<"slope at"<<ui->spinBox_aScan->value()<<"is: "<< slope;
        drawAndDisplaySlope(bufferImg,arg1,slope[0],slope[1],20);
        //qDebug()<<getDeltaThetaForPartner(slope[1],riMedium,riSample);
    }
}

void surface_fitting_test::on_spinBox_slopeSigma_valueChanged(int arg1)
{    
    QVector<double> slope{0,0};
    slope = getSlopeAtEntry(bufferImg,ui->spinBox_aScan->value(),arg1);
    if(slope[1]!=999){
        qDebug()<<"slope at"<<ui->spinBox_aScan->value()<<"is: "<< slope;
        drawAndDisplaySlope(bufferImg,ui->spinBox_aScan->value(),slope[0],slope[1],20);
    }
}


void surface_fitting_test::on_pushButton_createSinogram_clicked()
{
    nameRI = QString::number(riMedium,'g',4);
    nameRI.append("_");
    nameRI.append(QString::number(riSample,'g',4));
    nameRI.append("_");
    double higherRI = riMedium;
    if(higherRI<riSample){
        higherRI = riSample;
    }    riMediumMT = riMedium;
    riSampleMT = riSample;
    QElapsedTimer externalCorrectionTimer;
    externalCorrectionTimer.start();
    slopeSigmaMT = ui->spinBox_slopeSigma->value();
    QVector<surfaceInfo> surfaceList;
    QVector<newSurfaceInfo> newSurfaceList;
    bool createBScans = ui->checkBox_createBScans->isChecked();
    createTransmission = ui->checkBox_createTransmission->isChecked();
    sinogramHisto = QImage(inputSurface.width(),numberOfProjections+2,QImage::Format_Grayscale16);
    sinogramHisto.fill(0);
    transmissionHisto = sinogramHisto;
    rotatedEntryPoints = QVector<QVector<QVector<double>>>(numberOfProjections);
    newRotatedEntryPoints = QVector<QVector<entryPoint>>(numberOfProjections);
    if(useMultiThreading){
        for(int k = 0;k<numberOfProjections;k++){
            rotatedEntryPoints[k]=QVector<QVector<double>>(rotatedSurfacesThinnedOut[0].width());
            newRotatedEntryPoints[k] = QVector<entryPoint>(rotatedSurfacesThinnedOut[0].width());
        }
        surfaceList = makeStructVector(rotatedSurfacesThinnedOut,rotatedEntryPoints);
        newSurfaceList = makeNewStructVector(rotatedSurfacesThinnedOut,newRotatedEntryPoints);
        QtConcurrent::blockingMap(surfaceList,&surface_fitting_test::propagateRayMultiThreaded);
        QtConcurrent::blockingMap(newSurfaceList,&surface_fitting_test::newPropagateRayMultiThreaded);
        qDebug()<<"Strahlen wurden MT propagiert"<<externalCorrectionTimer.restart();
        for(int k=0; k<surfaceList.size();k++){
            rotatedEntryPoints[k] = surfaceList[k].entryPoints;
        }
        for(int k=0; k<newSurfaceList.size();k++){
            newRotatedEntryPoints[k] = newSurfaceList[k].ePoints;
        }
        for(int q = 0;q<numberOfProjections;q++){
            if(createBScans){
                bScan = QImage(inputSurface.width(),inputSurface.height()*higherRI,QImage::Format_Grayscale8);
                bScan.fill(0);
            }
            quint16 *dstHisto = (quint16*)(sinogramHisto.bits()+q*sinogramHisto.bytesPerLine());
            quint16 *dstTrans = (quint16*)(transmissionHisto.bits()+q*transmissionHisto.bytesPerLine());
            if(createTransmission){
                dstTrans[0]= int(65535);
            }
            for(int x = 1;x<inputSurface.width();x++){
                if(createBScans){
                    bScan = propagateOctRayThroughHisto(rotatedHistoImages[q],bScan,double(x),newRotatedEntryPoints[q][x].yEntry,newRotatedEntryPoints[q][x].gammaEntry,newRotatedEntryPoints[q][x].lengthEntry,riMedium,riSample);
                }

                double gray = 0.3*propagateRayThroughHisto(rotatedHistoImages[q],x,newRotatedEntryPoints[q][x].yEntry,newRotatedEntryPoints[q][x].gammaEntry,newRotatedEntryPoints[q][x].lengthEntry);
                if(gray>65535){
                    qDebug()<<"Aufaddierter Grauwert zu groß für 16bit: "<<q<<x<<gray;
                    gray = 65535;
                }
                dstHisto[x]= int(gray);
                if(createTransmission){
                    if(newRotatedEntryPoints[q][x].gammaEntry==1||newRotatedEntryPoints[q][x].gammaEntry==-1){
                        dstTrans[x]= int(0);
                    }else{
                        int buf = (65535 - 0.5*gray)*qCos(newRotatedEntryPoints[q][x].gammaEntry*10.0);
                        if (buf<0){
                            buf=0;
                        }
                        dstTrans[x]= buf;
                    }
                }
            }
            double rotateBy = (2*M_PI/double(numberOfProjections))*q;
            QString projAngle = QString::number(qRadiansToDegrees(rotateBy),'g',4);
            if(createBScans){
                bScan = noiseBScan(bScan,1);
                bScan.save(inputPathSurface+"\\"+nameRI+"_bScan_"+projAngle+".png");
                bScan.fill(0);
            }
        }
    }else{
        for(int q = 0;q<numberOfProjections;q++){
            qDebug()<<"new Projection"<<q;
            if(createBScans){
                bScan = QImage(inputSurface.width(),inputSurface.height()*higherRI,QImage::Format_Grayscale8);
                bScan.fill(0);
            }
            quint16 *dstTrans = (quint16*)(transmissionHisto.bits()+q*transmissionHisto.bytesPerLine());
            if(createTransmission){
                dstTrans[0]= int(65535);
            }
            quint16 *dstHisto = (quint16*)(sinogramHisto.bits()+q*sinogramHisto.bytesPerLine());
            rotatedEntryPoints[q] = QVector<QVector<double>>(inputSurface.width());
            newRotatedEntryPoints[q] = QVector<entryPoint>(inputSurface.width());
            newRotatedEntryPoints[q][0].xEntry = 0;
            rotatedEntryPoints[q][0]={0,0,0};
            for(int x = 1;x<inputSurface.width();x++){
                rotatedEntryPoints[q][x]={0,0};
                newRotatedEntryPoints[q][x].xEntry = x;
                if(createTransmission){
                    if(getFirstValueFromTop(rotatedSurfacesThinnedOut[q],x)==0){
                        dstTrans[x]= int(65535);
                    }
                }
                if(getFirstValueFromTop(rotatedSurfacesThinnedOut[q],x-1)!=0 && getFirstValueFromTop(rotatedSurfacesThinnedOut[q],x)!=0 && getFirstValueFromTop(rotatedSurfacesThinnedOut[q],x+1)!=0){ //dadurch werden die äußersten Punkte ignoriert
                    QVector<double>bufVec = getSlopeAtEntry(rotatedSurfacesThinnedOut[q],x,ui->spinBox_slopeSigma->value());
                    double exitAngle = getExitAngle(bufVec[1],riMedium,riSample);
                    rotatedEntryPoints[q][x] = {bufVec[0],exitAngle};
                    rotatedEntryPoints[q][x].append(0);
                    newRotatedEntryPoints[q][x].yEntry = bufVec[0];
                    newRotatedEntryPoints[q][x].slopeEntry = bufVec[1];
                    newRotatedEntryPoints[q][x].gammaEntry = exitAngle;
                    if(exitAngle>=0&&exitAngle<1.0){  //Strahl wird nach rechts abgelenkt
                        int X=x;
                        bool success = false;
                        while(success == false && getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X+1)>0){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X), X+1, getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X+1));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[0]>=X&&intersection[0]<X+1){
                                rotatedEntryPoints[q][x][2] = intersection[2];
                                newRotatedEntryPoints[q][x].lengthEntry = intersection[2];
                                success = true;
                            }else{
                                X++;
                            }
                        }
                        if(success == false){
                            //qDebug()<<"Strahl wurde nach rechts abgelenkt und fand keine Rückseite bei: "<<q<<x;
                            if(getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X)!=getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X)){
                                QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X), X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X));
                                QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                                QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                                if(intersection[1]>=getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X)&&intersection[1]<=getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X)){
                                    //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                                    rotatedEntryPoints[q][x][2] = intersection[2];
                                    newRotatedEntryPoints[q][x].lengthEntry = intersection[2];
                                    success = true;
                                }
                            }
                            while(success == false && X>=x){
                                QVector<double> surfaceVector = parameterizeFromPoints(X-1, getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X-1), X, getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X));
                                QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                                QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                                if(intersection[0]>=X&&intersection[0]<X+1){
                                    rotatedEntryPoints[q][x][2] = intersection[2];
                                    newRotatedEntryPoints[q][x].lengthEntry = intersection[2];
                                    //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                                    success = true;
                                }else{
                                    X--;
                                }
                            }
                        }
                    }else if(exitAngle<0&&exitAngle>-1.0){  //Strahl wird nach links abgelenkt
                        int X=x;
                        bool success = false;
                        while(success == false && getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X-1) > 0){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X), X-1, getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X-1));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[0]<=X&&intersection[0]>X-1){
                                rotatedEntryPoints[q][x][2] = intersection[2];
                                newRotatedEntryPoints[q][x].lengthEntry = intersection[2];
                                success = true;
                            }else{
                                X--;
                            }
                        }
                        if(success == false){
                            //qDebug()<<"Strahl wurde nach links abgelenkt und fand keine Rückseite bei: "<<q<<x;
                            if(getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X)!=getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X)){
                                QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X), X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X));
                                QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                                QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                                if(intersection[1]>=getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X)&&intersection[1]<=getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X)){
                                    //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                                    rotatedEntryPoints[q][x][2] = intersection[2];
                                    newRotatedEntryPoints[q][x].lengthEntry = intersection[2];
                                    success = true;
                                }
                            }
                            while(success == false && X<=x){
                                QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X), X+1, getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X+1));
                                QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                                QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                                if(intersection[0]>=X&&intersection[0]<X+1){
                                    rotatedEntryPoints[q][x][2] = intersection[2];
                                    newRotatedEntryPoints[q][x].lengthEntry = intersection[2];
                                    //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                                    success = true;
                                }else{
                                    X++;
                                }
                            }
                        }
                    }else{
                        //qDebug()<<"Strahl wurde total reflektiert!"<<q<<x;
                    }
                    if(createBScans){
                        bScan = propagateOctRayThroughHisto(rotatedHistoImages[q],bScan,double(x),rotatedEntryPoints[q][x][0],rotatedEntryPoints[q][x][1],rotatedEntryPoints[q][x][2],riMedium,riSample);
                    }
                    rotatedEntryPoints[q][x].append(bufVec[1]);
                    // Wert in ein Sinogram malen
                    if(rotatedEntryPoints[q][x][2]<0){
                        rotatedEntryPoints[q][x][2]=0;
                        qDebug()<<"irgendwas schiefgelaufen beim rayPropagate";
                    }
                    if(newRotatedEntryPoints[q][x].lengthEntry <0){
                        newRotatedEntryPoints[q][x].lengthEntry = 0;
                    }
                    double gray = 0.35*propagateRayThroughHisto(rotatedHistoImages[q],x,newRotatedEntryPoints[q][x].yEntry,newRotatedEntryPoints[q][x].gammaEntry,newRotatedEntryPoints[q][x].lengthEntry);
                    if(gray>65535){
                        qDebug()<<"Aufaddierter Grauwert zu groß für 16bit: "<<q<<x<<gray;
                        gray = 65535;
                    }
                    dstHisto[x]= int(gray);
                    if(createTransmission){
                        if(newRotatedEntryPoints[q][x].gammaEntry==1||newRotatedEntryPoints[q][x].gammaEntry==-1){
                            dstTrans[x]= int(0);
                        }else{
                            int buf = (65535 - gray)*qCos(newRotatedEntryPoints[q][x].gammaEntry*10.0);
                            if (buf<0){
                                buf=0;
                            }
                            dstTrans[x]= buf;
                        }
                    }
                }else{
                    rotatedEntryPoints[q][x] = {0,0,0,0};
                }
            }
            double rotateBy = (2*M_PI/double(numberOfProjections))*q;
            QString projAngle = QString::number(qRadiansToDegrees(rotateBy),'g',4);
            if(createBScans){
                bScan = noiseBScan(bScan,1);
                bScan.save(inputPathSurface+"\\"+nameRI+"_bScan_"+projAngle+".png");
                bScan.fill(0);
            }
        }
    }
    quint16 *dstSinoBot = (quint16*)(sinogramHisto.bits()+numberOfProjections*sinogramHisto.bytesPerLine());
    quint16 *dstSinoTop = (quint16*)(sinogramHisto.bits());
    for(int x = 0;x<sinogramHisto.width();x++){
        dstSinoBot[x] = dstSinoTop[x];
    }
    quint16 *dstSinoBot2 = (quint16*)(sinogramHisto.bits()+(numberOfProjections+1)*sinogramHisto.bytesPerLine());
    for(int x = 0;x<sinogramHisto.width();x++){
        dstSinoBot2[x] = dstSinoTop[x];
    }
    if(createTransmission){
        quint16 *dstTransBot = (quint16*)(transmissionHisto.bits()+numberOfProjections*transmissionHisto.bytesPerLine());
        quint16 *dstTransTop = (quint16*)(transmissionHisto.bits());
        for(int x = 0;x<transmissionHisto.width();x++){
            dstTransBot[x] = dstTransTop[x];
        }
        quint16 *dstTransBot2 = (quint16*)(transmissionHisto.bits()+(numberOfProjections+1)*transmissionHisto.bytesPerLine());
        for(int x = 0;x<transmissionHisto.width();x++){
            dstTransBot2[x] = dstTransBot[x];
        }
    }
    sinogramHisto.save(inputPathSurface+"\\"+nameRI+nrOfProjString+"_sinogramFromHisto.png");
    if(createTransmission){
        transmissionHisto.save(inputPathSurface+"\\"+nameRI+nrOfProjString+"_transmissionFromHisto.png");
    }
    qDebug()<<"jo";
}


void surface_fitting_test::on_doubleSpinBox_riMedium_valueChanged(double arg1)
{
    QVector<double> slope{0,0};
    riMedium = arg1;
    riMediumMT = riMedium;
    slope = getSlopeAtEntry(bufferImg,ui->spinBox_aScan->value(),ui->spinBox_slopeSigma->value());
    if(slope[1]!=999){
        qDebug()<<"slope at"<<ui->spinBox_aScan->value()<<"is: "<< slope;
        drawAndDisplaySlope(bufferImg,ui->spinBox_aScan->value(),slope[0],slope[1],20);
    }
}


void surface_fitting_test::on_doubleSpinBox_riSample_valueChanged(double arg1)
{
    QVector<double> slope{0,0};
    riSample = arg1;
    riSampleMT = riSample;
    slope = getSlopeAtEntry(bufferImg,ui->spinBox_aScan->value(),ui->spinBox_slopeSigma->value());
    if(slope[1]!=999){
        qDebug()<<"slope at"<<ui->spinBox_aScan->value()<<"is: "<< slope;
        drawAndDisplaySlope(bufferImg,ui->spinBox_aScan->value(),slope[0],slope[1],20);
    }
}


void surface_fitting_test::on_spinBox_rotateDisplayImageBy_valueChanged(int arg1)
{
    bufferImg.fill(0);
    for(int x = 0; x<inputSurface.width();x++){
        for(int y = 0; y<inputSurface.height();y++){
            if(getColor(inputSurface,x,y)!=0){
                QVector<double> newPixel = rotateImagePointTo({double(x),double(y)},{double(inputSurface.width()/2.0),double(inputSurface.height()/2.0)},rotateClockwise*qDegreesToRadians(double(arg1)),true);
                int xRound = std::round(newPixel[0]);
                int yRound = std::round(newPixel[1]);
                for(int x = xRound-2;x<=xRound+2;x++){
                    for(int y = yRound-2;y<=yRound+2;y++){
                        int newInt = getColor(bufferImg,x,y) + int(200*getValueForGaussDistribution(newPixel[0],newPixel[1],x,y,1));
                        if(newInt>255){
                            newInt=255;
                            qDebug()<<"Grauwert zu hoch";
                        }
                        QColor newColor(newInt,newInt,newInt,255);
                        bufferImg.setPixelColor(x,y,newColor);
                    }
                }
            }
        }
    }
    bufferImg = thinOutSurface(bufferImg);
    on_pushButton_getSlopeAt_clicked();
}


void surface_fitting_test::on_pushButton_correctSinogram_clicked() //Funktion veraltet
{
    QElapsedTimer reTimer;
    reTimer.start();
    QImage rearrangedSinogram = QImage(sinogramHisto.size(),QImage::Format_Grayscale16);
    rearrangedSinogramFails = QImage(sinogramHisto.size(),QImage::Format_RGB16);
    rearrangedSinogramFails.fill(0);
    rearrangedSinogram.fill(0);
    int deltaProj;
    if(!QDir(inputPathSurface).exists()){
        QDir().mkdir(inputPathSurface);
    }
    QVector<QVector<int>> travelledOnBackside;
    for(int p = 0; p<numberOfProjections;p++){
        qDebug()<<"Correction started for: "<<p<<reTimer.elapsed();
        quint16 *dstArrSino = (quint16*)(rearrangedSinogram.bits()+p*rearrangedSinogram.bytesPerLine());
        for(int x = 0; x<inputSurface.width();x++){
            if(newRotatedEntryPoints[p][x].yEntry!=0){
                rearrangedSinogramFails.setPixelColor(x,p,Qt::white);
                if(newRotatedEntryPoints[p][x].gammaEntry==0){                                                          //bei senkrechten Strahlen wird nichts verschoben!
                    quint16 *dstSinoStraight = (quint16*)(sinogramHisto.bits()+p*sinogramHisto.bytesPerLine());
                    rearrangedSinogramFails.setPixelColor(x,p,Qt::gray);
                    dstArrSino[x]=dstSinoStraight[x];
                }else if(newRotatedEntryPoints[p][x].gammaEntry<0){            // Probe wird im Uhrzeigersinn gedreht, um Partner zu finden; x wird größer
                    QVector<double> newPos;
                    double theta = 0;
                    deltaProj = 0;
                    double gamma = 0;
                    bool searchDone = false;
                    while(searchDone==false){
                        deltaProj++;
                        theta = (2*M_PI/double(numberOfProjections))*deltaProj;
                        newPos = rotateImagePointTo({double(x),newRotatedEntryPoints[p][x].yEntry},{double(inputSurface.width()/2.0),double(inputSurface.height()/2.0)},theta,true);
                        if(deltaProj>150){
                            rearrangedSinogramFails.setPixelColor(x,p,Qt::darkRed);
                            qDebug()<<"Punkt ist weggelaufen :"<<p<<x;
                            dstArrSino[x] = 0;
                            searchDone = true;
                        }
                        else if(riMedium<riSample && std::abs(linInterpolation(int(newPos[0]),int(newPos[0])+1,getFirstValueFromTop(rotatedSurfacesThinnedOut[(p+deltaProj+numberOfProjections)%numberOfProjections],int(newPos[0])),getFirstValueFromTop(rotatedSurfacesThinnedOut[(p+deltaProj+numberOfProjections)%numberOfProjections],int(newPos[0])+1),newPos[0])-newPos[1])>5){
                            rearrangedSinogramFails.setPixelColor(x,p,Qt::darkMagenta);
                            //qDebug()<<"Punkt auf Rückseite gelandet :"<<p<<x;
                            qDebug()<<newRotatedEntryPoints[p][x].gammaEntry;
                            travelledOnBackside.append({p,x});
                            dstArrSino[x] = 0;
                            searchDone = true;
                        }
//                        else if(std::abs(getFirstValueFromBottom(rotatedSurfacesThinnedOut[(p+deltaProj+numberOfProjections)%numberOfProjections],std::round(newPos[0]))-newPos[1])<5){
//                            rearrangedSinogramFails.setPixelColor(x,p,Qt::magenta);
//                            reOrderSucceded = true;
//                            qDebug()<<"Punkt auf Rückseite gelandet :"<<p<<x;
//                            qDebug()<<rotatedEntryPoints[p][x];
//                            dstArrSino[x] = 0;
//                            searchDone = true;
//                        }
                        else if(newRotatedEntryPoints[(p+deltaProj+numberOfProjections)%numberOfProjections][int(newPos[0])].lengthEntry!=0&&newRotatedEntryPoints[(p+deltaProj+numberOfProjections)%numberOfProjections][int(newPos[0])+1].lengthEntry!=0){
                            gamma = linInterpolation(int(newPos[0]),int(newPos[0])+1,newRotatedEntryPoints[(p+deltaProj+numberOfProjections)%numberOfProjections][int(newPos[0])].gammaEntry,newRotatedEntryPoints[(p+deltaProj+numberOfProjections)%numberOfProjections][int(newPos[0])+1].gammaEntry,newPos[0]);
                            if(theta >= std::abs(gamma)){
                                newPos = rotateImagePointTo({double(x),newRotatedEntryPoints[p][x].yEntry},{double(inputSurface.width()/2.0),double(inputSurface.height()/2.0)},-gamma,true);
                                double value = bilinInterpolFromSinogram(sinogramHisto,newPos[0],-gamma,p,deltaProj);
                                if(value<0){
                                    rearrangedSinogramFails.setPixelColor(x,p,Qt::cyan);
                                    qDebug()<<"Wert war unter Null bei: "<<p<<x;
                                    value=0;
                                }else if(value>65000){
                                    value=65000;
                                    qDebug()<<"Wert aus Sinograminterpolation war zu hoch";
                                }else{
                                    rearrangedSinogramFails.setPixelColor(x,p,Qt::darkBlue);
                                }
                                dstArrSino[x] = value;
                                searchDone = true;
                            }
                        }
                    }
                }else if(newRotatedEntryPoints[p][x].gammaEntry>0){
                    QVector<double> newPos;
                    double theta = 0;
                    deltaProj = 0;
                    double gamma = 0;
                    bool searchDone = false;
                    while(searchDone==false){
                        deltaProj--;
                        theta = (2*M_PI/double(numberOfProjections))*deltaProj;
                        newPos = rotateImagePointTo({double(x),newRotatedEntryPoints[p][x].yEntry},{double(inputSurface.width()/2.0),double(inputSurface.height()/2.0)},theta,true);
                        if(std::abs(deltaProj)>150){
                            qDebug()<<"Punkt ist weggelaufen :"<<p<<x;
                            dstArrSino[x] = 0;
                            rearrangedSinogramFails.setPixelColor(x,p,Qt::red);
                            searchDone = true;
                        }
                        else if(riMedium<riSample&&std::abs(linInterpolation(int(newPos[0]),int(newPos[0])+1,getFirstValueFromTop(rotatedSurfacesThinnedOut[(p+deltaProj+numberOfProjections)%numberOfProjections],int(newPos[0])),getFirstValueFromTop(rotatedSurfacesThinnedOut[(p+deltaProj+numberOfProjections)%numberOfProjections],int(newPos[0])+1),newPos[0])-newPos[1])>5){
                            rearrangedSinogramFails.setPixelColor(x,p,Qt::darkMagenta);
                            //qDebug()<<"Punkt auf Rückseite gelandet :"<<p<<x;
                            qDebug()<<newRotatedEntryPoints[p][x].yEntry;
                            travelledOnBackside.append({p,x});
                            dstArrSino[x] = 0;
                            searchDone = true;
                        }
//                        else if(std::abs(getFirstValueFromBottom(rotatedSurfacesThinnedOut[(p+deltaProj+numberOfProjections)%numberOfProjections],std::round(newPos[0]))-newPos[1])<5){
//                            rearrangedSinogramFails.setPixelColor(x,p,Qt::darkMagenta);
//                            qDebug()<<"Punkt auf Rückseite gelandet :"<<p<<x;
//                            qDebug()<<rotatedEntryPoints[p][x];
//                            reOrderSucceded = true;
//                            dstArrSino[x] = 0;
//                            searchDone = true;
//                        }
                        else if(newRotatedEntryPoints[(p+deltaProj+numberOfProjections)%numberOfProjections][int(newPos[0])].lengthEntry!=0&&newRotatedEntryPoints[(p+deltaProj+numberOfProjections)%numberOfProjections][int(newPos[0])+1].lengthEntry!=0){
                            gamma = linInterpolation(int(newPos[0]),int(newPos[0])+1,newRotatedEntryPoints[(p+deltaProj+numberOfProjections)%numberOfProjections][int(newPos[0])].gammaEntry,newRotatedEntryPoints[(p+deltaProj+numberOfProjections)%numberOfProjections][int(newPos[0])+1].gammaEntry,newPos[0]);
                            if(std::abs(theta) >= std::abs(gamma)){
                                newPos = rotateImagePointTo({double(x),newRotatedEntryPoints[p][x].yEntry},{double(inputSurface.width()/2.0),double(inputSurface.height()/2.0)},-gamma,true);
                                double value = bilinInterpolFromSinogram(sinogramHisto,newPos[0],-gamma,p,deltaProj);
                                if(value<0){
                                    rearrangedSinogramFails.setPixelColor(x,p,Qt::darkCyan);
                                    qDebug()<<"Wert war unter Null bei: "<<p<<x;
                                    value=0;
                                }else if(value>65000){
                                    value=65000;
                                    qDebug()<<"Wert aus Sinograminterpolation war zu hoch";
                                }else{
                                    rearrangedSinogramFails.setPixelColor(x,p,Qt::green);
                                }
                                dstArrSino[x] = value;
                                searchDone = true;
                            }
                        }
                    }
                }
            }
        }
    } //Am Ende noch die letzten beiden Zeilen aus der ersten kopieren
    quint16 *dstArrSinoBot = (quint16*)(rearrangedSinogram.bits()+numberOfProjections*rearrangedSinogram.bytesPerLine());
    quint16 *dstArrSinoTop = (quint16*)(rearrangedSinogram.bits());
    for(int x = 0;x<sinogramHisto.width();x++){
        dstArrSinoBot[x] = dstArrSinoTop[x];
    }
    quint16 *dstArrSinoBot2 = (quint16*)(rearrangedSinogram.bits()+(numberOfProjections+1)*rearrangedSinogram.bytesPerLine());
    for(int x = 0;x<sinogramHisto.width();x++){
        dstArrSinoBot2[x] = dstArrSinoTop[x];
    }    
    for(int r = 0;r<travelledOnBackside.size();r++){ // Wenn Punkte bei Korrektur auf die Rückseite gewandert sind, wird hier der Wert "von der anderen Seite" kopiert
        int fromX = inputHisto.width()-travelledOnBackside[r][1]-1;
        quint16 *dstArrSinoSource = (quint16*)(rearrangedSinogram.bits()+((travelledOnBackside[r][0]+numberOfProjections/2)%numberOfProjections)*rearrangedSinogram.bytesPerLine());
        quint16 *dstArrSinoFailed = (quint16*)(rearrangedSinogram.bits()+(travelledOnBackside[r][0])*rearrangedSinogram.bytesPerLine());
        if(dstArrSinoSource[fromX]!=0){
            rearrangedSinogramFails.setPixelColor(travelledOnBackside[r][1],travelledOnBackside[r][0],Qt::yellow);
            dstArrSinoFailed[travelledOnBackside[r][1]] = dstArrSinoSource[fromX];
        }
    }
    rearrangedSinogramFails.save(inputPathSurface+"\\"+nameRI+nrOfProjString+"_sinogramRearrangedFails.png");
    rearrangedSinogram.save(inputPathSurface+"\\"+nameRI+nrOfProjString+"_sinogramRearranged.png");
    rearrangedSinogramFails.fill(0);
}

void surface_fitting_test::on_pushButton_testMath_clicked()
{
//    determineTeleError(momentsList);
//    momentsList = makeListRelativeAndScaled(momentsList);
//    QVector<double> fittedRI(momentsList.size());
//    for (int x = 2; x<momentsList.size()-2;x++){
//        fittedRI[x] = getFittingSampleRI(rotatedSurfacesThinnedOut[0],x,momentsList[x][0],ui->doubleSpinBox_riMedium->value(),ui->doubleSpinBox_riSample->value(),0.04,0.001,0.1);
//        qDebug()<<"Fitted RI für Strahl "<<x<<fittedRI[x];
//    }
//    qDebug()<<"Punkte ohne Oberfläche entfernt "<<fittedRI.removeAll(999);
//    fittedRI.removeAll(333);
//    fittedRI.removeAll(777);
//    fittedRI.removeAll(0);
//    qDebug()<<fittedRI;
//    double sum=0;
//    for(int i = 0; i<fittedRI.size();i++){
//        sum = sum + fittedRI[i];
//    }
//    qDebug()<<"Durchschnittlicher Wert für RI Sample ist: "<<sum/fittedRI.size();

    getBackExitPointAndAngle(rotatedSurfacesThinnedOut[0],ui->spinBox_aScan->value(),ui->doubleSpinBox_riMedium->value(),ui->doubleSpinBox_riSample->value());
}

void surface_fitting_test::on_spinBox_arithMiddleSigma_valueChanged(int arg1)
{
    arithMiddleSigma = arg1;
    on_spinBox_rotateDisplayImageBy_valueChanged(ui->spinBox_rotateDisplayImageBy->value());
}


void surface_fitting_test::on_pushButton_rotateSurfaceAndHisto_clicked()
{

    if(!QDir(inputPathSurface).exists()){
        QDir().mkdir(inputPathSurface);
    }
    if(!QDir(inputPathHisto).exists()){
        QDir().mkdir(inputPathHisto);
    }
    QElapsedTimer bigTimer;
    bigTimer.start();
    numberOfProjections = ui->spinBox_nrOfProjections->value();
    nrOfProjString = QString::number(numberOfProjections);
    rotatedHistoImages = QVector<QImage>(numberOfProjections);
    rotatedSurfacesThinnedOut = QVector<QImage>(numberOfProjections);
    if(useArrayFire){
        qDebug()<<"Surfaces rotated"<<bigTimer.elapsed();
        rotatedSurfacesThinnedOut = makeRotatedImageStack(inputPathSurface,numberOfProjections);
        if(useMultiThreading){
            QtConcurrent::blockingMap(rotatedSurfacesThinnedOut,&threadBoi::thinOutSurfaceThreaded);
        }else{
            for(int k = 0; k<numberOfProjections;k++){
                rotatedSurfacesThinnedOut[k]=thinOutSurface(rotatedSurfacesThinnedOut[k]);
            }
        }
        qDebug()<<"Surfaces thinned out"<<bigTimer.elapsed();
        rotatedHistoImages = makeRotatedImageStack(inputPathHisto,numberOfProjections);
        qDebug()<<"Histos rotated"<<bigTimer.elapsed();
    }
    else{
        for(int k = 0;k<numberOfProjections;k++){
            QVector<double>rotatedPoint=QVector<double>(2);
            double rotateBy = (2*M_PI/double(numberOfProjections))*k;
            rotatedSurfacesThinnedOut[k] = QImage(inputSurface.size(),QImage::Format_Grayscale8);
            rotatedSurfacesThinnedOut[k].fill(0);
            for(int x = 0;x<inputSurface.width();x++){
                for(int y = 0;y<inputSurface.height();y++){
                    if(getColor(inputSurface,x,y)!=0){
                        rotatedPoint=rotateImagePointTo({double(x),double(y)},{double(inputSurface.width()/2.0),double(inputSurface.height()/2.0)},rotateClockwise*rotateBy,true);
                        int xRound = std::round(rotatedPoint[0]);
                        int yRound = std::round(rotatedPoint[1]);
                        for(int xGauss = xRound-2;xGauss<=xRound+2;xGauss++){
                            for(int yGauss = yRound-2;yGauss<=yRound+2;yGauss++){
                                int newInt = getColor(rotatedSurfacesThinnedOut[k],xGauss,yGauss) + int(200.0*getValueForGaussDistribution(rotatedPoint[0],rotatedPoint[1],xGauss,yGauss,1));
                                if(newInt>255){
                                    newInt=255;
                                    qDebug()<<"Grauwert zu hoch";
                                }
                                quint8 *dstRotSurface = (quint8*)(rotatedSurfacesThinnedOut[k].bits()+yGauss*rotatedSurfacesThinnedOut[k].bytesPerLine());
                                dstRotSurface[xGauss] = newInt;
                            }
                        }
                    }
                }
            }
            rotatedSurfacesThinnedOut[k]=thinOutSurface(rotatedSurfacesThinnedOut[k]);
            rotatedHistoImages[k]=QImage(inputHisto.size(),QImage::Format_Grayscale8);
            rotatedHistoImages[k].fill(0);
            double rotationAngleHisto = double((2*M_PI)/double(numberOfProjections))*double(k);
            double histoColor;
            for(int x = 0; x<inputHisto.width();x++){
                for(int y = 0;y<inputHisto.height();y++){
                    histoColor = getColor(inputHisto,x,y);
                    if(histoColor!=0){
                        QVector<double> newPixel = rotateImagePointTo({double(x),double(y)},{double(inputHisto.width()/2.0),double(inputHisto.height()/2.0)},rotateClockwise*rotationAngleHisto,true);
                        int xRound = std::round(newPixel[0]);
                        int yRound = std::round(newPixel[1]);
                        for(int xGauss = xRound-2;xGauss<=xRound+2;xGauss++){
                            for(int yGauss = yRound-2;yGauss<=yRound+2;yGauss++){
                                int newInt = getColor(rotatedHistoImages[k],xGauss,yGauss) + int(histoColor*getValueForGaussDistribution(newPixel[0],newPixel[1],xGauss,yGauss,1));
                                if(newInt>255){
                                    newInt=255;
                                    qDebug()<<"Grauwert zu hoch";
                                }
                                quint8 *dstRotHisto = (quint8*)(rotatedHistoImages[k].bits()+yGauss*rotatedHistoImages[k].bytesPerLine());
                                dstRotHisto[xGauss] = newInt;
                            }
                        }
                    }
                }
            }
            qDebug()<<"Histogram rotiert: "<<k<<bigTimer.elapsed();
        }
    }
}

void surface_fitting_test::on_pushButton_continousSimulation_clicked()
{
    while(riMedium<=ui->doubleSpinBox_endRiMedium->value()){
        on_pushButton_createSinogram_clicked();
        on_pushButton_newCorrection_clicked();
        rotatedEntryPoints.resize(0);
        newRotatedEntryPoints.resize(0);
        sinogramHisto.fill(0);
        QString riString = QString::number(riMedium,'g',4);
        qDebug()<<"Simulation of following Medium RI is fnished:"<<riString;
        riMedium = riMedium + 0.02;
    }
    qDebug()<<"All Simulations finished";
}


void surface_fitting_test::on_pushButton_newCorrection_clicked()
{
    QElapsedTimer reTimer;
    reTimer.start();
    QImage rearrangedSinogram = QImage(sinogramHisto.size(),QImage::Format_Grayscale16);
    rearrangedSinogramFails = QImage(sinogramHisto.size(),QImage::Format_RGB16);
    rearrangedSinogramFails.fill(0);
    rearrangedSinogram.fill(0);
    QVector<QVector<int>> travelledOnBackside;
    for(int p = 0; p<numberOfProjections;p++){
        qDebug()<<"Correction started for: "<<p<<reTimer.elapsed();
        quint16 *dstArrSino = (quint16*)(rearrangedSinogram.bits()+p*rearrangedSinogram.bytesPerLine());
        for(int x = 0; x<inputSurface.width();x++){
            if(newRotatedEntryPoints[p][x].yEntry!=0){
                rearrangedSinogramFails.setPixelColor(x,p,Qt::white);
                if(newRotatedEntryPoints[p][x].gammaEntry==0){                                                          //bei senkrechten Strahlen wird nichts verschoben!
                    quint16 *dstSinoStraight = (quint16*)(sinogramHisto.bits()+p*sinogramHisto.bytesPerLine());
                    rearrangedSinogramFails.setPixelColor(x,p,Qt::gray);
                    dstArrSino[x]=dstSinoStraight[x];
                }else{
                    double deltaTheta = getDeltaThetaForPartner(newRotatedEntryPoints[p][x].slopeEntry,riMedium,riSample);
                    if(deltaTheta!=999){
                        double deltaProj = deltaTheta*double(numberOfProjections)/(2*M_PI);
                        QVector<double> buf = rotateImagePointTo({double(x),newRotatedEntryPoints[p][x].yEntry},{double(inputSurface.width()/2.0),double(inputSurface.height()/2.0)},deltaTheta,true);
                        double absNewX = buf[0];
                        double value = newBilinInterpolFromSinogram(sinogramHisto,absNewX,double(p)+rotateClockwise*deltaProj);
                        if(value<0){
                            rearrangedSinogramFails.setPixelColor(x,p,Qt::cyan);
                            qDebug()<<"Wert war unter Null bei: "<<p<<x;
                            value=0;
                        }else if(value>65000){
                            value=65000;
                            qDebug()<<"Wert aus Sinograminterpolation war zu hoch";
                        }else{
                            rearrangedSinogramFails.setPixelColor(x,p,Qt::darkBlue);
                        }
                        dstArrSino[x] = value;
                    }else{
                        travelledOnBackside.append({p,x});
                        rearrangedSinogramFails.setPixelColor(x,p,Qt::red);
                    }
                }

            }
        }
    } //Am Ende noch die letzten beiden Zeilen aus der ersten kopieren
    quint16 *dstArrSinoBot = (quint16*)(rearrangedSinogram.bits()+numberOfProjections*rearrangedSinogram.bytesPerLine());
    quint16 *dstArrSinoTop = (quint16*)(rearrangedSinogram.bits());
    for(int x = 0;x<sinogramHisto.width();x++){
        dstArrSinoBot[x] = dstArrSinoTop[x];
    }
    quint16 *dstArrSinoBot2 = (quint16*)(rearrangedSinogram.bits()+(numberOfProjections+1)*rearrangedSinogram.bytesPerLine());
    for(int x = 0;x<sinogramHisto.width();x++){
        dstArrSinoBot2[x] = dstArrSinoTop[x];
    }
    for(int r = 0;r<travelledOnBackside.size();r++){ // Wenn Punkte bei Korrektur auf die Rückseite gewandert sind, wird hier der Wert "von der anderen Seite" kopiert
        int fromX = inputHisto.width()-travelledOnBackside[r][1]-1;
        quint16 *dstArrSinoSource = (quint16*)(rearrangedSinogram.bits()+((travelledOnBackside[r][0]+numberOfProjections/2)%numberOfProjections)*rearrangedSinogram.bytesPerLine());
        quint16 *dstArrSinoFailed = (quint16*)(rearrangedSinogram.bits()+(travelledOnBackside[r][0])*rearrangedSinogram.bytesPerLine());
        if(dstArrSinoSource[fromX]!=0){
            rearrangedSinogramFails.setPixelColor(travelledOnBackside[r][1],travelledOnBackside[r][0],Qt::yellow);
            dstArrSinoFailed[travelledOnBackside[r][1]] = dstArrSinoSource[fromX];
        }
    }
    rearrangedSinogramFails.save(inputPathSurface+"\\"+nameRI+nrOfProjString+"_sinogramRearrangedFailsNewMethod.png");
    rearrangedSinogram.save(inputPathSurface+"\\"+nameRI+nrOfProjString+"_sinogramRearrangedNewMethod.png");
    rearrangedSinogramFails.fill(0);
}




void surface_fitting_test::on_pushButton_correctInputSinogram_clicked()
{
    if(!QDir(inputPathSinogram).exists()){
        QDir().mkdir(inputPathSinogram);
    }
    QImage exSino = correctExternalSinogram(inputSinogram,inputSurface,inputPathSurface,riMedium,riSample);
    QString nrOfProjString2 = QString::number(inputSinogram.height()-2);
    QString nameRI2 = QString::number(riMedium,'g',4);
    nameRI2.append("_");
    nameRI2.append(QString::number(riSample,'g',4));
    nameRI2.append("_");
    exSino.save(inputPathSinogram+"\\"+nameRI2+nrOfProjString2+"_sinogramRearrangedFromExternal.png");
}


void surface_fitting_test::on_checkBox_createTransmission_toggled(bool checked)
{
    createTransmission = checked;
    qDebug()<<"Transmission wird erstellt:"<<checked;
}


void surface_fitting_test::on_checkBox_accountForReflection_stateChanged(int arg1)
{
    accountForReflection = arg1;
}


void surface_fitting_test::on_pushButton_chooseSurfaceDirectory_clicked()
{
    const QString folderpathSurfaceStack = QFileDialog::getExistingDirectory(this,tr("Surface Folder"),"E:/mSLOT/");
    QDir dir(folderpathSurfaceStack);
    fileListInfoSurface = dir.entryInfoList();

    fileListInfoSurface.removeFirst();
    fileListInfoSurface.removeFirst();
    qDebug()<<fileListInfoSurface;
}


void surface_fitting_test::on_pushButton_ChooseSinogramDirectory_clicked()
{
    const QString folderpathSurfaceStack = QFileDialog::getExistingDirectory(this,tr("Surface Folder"),"E:/mSLOT/");
    QDir dir(folderpathSurfaceStack);
    fileListInfoSinogram = dir.entryInfoList();
    fileListInfoSinogram.removeFirst();
    fileListInfoSinogram.removeFirst();
}


void surface_fitting_test::on_pushButton_correctStack_clicked()
{
    riMedium = ui->doubleSpinBox_riMedium->value();
    riSample = ui->doubleSpinBox_riSample->value();
    arithMiddleSigma = ui->spinBox_arithMiddleSigma->value();
    const QString folderpathSave = QFileDialog::getExistingDirectory(this,tr("Surface Folder"),"E:/mSLOT/");
    QDir saveDir(folderpathSave);
    QString savePath = saveDir.absolutePath();
    savePath.append("/");
    qDebug()<<savePath;
    if(fileListInfoSinogram.size()==fileListInfoSurface.size()){
        int stackSize = fileListInfoSinogram.size();
        for ( int i = 0; i<stackSize;i++){
            QImage bufSinogram, bufSurface;
            if(bufSinogram.load(fileListInfoSinogram[i].absoluteFilePath())&&bufSurface.load(fileListInfoSurface[i].absoluteFilePath())){
                numberOfProjections = bufSinogram.height()-2;
                QImage correctedSinogram = correctExternalSinogram(bufSinogram,bufSurface, fileListInfoSurface[i].absoluteFilePath(),riMedium,riSample);
                QString name = "korrigiertesSinogram_";
                name.append(QString::number(i)).append(".png");
                correctedSinogram.save(savePath + name);
            }else{
                qDebug()<<"Loading images Failed!";
            }
        }
    }else{
        qDebug()<<"Number of Surface and Sinograms did not match";
    }
}

void surface_fitting_test::on_checkBox_useArrayFire_stateChanged(int arg1)
{
    useArrayFire = arg1;
}


void surface_fitting_test::on_pushButton_varyingRiCorrection_clicked()
{
    const QString folderpathSave = QFileDialog::getExistingDirectory(this,tr("Surface Folder"),"E:/mSLOT/");
    QDir saveDir(folderpathSave);
    QString savePath = saveDir.absolutePath();
    savePath.append("/");
    while(riMedium<=targetMediumRi){
        numberOfProjections = inputSinogram.height()-2;
        QImage correctedSinogram = correctExternalSinogram(inputSinogram,inputSurface,inputPathSurface,riMedium,riSample);
        qDebug()<<"Sinogram mit medium RI"<<QString::number(riMedium,'g',4)<<"korrigiert. Ziel RI Medium ist:"<<QString::number(targetMediumRi,'g',4);
        QString name = "korrigiertesSinogram_";
        name.append(QString::number(riMedium,'g',4)).append(".png");
        qDebug()<<savePath<<name;
        correctedSinogram.save(savePath + name);
        riMedium = riMedium + riIncrement;
    }
}


void surface_fitting_test::on_doubleSpinBox_varyingMediumRi_valueChanged(double arg1)
{
    targetMediumRi = arg1;
}


void surface_fitting_test::on_doubleSpinBox_mediumRiIncrement_valueChanged(double arg1)
{
    riIncrement = arg1;
}


void surface_fitting_test::on_checkBox_clockwise_stateChanged(int arg1)
{
    if(arg1==0){
        rotateClockwise = -1;
    }else{
        rotateClockwise = 1;
    }
    qDebug()<<rotateClockwise;
}


void surface_fitting_test::on_pushButton_startThread_clicked()
{
    connect(&thready,&threadBoi::thinnedOutSurface,this,&surface_fitting_test::fillInThinnedSurface);
    QElapsedTimer timmer;
    rotatedSurfacesThinnedOut = makeRotatedImageStack(inputPathSurface,numberOfProjections);
    qDebug()<<"Gleich gehts los mit Multithreading!!";
    timmer.start();
    QtConcurrent::blockingMap(rotatedSurfacesThinnedOut,&threadBoi::thinOutSurfaceThreaded);
    qDebug()<<"Multithreading dauerte:"<<timmer.elapsed();
    displayImageLeft(rotatedSurfacesThinnedOut[300]);
}


void surface_fitting_test::on_pushButton_stopThread_clicked()
{
    emit on_stop();
}


void surface_fitting_test::on_spinBox_nrOfProjections_valueChanged(int arg1)
{
    numberOfProjections = arg1;
}


void surface_fitting_test::on_checkBox_useMultiThreading_stateChanged(int arg1)
{
    if(arg1==0){
        useMultiThreading = false;
    }else{
        useMultiThreading = true;
    }
    qDebug()<<"Multithreading aktivier: "<<useMultiThreading;
}


void surface_fitting_test::on_radioButton_pmt_toggled(bool checked)
{
    if(checked){
        correctingPmtSinogram = true;
        qDebug()<<"PMT Sinogram wird korrigiert:"<<correctingPmtSinogram;
    }
}

void surface_fitting_test::on_radioButton_PD_toggled(bool checked)
{
    if(checked){
        correctingPmtSinogram = false;
        qDebug()<<"PMT Sinogram wird korrigiert:"<<correctingPmtSinogram;
    }
}

void surface_fitting_test::on_pushButton_openOpenCV_clicked()
{
    ref = new refraction();
    ref->show();
    connect(ref,&refraction::sendMomentsList,this,&surface_fitting_test::getMomentsList);
}

