#include "header/surface_fitting_test.h"
#include "ui_surface_fitting_test.h"

double surface_fitting_test::riMediumMT = 1;
double surface_fitting_test::riSampleMT = 1;
int surface_fitting_test::slopeSigmaMT = 1;
int surface_fitting_test::slopePxlNoMT = 1;
bool surface_fitting_test::usePoly = 1;
int surface_fitting_test::fitOrderMT = 1;

surface_fitting_test::surface_fitting_test(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::surface_fitting_test)
{
    ui->setupUi(this);
    riMediumMT = ui->doubleSpinBox_riMedium->value();
    riMedium = riMediumMT;
    riSampleMT = ui->doubleSpinBox_riSample->value();
    riSample = riSampleMT;
    slopeSigmaMT = ui->spinBox_slopeSigma->value();
    slopePxlNoMT = ui->spinBox_ariMiddleSigma->value();
    fitOrder = ui->spinBox_fitOrder->value();
    fitOrderMT = fitOrder;
    rotateClockwise = -1;
}

surface_fitting_test::~surface_fitting_test()
{
    emit on_stop();
    delete ui;
}

void surface_fitting_test::on_pushButton_loadAndDisplay_clicked()
{
    arithMiddleSigma = ui->spinBox_ariMiddleSigma->value();
    inputPathSurface = ui->lineEdit_inputSurface->text();
    inputPathSurface.replace("\\","/",Qt::CaseSensitivity());
    if(inputSurface.load(inputPathSurface)==0){
        qDebug()<<"Failed to load input surface, check path";
    }
    inputSurface.convertTo(QImage::Format_Grayscale8,Qt::AutoColor);
    inputPathHisto = ui->lineEdit_inputHisto->text();
    inputPathHisto.replace("\\","/",Qt::CaseSensitivity());
    if(inputHisto.load(inputPathHisto)==0){
        qDebug()<<"Failed to load input histogram, check path";
    }
    inputHisto.convertTo(QImage::Format_Grayscale8,Qt::AutoColor);
    bufferImg = inputSurface;
    displayImageLeft(inputSurface);
    riMedium = ui->doubleSpinBox_riMedium->value();
    riSample = ui->doubleSpinBox_riSample->value();
    numberOfProjections = ui->spinBox_nrOfProjections->value();
}

void surface_fitting_test::on_pushButton_loadAndDisplaySinogram_clicked()
{
    arithMiddleSigma = ui->spinBox_ariMiddleSigma->value();
    inputPathSurface = ui->lineEdit_inputSurface->text();
    inputPathSurface.replace("\\","/",Qt::CaseSensitivity());
    if(inputSurface.load(inputPathSurface)==0){
        qDebug()<<"Failed to load input surface, check path";
    }
    inputSurface.convertTo(QImage::Format_Grayscale8,Qt::AutoColor);
    inputPathSinogram = ui->lineEdit_inputSinogram->text();
    inputPathSinogram.replace("\\","/",Qt::CaseSensitivity());
    if(inputSinogram.load(inputPathSinogram)==0){
        qDebug()<<"Failed to load input sinogram, check path";
    }
    inputHisto.convertTo(QImage::Format_Grayscale8,Qt::AutoColor);
    auto sinogramFormat = inputSinogram.pixelFormat();
    qDebug()<<"Sinogram Bits per Pixel:"<<sinogramFormat.bitsPerPixel();
    qDebug()<<"Sinogram Color Model (3 is grayscale)"<<sinogramFormat.colorModel();
    if(inputSinogram.width()!=inputSurface.width()){
        qDebug()<<"Sinogram and Surface don't match!!";
    }
    bufferImg = inputSurface;
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
    if(X<0||X>=image.width()){
        return 0;
    }
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
    if(X<0||X>=image.width()){
        return 0;
    }
    for(int y = image.height()-1;y>=0;y--){
        if(getColor(image,X,y)!=0){
            return y;
        }
    }
    return 0;
}

int surface_fitting_test::getFirstIndexFromLeft(QImage surface)
{
    int index = 0;
    while(getFirstValueFromTop(surface,index)==0){
        index++;
    }
    return index;
}

int surface_fitting_test::getFirstIndexFromRight(QImage surface)
{
    int index = surface.width()-1;
    while(getFirstValueFromTop(surface,index)==0){
        index--;
    }
    return index;
}

QImage surface_fitting_test::thinOutSurface(QImage image)
{
    QImage output = QImage(image.size(),QImage::Format_Grayscale8);
    output.fill(0);
    for(int x = 0;x<image.width();x++){
        int y=getFirstValueFromTop(image,x);
        if(y!=0){
            QVector<int> ascan = fillVectorWithAscan(image,x);
            double middle = getArithmicMiddle(ascan,y,arithMiddleSigma);
            output.setPixelColor(x,std::round(middle),255);
        }
    }
    for(int x = 0;x<image.width();x++){
        int y=getFirstValueFromBottom(image,x);
        if(y!=0){
            QVector<int> ascan = fillVectorWithAscan(image,x);
            double middle = getArithmicMiddle(ascan,y,-arithMiddleSigma);
            output.setPixelColor(x,std::round(middle),255);
        }
    }
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
    rotatedSurfaces = QVector<QImage>(numberOfAngles);
    rotatedSurfacesThinnedOut = QVector<QImage>(numberOfAngles);
    newRotatedEntryPoints = QVector<QVector<entryPoint>>(numberOfAngles);
    QVector<surfaceInfo> surfaceList;
    QVector<newSurfaceInfo> newSurfaceList;
    if(useArrayFire){
        rotatedSurfaces = makeRotatedImageStack(surfacePath, numberOfAngles);
        rotatedSurfacesThinnedOut = rotatedSurfaces;
    }
    if(useMultiThreading){
        QtConcurrent::blockingMap(rotatedSurfacesThinnedOut,&threadBoi::thinOutSurfaceThreaded);
        for(int k = 0;k<numberOfAngles;k++){
            newRotatedEntryPoints[k]=QVector<entryPoint>(rotatedSurfacesThinnedOut[0].width());
        }
        qDebug()<<"Struct wird erstellt"<<externalCorrectionTimer.restart();
        newSurfaceList = makeNewStructVector(rotatedSurfacesThinnedOut,rotatedSurfaces,newRotatedEntryPoints);
        qDebug()<<"Struct wurde erstellt"<<externalCorrectionTimer.restart();
        QtConcurrent::blockingMap(newSurfaceList,&surface_fitting_test::newPropagateRayMultiThreaded);
        qDebug()<<"Strahlen wurden MT propagiert"<<externalCorrectionTimer.restart();
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
            rotatedSurfaces[k] = QImage(surface.size(),QImage::Format_Grayscale8);
            rotatedSurfaces[k].fill(0);
            for(int x = 0;x<surface.width();x++){
                for(int y = 0;y<surface.height();y++){
                    if(getColor(surface,x,y)!=0){
                        rotatedPoint=rotateImagePointTo({double(x),double(y)},{double(surface.width()/2.0),double(surface.height()/2.0)},rotateClockwise*rotateBy,true);
                        int xRound = std::round(rotatedPoint[0]);
                        int yRound = std::round(rotatedPoint[1]);
                        for(int xGauss = xRound-2;xGauss<=xRound+2;xGauss++){
                            for(int yGauss = yRound-2;yGauss<=yRound+2;yGauss++){
                                int newInt = getColor(rotatedSurfaces[k],xGauss,yGauss) + int(200.0*getValueForGaussDistribution(rotatedPoint[0],rotatedPoint[1],xGauss,yGauss,1));
                                if(newInt>255){
                                    newInt=255;
                                    qDebug()<<"Grauwert zu hoch";
                                }
                                quint8 *dstRotSurface = (quint8*)(rotatedSurfaces[k].bits()+yGauss*rotatedSurfaces[k].bytesPerLine());
                                dstRotSurface[xGauss] = newInt;
                            }
                        }
                    }
                }
            }
        }
        if(!useMultiThreading){
            rotatedSurfacesThinnedOut[k]=thinOutSurface(rotatedSurfaces[k]);
        }
        newRotatedEntryPoints[k] = QVector<entryPoint>(surface.width());
        newRotatedEntryPoints[k][0]={0,0,0,0,0};
        newRotatedEntryPoints[k][0].xEntry = 0;
        //Strahl durch die "Projektion" propagieren
        for(int x = 1;x<surface.width();x++){
            newRotatedEntryPoints[k][x].xEntry = x;
            if(newRotatedEntryPoints[k][x-1].yEntry!=0 && newRotatedEntryPoints[k][x].yEntry!=0!=0 && newRotatedEntryPoints[k][x+1].yEntry!=0){ //dadurch werden die äußersten Punkte ignoriert
                QVector<double>bufVec = getSlopeAtEntry(rotatedSurfacesThinnedOut[k],x,slopeSigma);
                double exitAngle = getExitAngle(bufVec[1],mediumRI,sampleRI);
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
                            newRotatedEntryPoints[k][x].lengthEntry = intersection[2];
                            success = true;
                        }else{
                            X++;
                        }
                    }
                    if(success == false){
                        //qDebug()<<"Strahl wurde nach rechts abgelenkt und fand keine Rückseite bei: "<<k<<x;
                        if(newRotatedEntryPoints[k][X].yEntry!=getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X)){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, newRotatedEntryPoints[k][X].yEntry, X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[1]>=newRotatedEntryPoints[k][X].yEntry&&intersection[1]<=getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X)){
                                //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                                newRotatedEntryPoints[k][x].lengthEntry = intersection[2];
                                success = true;
                            }
                        }
                        while(success == false && X>=x){
                            QVector<double> surfaceVector = parameterizeFromPoints(X-1, newRotatedEntryPoints[k][X-1].yEntry, X, newRotatedEntryPoints[k][X].yEntry);
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[0]>=X-1&&intersection[0]<X){
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
                            newRotatedEntryPoints[k][x].lengthEntry = intersection[2];
                            success = true;
                        }else{
                            X--;
                        }
                    }
                    if(success == false){
                        //qDebug()<<"Strahl wurde nach links abgelenkt und fand keine Rückseite bei: "<<k<<x;
                        if(getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X)!=getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X)){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, newRotatedEntryPoints[k][X].yEntry, X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[1]>=newRotatedEntryPoints[k][X].yEntry&&intersection[1]<=getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X)){
                                //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                                newRotatedEntryPoints[k][x].lengthEntry = intersection[2];
                                success = true;
                            }
                        }
                        while(success == false && X<=x){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, newRotatedEntryPoints[k][X].yEntry, X+1, newRotatedEntryPoints[k][X+1].yEntry);
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[0]>=X&&intersection[0]<X+1){
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
                // Wert in ein Sinogram malen

                if(newRotatedEntryPoints[k][x].lengthEntry < 0){
                    newRotatedEntryPoints[k][x].lengthEntry = 0;
                }

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
            quint16 *dstSinoStraight = (quint16*)(sinogram.bits()+p*sinogram.bytesPerLine());
            if(newRotatedEntryPoints[p][x].yEntry!=0){
                if(newRotatedEntryPoints[p][x].gammaEntry==0){                                                  //bei senkrechten Strahlen wird nichts verschoben!
                    quint16 *dstSinoStraight = (quint16*)(sinogram.bits()+p*sinogram.bytesPerLine());
                    dstArrSino[x]=dstSinoStraight[x];
                }else{
                    double deltaTheta = getDeltaThetaForPartner(newRotatedEntryPoints[p][x].slopeEntry,mediumRI,sampleRI);
                    if(deltaTheta!=999){
                        double deltaProj = deltaTheta*double(numberOfAngles)/(2*M_PI);
                        QVector<double> buf = rotateImagePointTo({double(x),newRotatedEntryPoints[p][x].yEntry},{double(surface.width()/2),double(surface.height()/2)},deltaTheta,true);
                        double absNewX = buf[0];
                        if(newRotatedEntryPoints[(int(std::round(double(p)+rotateClockwise*deltaProj)+numberOfProjections))%(numberOfProjections)][std::round(absNewX)-2].yEntry!=0&&newRotatedEntryPoints[(int(std::round(double(p)+rotateClockwise*deltaProj)+numberOfProjections))%(numberOfProjections)][std::round(absNewX)+2].yEntry!=0){
                            long double value = newBilinInterpolFromSinogram(sinogram,absNewX,double(p)+rotateClockwise*deltaProj);
                            if(accountForReflection==true){
                             //   int newP = std::round(p+deltaProj);
                             //   newP = (newP+numberOfAngles)%numberOfAngles;
                             //   value = value/getTransmissionGrade(mediumRI,sampleRI,newRotatedEntryPoints[newP][std::round(absNewX)]); // muss noch auf neues Struct umgeschrieben werden ... hier müsste eigentlich der Winkel vom Partner angegeben werden!! Deswegen zu hell
                                value = value*(1+std::abs(newRotatedEntryPoints[(int(std::round(double(p)+rotateClockwise*deltaProj)+numberOfProjections))%(numberOfProjections)][absNewX].slopeEntry));
                                //valuePD = valuePD*(1+newRotatedEntryPoints[(int(std::round(double(p)+rotateClockwise*deltaProj)+numberOfProjections))%(numberOfProjections)][absNewX].slopeEntry);
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
                            dstArrSino[x] = dstSinoStraight[x];
                        }
                    }else{
                        travelledOnBackside.append({p,x});
                        dstArrSino[x] = dstSinoStraight[x];
                        //rearrangedSinogramFails.setPixelColor(x,p,Qt::red);
                    }
                }

            }else{
                dstArrSino[x] = dstSinoStraight[x];
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

    newRotatedEntryPoints.resize(0);
    rotatedSurfacesThinnedOut.resize(0);
    rotatedSurfaces.resize(0);
    qDebug()<<"Sinogram fertig sortiert!"<<externalCorrectionTimer.elapsed();
    return rearrangedExternalSinogram;
}

bool surface_fitting_test::correctExternalSinogramPDandPMT(QImage &sinogramPD, QImage &sinogramPMT, QImage &rearSinoPD, QImage &rearSinoPMT, QImage &surface, QString surfacePath, double mediumRI, double sampleRI)
{
    riMediumMT = mediumRI;
    riSampleMT = sampleRI;
    QElapsedTimer externalCorrectionTimer;
    externalCorrectionTimer.start();
    int slopeSigma = ui->spinBox_slopeSigma->value();
    slopeSigmaMT = ui->spinBox_slopeSigma->value();
    int numberOfAngles = sinogramPMT.height()-2;
    rotatedSurfaces = QVector<QImage>(numberOfAngles);
    rotatedSurfacesThinnedOut = QVector<QImage>(numberOfAngles);
    newRotatedEntryPoints = QVector<QVector<entryPoint>>(numberOfAngles);
    QVector<surfaceInfo> surfaceList;
    QVector<newSurfaceInfo> newSurfaceList;
    if(useArrayFire){
        rotatedSurfaces = makeRotatedImageStack(surfacePath, numberOfAngles);
        rotatedSurfacesThinnedOut = rotatedSurfaces;
    }
    if(useMultiThreading){
        QtConcurrent::blockingMap(rotatedSurfacesThinnedOut,&threadBoi::thinOutSurfaceThreaded);
        for(int k = 0;k<numberOfAngles;k++){
            newRotatedEntryPoints[k]=QVector<entryPoint>(rotatedSurfacesThinnedOut[0].width());
        }
        qDebug()<<"Struct wird erstellt"<<externalCorrectionTimer.restart();
        newSurfaceList = makeNewStructVector(rotatedSurfacesThinnedOut,rotatedSurfaces,newRotatedEntryPoints);
        qDebug()<<"Struct wurde erstellt"<<externalCorrectionTimer.restart();
        QtConcurrent::blockingMap(newSurfaceList,&surface_fitting_test::newPropagateRayMultiThreaded);
        qDebug()<<"Strahlen wurden MT propagiert"<<externalCorrectionTimer.restart();
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
            rotatedSurfaces[k] = QImage(surface.size(),QImage::Format_Grayscale8);
            rotatedSurfaces[k].fill(0);
            for(int x = 0;x<surface.width();x++){
                for(int y = 0;y<surface.height();y++){
                    if(getColor(surface,x,y)!=0){
                        rotatedPoint=rotateImagePointTo({double(x),double(y)},{double(surface.width()/2.0),double(surface.height()/2.0)},rotateClockwise*rotateBy,true);
                        int xRound = std::round(rotatedPoint[0]);
                        int yRound = std::round(rotatedPoint[1]);
                        for(int xGauss = xRound-2;xGauss<=xRound+2;xGauss++){
                            for(int yGauss = yRound-2;yGauss<=yRound+2;yGauss++){
                                int newInt = getColor(rotatedSurfaces[k],xGauss,yGauss) + int(200.0*getValueForGaussDistribution(rotatedPoint[0],rotatedPoint[1],xGauss,yGauss,1));
                                if(newInt>255){
                                    newInt=255;
                                    qDebug()<<"Grauwert zu hoch";
                                }
                                quint8 *dstRotSurface = (quint8*)(rotatedSurfaces[k].bits()+yGauss*rotatedSurfaces[k].bytesPerLine());
                                dstRotSurface[xGauss] = newInt;
                            }
                        }
                    }
                }
            }
        }
        if(!useMultiThreading){
            rotatedSurfacesThinnedOut[k]=thinOutSurface(rotatedSurfaces[k]);
        }
        newRotatedEntryPoints[k] = QVector<entryPoint>(surface.width());
        newRotatedEntryPoints[k][0]={0,0,0,0,0};
        newRotatedEntryPoints[k][0].xEntry = 0;
        //Strahl durch die "Projektion" propagieren
        for(int x = 1;x<surface.width();x++){
            newRotatedEntryPoints[k][x].xEntry = x;
            if(newRotatedEntryPoints[k][x-1].yEntry!=0 && newRotatedEntryPoints[k][x].yEntry!=0 && newRotatedEntryPoints[k][x+1].yEntry!=0){ //dadurch werden die äußersten Punkte ignoriert
                QVector<double>bufVec = getSlopeAtEntry(rotatedSurfacesThinnedOut[k],x,slopeSigma);
                double exitAngle = getExitAngle(bufVec[1],mediumRI,sampleRI);
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
                            newRotatedEntryPoints[k][x].lengthEntry = intersection[2];
                            success = true;
                        }else{
                            X++;
                        }
                    }
                    if(success == false){
                        //qDebug()<<"Strahl wurde nach rechts abgelenkt und fand keine Rückseite bei: "<<k<<x;
                        if(newRotatedEntryPoints[k][X].yEntry!=getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X)){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, newRotatedEntryPoints[k][X].yEntry, X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[1]>=newRotatedEntryPoints[k][X].yEntry&&intersection[1]<=getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X)){
                                //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                                newRotatedEntryPoints[k][x].lengthEntry = intersection[2];
                                success = true;
                            }
                        }
                        while(success == false && X>=x){
                            QVector<double> surfaceVector = parameterizeFromPoints(X-1, newRotatedEntryPoints[k][X-1].yEntry, X, newRotatedEntryPoints[k][X].yEntry);
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[0]>=X-1&&intersection[0]<X){
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
                            newRotatedEntryPoints[k][x].lengthEntry = intersection[2];
                            success = true;
                        }else{
                            X--;
                        }
                    }
                    if(success == false){
                        //qDebug()<<"Strahl wurde nach links abgelenkt und fand keine Rückseite bei: "<<k<<x;
                        if(getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X)!=getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X)){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, newRotatedEntryPoints[k][X].yEntry, X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[1]>=newRotatedEntryPoints[k][X].yEntry&&intersection[1]<=getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X)){
                                //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                                newRotatedEntryPoints[k][x].lengthEntry = intersection[2];
                                success = true;
                            }
                        }
                        while(success == false && X<=x){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, newRotatedEntryPoints[k][X].yEntry, X+1, newRotatedEntryPoints[k][X+1].yEntry);
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[0]>=X&&intersection[0]<X+1){
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
                // Wert in ein Sinogram malen

                if(newRotatedEntryPoints[k][x].lengthEntry < 0){
                    newRotatedEntryPoints[k][x].lengthEntry = 0;
                }

            }
        }
        qDebug()<<"Rays propagated for Input Surface "<<k<<"of "<<numberOfAngles<<", time: "<<externalCorrectionTimer.elapsed();
    }
    }
    QElapsedTimer reTimer;
    reTimer.start();
//    if(correctingPmtSinogram){
//        rearrangedExternalSinogramPMT.fill(0);
//    }else{
//        rearrangedExternalSinogram.fill(36000);
//    }
    quint16 *dstSinogramEdge = (quint16*)(sinogramPMT.bits()+(sinogramPMT.height()-5)*sinogramPMT.bytesPerLine());
    quint32 forAvg = 0;
    for(int x =8; x<14;x++){
        forAvg = forAvg + dstSinogramEdge[x];
    }
    rearSinoPMT.fill(forAvg/6);
    quint16 *dstSinogramEdgePD = (quint16*)(sinogramPD.bits()+(sinogramPD.height()-5)*sinogramPD.bytesPerLine());
    forAvg = 0;
    for(int x =8; x<14;x++){
        forAvg = forAvg + dstSinogramEdgePD[x];
    }
    rearSinoPD.fill(forAvg/6);
    QVector<QVector<int>> travelledOnBackside;
    for(int p = 0; p<numberOfAngles;p++){
        quint16 *dstArrSinoPMT = (quint16*)(rearSinoPMT.bits()+p*rearSinoPMT.bytesPerLine());
        quint16 *dstArrSinoPD = (quint16*)(rearSinoPD.bits()+p*rearSinoPD.bytesPerLine());
        for(int x = 0; x<surface.width();x++){
            quint16 *dstSinoStraightPMT = (quint16*)(sinogramPMT.bits()+p*sinogramPMT.bytesPerLine());
            quint16 *dstSinoStraightPD = (quint16*)(sinogramPD.bits()+p*sinogramPD.bytesPerLine());
            if(newRotatedEntryPoints[p][x].yEntry!=0){
                if(newRotatedEntryPoints[p][x].gammaEntry==0){
                    dstArrSinoPMT[x]=dstSinoStraightPMT[x];
                    dstArrSinoPD[x]=dstSinoStraightPD[x];
                }else{
                    double deltaTheta = getDeltaThetaForPartner(newRotatedEntryPoints[p][x].slopeEntry,mediumRI,sampleRI);
                    if(deltaTheta!=999){
                        double deltaProj = deltaTheta*double(numberOfAngles)/(2*M_PI);
                        QVector<double> buf = rotateImagePointTo({double(x),newRotatedEntryPoints[p][x].yEntry},{double(surface.width()/2),double(surface.height()/2)},deltaTheta,true);
                        double absNewX = buf[0];
                        if(newRotatedEntryPoints[(int(std::round(double(p)+rotateClockwise*deltaProj)+numberOfProjections))%(numberOfProjections)][std::round(absNewX)-3].yEntry!=0&&newRotatedEntryPoints[(int(std::round(double(p)+rotateClockwise*deltaProj)+numberOfProjections))%(numberOfProjections)][std::round(absNewX)+3].yEntry!=0){
                            long double valuePMT = newBilinInterpolFromSinogram(sinogramPMT,absNewX,double(p)+rotateClockwise*deltaProj);
                            long double valuePD = 300;
                            //if(newRotatedEntryPoints[(int(std::round(double(p)+rotateClockwise*deltaProj)+numberOfProjections))%(numberOfProjections)][std::round(absNewX)-5].yEntry!=0&&newRotatedEntryPoints[(int(std::round(double(p)+rotateClockwise*deltaProj)+numberOfProjections))%(numberOfProjections)][std::round(absNewX)+5].yEntry!=0){
                            //    valuePD = newBilinInterpolFromSinogram(sinogramPD,absNewX,double(p)+rotateClockwise*deltaProj);
                            //}
                            if(newRotatedEntryPoints[(int(std::round(double(p)+rotateClockwise*deltaProj)+numberOfProjections))%(numberOfProjections)][std::round(absNewX)-13].yEntry!=0&&newRotatedEntryPoints[(int(std::round(double(p)+rotateClockwise*deltaProj)+numberOfProjections))%(numberOfProjections)][std::round(absNewX)+13].yEntry!=0){
                                valuePD = newBilinInterpolFromSinogram(sinogramPD,absNewX,double(p)+rotateClockwise*deltaProj);
                                if(accountForReflection){
                                    valuePD = valuePD*(1+std::abs(newRotatedEntryPoints[(int(std::round(double(p)+rotateClockwise*deltaProj)+numberOfProjections))%(numberOfProjections)][absNewX].slopeEntry));
                                }
                            }
                            if(accountForReflection==true){
                                valuePMT = valuePMT*(1+std::abs(newRotatedEntryPoints[(int(std::round(double(p)+rotateClockwise*deltaProj)+numberOfProjections))%(numberOfProjections)][absNewX].slopeEntry));
                                }
                            if(valuePMT<0){
                                qDebug()<<"Wert war unter Null bei: "<<p<<x;
                                valuePMT=0;
                            }else if(valuePMT>65000){
                                valuePMT=65000;
                                qDebug()<<"Wert aus Sinograminterpolation war zu hoch";
                            }
                            if(valuePD<0){
                                qDebug()<<"Wert war unter Null bei: "<<p<<x;
                                valuePD=0;
                            }else if(valuePD>65000){
                                valuePD=65000;
                                qDebug()<<"Wert aus Sinograminterpolation war zu hoch";
                            }
                            //qDebug()<<"crash after line 462";
                            dstArrSinoPMT[x] = std::round(valuePMT);
                            dstArrSinoPD[x] = std::round(valuePD);
                        }else{
                            //dstArrSinoPMT[x] = dstSinoStraightPMT[x];
                            dstArrSinoPD[x]=dstSinoStraightPD[x];
                        }
                    }else{
                        travelledOnBackside.append({p,x});
//                        dstArrSinoPMT[x] = dstSinoStraightPMT[x];
                        dstArrSinoPD[x]=300;
                        //rearrangedSinogramFails.setPixelColor(x,p,Qt::red);
                    }
                }

            }else{
                dstArrSinoPMT[x] = dstSinoStraightPMT[x];
                dstArrSinoPD[x]=dstSinoStraightPD[x];
            }
        }
    } //Am Ende noch die letzten beiden Zeilen aus der ersten kopieren
    quint16 *dstArrSinoBotPMT = (quint16*)(rearSinoPMT.bits()+numberOfAngles*rearSinoPMT.bytesPerLine());
    quint16 *dstArrSinoBotPD = (quint16*)(rearSinoPD.bits()+numberOfAngles*rearSinoPD.bytesPerLine());
    quint16 *dstArrSinoTopPMT = (quint16*)(rearSinoPMT.bits());
    quint16 *dstArrSinoTopPD = (quint16*)(rearSinoPD.bits());
    for(int x = 0;x<sinogramPMT.width();x++){
        dstArrSinoBotPMT[x] = dstArrSinoTopPMT[x];
        dstArrSinoBotPD[x] = dstArrSinoTopPD[x];
    }
    quint16 *dstArrSinoBotPMT2 = (quint16*)(rearSinoPMT.bits()+(numberOfAngles+1)*rearSinoPMT.bytesPerLine());
    quint16 *dstArrSinoBotPD2 = (quint16*)(rearSinoPD.bits()+(numberOfAngles+1)*rearSinoPD.bytesPerLine());
    for(int x = 0;x<sinogramPMT.width();x++){
        dstArrSinoBotPMT2[x] = dstArrSinoTopPMT[x];
        dstArrSinoBotPD2[x] = dstArrSinoTopPD[x];
    }
    for(int r = 0;r<travelledOnBackside.size();r++){ // Wenn Punkte bei Korrektur auf die Rückseite gewandert sind, wird hier der Wert "von der anderen Seite" kopiert
        int fromX = sinogramPMT.width()-travelledOnBackside[r][1]-1;
        quint16 *dstArrSinoSource = (quint16*)(rearSinoPMT.bits()+((travelledOnBackside[r][0]+numberOfAngles/2)%numberOfAngles)*rearSinoPMT.bytesPerLine());
        quint16 *dstArrSinoFailed = (quint16*)(rearSinoPMT.bits()+(travelledOnBackside[r][0])*rearSinoPMT.bytesPerLine());
        if(dstArrSinoSource[fromX]!=0){
            //rearrangedSinogramFails.setPixelColor(travelledOnBackside[r][1],travelledOnBackside[r][0],Qt::yellow);
            dstArrSinoFailed[travelledOnBackside[r][1]] = dstArrSinoSource[fromX];
        }
        quint16 *dstArrSinoSourcePD = (quint16*)(rearSinoPD.bits()+((travelledOnBackside[r][0]+numberOfAngles/2)%numberOfAngles)*rearSinoPD.bytesPerLine());
        quint16 *dstArrSinoFailedPD = (quint16*)(rearSinoPD.bits()+(travelledOnBackside[r][0])*rearSinoPD.bytesPerLine());
        if(dstArrSinoSourcePD[fromX]!=0){
            //rearrangedSinogramFails.setPixelColor(travelledOnBackside[r][1],travelledOnBackside[r][0],Qt::yellow);
            //dstArrSinoFailedPD[travelledOnBackside[r][1]] = dstArrSinoSourcePD[fromX];
        }
    }

    newRotatedEntryPoints.resize(0);
    rotatedSurfacesThinnedOut.resize(0);
    rotatedSurfaces.resize(0);
    qDebug()<<"Sinogram fertig sortiert!"<<externalCorrectionTimer.elapsed();
    return true;
}


void surface_fitting_test::drawAndDisplaySlope(QImage image, int X, int Y, double slope, int width)
{
    QImage drawn = image;
    for(int x = X-width/2;x<=X+width/2;x++){
        int y = std::round(Y + (x-X)*slope);
        drawn.setPixelColor(x,y,Qt::green);
    }
    double gamma = getExitAngle(slope,riMedium,riSample);
    qDebug()<<"Gamma beträgt: "<<gamma;
    for(int y = 0;y<image.height()/10;y++){
        drawn.setPixelColor(X-1+std::round(y*qTan(gamma)),y+Y,Qt::red);
        drawn.setPixelColor(X+std::round(y*qTan(gamma)),y+Y,Qt::red);
        drawn.setPixelColor(X+1+std::round(y*qTan(gamma)),y+Y,Qt::red);
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
    //qDebug()<<"In der Probe ist der Winkel "<<gamma<<"und trifft auf eine Steigung von "<<slope;
    if(nSample==nMedium){
        return gamma;
    }else if((nSample*qSin(qAtan(slope)+gamma))/nMedium<-1){
        return -1.0;
    }else if((nSample*qSin(qAtan(slope)+gamma))/nMedium>1){
        return 1.0;
    }else{
        //qDebug()<<"Der Austrittswinkel hinter der Oberfläche ist "<<qAsin((nSample*qSin(gamma + qAtan(slope)))/nMedium) - qAtan(slope);
        return qAsin((nSample*qSin(gamma + qAtan(slope)))/nMedium) - qAtan(slope);
    }
}

double surface_fitting_test::getValueForGaussDistribution(double gammaX, double gammaY, int x, int y, double sigma)
{
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


QVector<surface_fitting_test::newSurfaceInfo> surface_fitting_test::makeNewStructVector(QVector<QImage> imageListThinnedSurface, QVector<QImage> imageListRotSurface, QVector<QVector<entryPoint> > pointList)
{
    QVector<newSurfaceInfo> returnList(imageListThinnedSurface.size());
    for(int k=0; k<imageListThinnedSurface.size();k++){
        returnList[k].thinnedOut = imageListThinnedSurface[k];
        returnList[k].ePoints = pointList[k];
        returnList[k].rotSurface = imageListRotSurface[k];
    }
    return returnList;
}


int surface_fitting_test::newPropagateRayMultiThreaded(newSurfaceInfo &surfaceList)
{
    surfaceList.ePoints[0]={0,0,0,0,0};
    for(int x = 1;x<surfaceList.thinnedOut.width();x++){
        if(getFirstValueFromTopStatic(surfaceList.thinnedOut,x-1)!=0 && getFirstValueFromTopStatic(surfaceList.thinnedOut,x)!=0 && getFirstValueFromTopStatic(surfaceList.thinnedOut,x+1)!=0){ //dadurch werden die äußersten Punkte ignoriert
            surfaceList.ePoints[x].xEntry = x;
            if(usePoly){
                surfaceList.ePoints[x].slopeEntry = getPolySlopeAtEntryStatic(surfaceList.rotSurface,x,fitOrderMT,true)[0];
                surfaceList.ePoints[x].yEntry = getPolySlopeAtEntryStatic(surfaceList.rotSurface,x,fitOrderMT,true)[1];
                surfaceList.ePoints[x].gammaEntry = getExitAngleStatic(surfaceList.ePoints[x].slopeEntry,riMediumMT,riSampleMT);
            }else{
                QVector<double>bufVec = getSlopeAtEntryStatic(surfaceList.thinnedOut,x,slopeSigmaMT);
                double exitAngle = getExitAngleStatic(bufVec[1],riMediumMT,riSampleMT);
                surfaceList.ePoints[x].gammaEntry = exitAngle;
                surfaceList.ePoints[x].yEntry = bufVec[0];
                surfaceList.ePoints[x].slopeEntry = bufVec[1];
            }
            QVector<double> rayVector = parameterizeFromAngleStatic(x,surfaceList.ePoints[x].yEntry,surfaceList.ePoints[x].gammaEntry);
            surfaceList.ePoints[x].lengthEntry = 0;
            if(surfaceList.ePoints[x].gammaEntry>=0&&surfaceList.ePoints[x].gammaEntry<1.0){  //Strahl wird nach rechts abgelenkt
                int X=x;
                bool success = false;
                while(success == false && getFirstValueFromBottomStatic(surfaceList.thinnedOut,X+1)>0){
                    QVector<double> surfaceVector = parameterizeFromPointsStatic(X, getFirstValueFromBottomStatic(surfaceList.thinnedOut,X), X+1, getFirstValueFromBottomStatic(surfaceList.thinnedOut,X+1));
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
                        QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                        if(intersection[1]>=getFirstValueFromTopStatic(surfaceList.thinnedOut,X)&&intersection[1]<=getFirstValueFromBottomStatic(surfaceList.thinnedOut,X)){
                            //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                            surfaceList.ePoints[x].lengthEntry = intersection[2];
                            success = true;
                        }
                    }
                    while(success == false && X>=x){
                        QVector<double> surfaceVector = parameterizeFromPointsStatic(X-1, getFirstValueFromTopStatic(surfaceList.thinnedOut,X-1), X, getFirstValueFromTopStatic(surfaceList.thinnedOut,X));
                        QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                        if(intersection[0]>=X-1&&intersection[0]<X){
                            surfaceList.ePoints[x].lengthEntry = intersection[2];
                            //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                            success = true;
                        }else{
                            X--;
                        }
                    }
                }
            }else if(surfaceList.ePoints[x].gammaEntry<0&&surfaceList.ePoints[x].gammaEntry>-1.0){  //Strahl wird nach links abgelenkt
                int X=x;
                bool success = false;
                while(success == false && getFirstValueFromBottomStatic(surfaceList.thinnedOut,X-1) > 0){
                    QVector<double> surfaceVector = parameterizeFromPointsStatic(X, getFirstValueFromBottomStatic(surfaceList.thinnedOut,X), X-1, getFirstValueFromBottomStatic(surfaceList.thinnedOut,X-1));
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
                        QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                        if(intersection[1]>=getFirstValueFromTopStatic(surfaceList.thinnedOut,X)&&intersection[1]<=getFirstValueFromBottomStatic(surfaceList.thinnedOut,X)){
                            //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                            surfaceList.ePoints[x].lengthEntry = intersection[2];
                            success = true;
                        }
                    }
                    while(success == false && X<=x){
                        QVector<double> surfaceVector = parameterizeFromPointsStatic(X, getFirstValueFromTopStatic(surfaceList.thinnedOut,X), X+1, getFirstValueFromTopStatic(surfaceList.thinnedOut,X+1));
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


QVector<double> surface_fitting_test::getPolySlopeAtEntryQt(QImage &surface, int X, int order, bool applyWeighting)
{
    if(getFirstValueFromTop(surface,X)!=0){

        QVector<double> xValues;
        QVector<double> yValues;
        int realsize = 0;
        for( int x = X-slopeSigmaMT; x<=X+slopeSigmaMT;x++){
            if(getFirstValueFromTop(surface,x)!=0){
                QVector<int> aScan = fillVectorWithAscan(surface,x);
                double ariMi = getArithmicMiddle(aScan,getFirstValueFromTop(surface,x),slopePxlNoMT);
                xValues.append(x-X);
                yValues.append(ariMi);
                realsize++;
            }
        }
        QVector<QVector<double>> array=QVector<QVector<double>>(realsize);
        for(size_t i = 0; i < realsize; i++) {
            array[i] = QVector<double>(realsize);
            for(size_t j = 0; j < realsize; j++) {
                if(i==j){
                    if(applyWeighting){
                        double weighting = qPow(realsize-std::abs(xValues[i]),2)/10.0;
                        array[i][j] = weighting;
                    }else{
                        array[i][j] = 1;
                    }
                }else{
                    array[i][j] = 0.;
                }
            }
        }
        QVector<double> gamba = poly->getSlopeStaticQt(xValues,yValues,array,order);
        return gamba;
    }else{
        return {300,300};
    }
}

QVector<double> surface_fitting_test::getPolySlopeAtExitQt(QImage &surface, int X, int order, bool applyWeighting)
{
    if(getFirstValueFromBottom(surface,X)!=0){

        QVector<double> xValues;
        QVector<double> yValues;
        int realsize = 0;
        for( int x = X-slopeSigmaMT; x<=X+slopeSigmaMT;x++){
            if(getFirstValueFromBottom(surface,x)!=0){
                QVector<int> aScan = fillVectorWithAscan(surface,x);
                double ariMi = getArithmicMiddle(aScan,getFirstValueFromBottom(surface,x),-slopePxlNoMT);
                xValues.append(x-X);
                yValues.append(ariMi);
                realsize++;
            }
        }
        QVector<QVector<double>> array=QVector<QVector<double>>(realsize);
        for(size_t i = 0; i < realsize; i++) {
            array[i] = QVector<double>(realsize);
            for(size_t j = 0; j < realsize; j++) {
                if(i==j){
                    if(applyWeighting){
                        double weighting = qPow(realsize-std::abs(xValues[i]),2)/10.0;
                        array[i][j] = weighting;
                    }
                }else{
                    array[i][j] = 0.;
                }
            }
        }
        QVector<double> gamba = poly->getSlopeStaticQt(xValues,yValues,array, order);
        return gamba;
    }else{
        return {300,300};
    }
}

QVector<double> surface_fitting_test::getPolySlopeAtEntryStatic(QImage &surface, int X, int order, bool applyWeighting)
{
    if(getFirstValueFromTopStatic(surface,X)!=0){

        QVector<double> xValues;
        QVector<double> yValues;
        int realsize = 0;
        for( int x = X-slopeSigmaMT; x<=X+slopeSigmaMT;x++){
            if(getFirstValueFromTopStatic(surface,x)!=0){
                QVector<int> aScan = fillVectorWithAscanStatic(surface,x);
                double ariMi = getArithmicMiddleStatic(aScan,getFirstValueFromTopStatic(surface,x),slopePxlNoMT);
                xValues.append(x-X);
                yValues.append(ariMi);
                realsize++; //hier muss es anders laufen
            }
        }
        QVector<QVector<double>> array=QVector<QVector<double>>(realsize);
        for(size_t i = 0; i < realsize; i++) {
            array[i] = QVector<double>(realsize);
            for(size_t j = 0; j < realsize; j++) {
                if(i==j){

                    array[i][j] = 1;
                }else{
                    array[i][j] = 0.;
                }
            }
        }
        QVector<double> gamba = polyFit::getSlopeStaticQt(xValues,yValues,array, order);
        //qDebug()<<"Gamba?"<<gamba;
        //qDebug()<<yValues<< realsize<<gamba;
        return gamba;
    }else{
        return {300,300};
    }
}

int surface_fitting_test::getFirstValueFromTopStatic(QImage &image, int X)
{
    if(X<0||X>=image.width()){
        return 0;
    }
    for(int y = 0;y<image.height();y++){
        if(getColorStatic(image,X,y)!=0){
            return y;
        }
    }
    return 0;
}

int surface_fitting_test::getFirstValueFromBottomStatic(QImage &image, int X)
{
    if(X<0||X>=image.width()){
        return 0;
    }
    for(int y = image.height()-1;y>=0;y--){
        if(getColorStatic(image,X,y)!=0){
            return y;
        }
    }
    return 0;
}

int surface_fitting_test::getColorStatic(QImage &image, int x, int y)
{
    QColor color = image.pixelColor(x,y);
    return color.green();
}

QVector<int> surface_fitting_test::fillVectorWithAscanStatic(QImage image, int X)
{
    QVector<int> vec = QVector<int>(image.height());
    vec.fill(0,image.height());

    for(int i = 0;i<image.height();i++){
     vec[i]=getColorStatic(image,X,i);
    }
    return vec;
}

double surface_fitting_test::getArithmicMiddleStatic(QVector<int> vec, int base, int integral)
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


double surface_fitting_test::calculateRelativCameraOffset(double mediumRI, double yExit, double xExit, double angleExit)
{
    if(angleExit == -1){
        return -200;
    }else if(angleExit ==1){
        return 200;
    }
    //qDebug()<<"Versatz in der Probe "<<xExit;
    double radiusGlassCuvette = 9.0;
    double n2 = 1.523;
    double n3 = 1;
    double y1 = radiusGlassCuvette-yExit+mmPerPixelInReco*inputSurface.height()/2;
    double y2 = 2.0;
    double y3 = 73.0;
    double x1 = y1*qTan(angleExit);
    //qDebug()<<"Versatz hinter der Probe "<<x1;
    double x2 = y2*qTan(qAsin(qSin(angleExit)*mediumRI/n2));
    //qDebug()<<"Versatz im Glas "<<x2;
    double x3 = y3*qTan(qAsin(mediumRI*qSin(angleExit)));
    //qDebug()<<"Versatz hinter der Küvette "<<x3;
    return xExit+x1+x2+x3;
}

QVector<double> surface_fitting_test::getBackExitPointAndAngle(QVector<entryPoint> surfacePoints, int xEntry, double mediaRI, double samplRi)
{
    struct entryPoint myPoint = surfacePoints[xEntry];
    myPoint.yExit = 0;
    myPoint.lengthEntry = 0;
    myPoint.xExit = 0;
    if(myPoint.yEntry!=0){ //dadurch werden die äußersten Punkte ignoriert
        myPoint.gammaEntry = getExitAngle(myPoint.slopeEntry,mediaRI,samplRi);
        QVector<double> rayVector = parameterizeFromAngle(xEntry,myPoint.yEntry, myPoint.gammaEntry);
        myPoint.lengthEntry = 0;
        if( myPoint.gammaEntry>=0 && myPoint.gammaEntry<1.0){  //Strahl wird nach rechts abgelenkt
            int X=xEntry;
            bool success = false;
            while(success == false && surfacePoints[X+1].yExit>0){
                QVector<double> surfaceVector = parameterizeFromPoints(X, surfacePoints[X].yExit, X+1, surfacePoints[X+1].yExit);
                QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                if(intersection[0]>=X&&intersection[0]<X+1){
                    myPoint.lengthEntry = intersection[2];
                    myPoint.yExit = intersection[1];
                    success = true;
                }else{
                    X++;
                }
            }
            if(success == false){
                //qDebug()<<"Strahl wurde nach rechts abgelenkt und fand keine Rückseite bei: "<<q<<x;
                if(surfacePoints[X].yEntry!=surfacePoints[X].yExit){
                    QVector<double> surfaceVector = parameterizeFromPoints(X, surfacePoints[X].yEntry, X, surfacePoints[X].yExit);
                    QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                    if(intersection[1]>=surfacePoints[X].yEntry&&intersection[1]<=surfacePoints[X].yExit){
                        //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                        myPoint.lengthEntry = intersection[2];
                        myPoint.yExit = intersection[1];
                        success = true;
                    }
                }
                while(success == false && X>=xEntry){
                    QVector<double> surfaceVector = parameterizeFromPoints(X-1, surfacePoints[X-1].yEntry, X, surfacePoints[X].yEntry);
                    QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                    if(intersection[0]>=X-1&&intersection[0]<X){
                        myPoint.lengthEntry = intersection[2];
                        myPoint.yExit = intersection[1];
                        //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                        success = true;
                    }else{
                        X--;
                    }
                }
            }
        }else if(myPoint.gammaEntry<0&&myPoint.gammaEntry>-1.0){  //Strahl wird nach links abgelenkt
            int X=xEntry;
            bool success = false;
            while(success == false && surfacePoints[X-1].yExit > 0){
                QVector<double> surfaceVector = parameterizeFromPoints(X, surfacePoints[X].yExit, X-1, surfacePoints[X-1].yExit);
                QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                if(intersection[0]<=X&&intersection[0]>X-1){
                    myPoint.lengthEntry = intersection[2];
                    myPoint.yExit = intersection[1];
                    success = true;
                }else{
                    X--;
                }
            }
            if(success == false){
                //qDebug()<<"Strahl wurde nach links abgelenkt und fand keine Rückseite bei: "<<q<<x;
                if(surfacePoints[X].yEntry!=surfacePoints[X].yExit){
                    QVector<double> surfaceVector = parameterizeFromPoints(X, surfacePoints[X].yEntry, X, surfacePoints[X].yExit);
                    QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                    if(intersection[1]>=surfacePoints[X].yEntry&&intersection[1]<=surfacePoints[X].yExit){
                        //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                        myPoint.lengthEntry = intersection[2];
                        myPoint.yExit = intersection[1];
                        success = true;
                    }
                }
                while(success == false && X<=xEntry){
                    QVector<double> surfaceVector = parameterizeFromPoints(X, surfacePoints[X].yEntry, X+1, surfacePoints[X+1].yEntry);
                    QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                    if(intersection[0]>=X&&intersection[0]<X+1){
                        myPoint.lengthEntry = intersection[2];
                        myPoint.yExit = intersection[1];
                        //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                        success = true;
                    }else{
                        X++;
                    }
                }
            }
        }/*else if(myPoint.gammaEntry==1){
            //qDebug()<<"Strahl wurde totalreflektiert!";
            return {double(xEntry),0,1};
        }else{
            //qDebug()<<"Strahl wurde totalreflektiert!";
            return {double(xEntry),0,-1};
        }*/

        myPoint.xExitRelative = myPoint.lengthEntry*qSin(myPoint.gammaEntry);

        //myPoint.xExit = myPoint.lengthEntry*qSin(myPoint.gammaEntry);

        double exitSlope = surfacePoints[std::round(myPoint.xExitRelative+xEntry)].slopeExit;

        double backExitAngle = getExitAngleAtBack(exitSlope,samplRi,mediaRI,myPoint.gammaEntry);

        return {myPoint.xExitRelative, myPoint.yExit, backExitAngle};
    }else{
        qDebug()<<"Gab kein Y Entry";
        return {double(xEntry),0,2};
    }
}
QVector<double> surface_fitting_test::generateRefractionPattern(QVector<entryPoint> surfacePoints, double mediaRI, double sampleRI) //Nur für die Punkte, die auf die Probe treffen
{
    QVector<double> pattern(surfacePoints.size()+2);
    pattern[0]=-mmPerPixelInReco; //Punkt ganz links neben der Probe
    pattern.last()=(pattern.size()-2)*mmPerPixelInReco; //Punkt ganz rechts neben der Probe
    for(int x = 0; x<surfacePoints.size();x++){
        surfacePoints[x].gammaEntry = getExitAngle(surfacePoints[x].slopeEntry,mediaRI,sampleRI);
        QVector<double> rayVector = parameterizeFromAngleStatic(x,surfacePoints[x].yEntry,surfacePoints[x].gammaEntry);
        if(surfacePoints[x].gammaEntry>=0&&surfacePoints[x].gammaEntry<1.0){  //Strahl wird nach rechts abgelenkt
            int X=x;
            bool success = false;
            while(success == false && surfacePoints[X+1].yExit>0){ //kann sein, dass es hier abkackt, weil irgendwann kein X+1 mehr existiert
                QVector<double> surfaceVector = parameterizeFromPointsStatic(X, surfacePoints[X].yExit, X+1, surfacePoints[X+1].yExit);
                QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                if(intersection[0]>=X&&intersection[0]<X+1){
                    surfacePoints[x].lengthEntry = intersection[2];
                    surfacePoints[x].yExit = intersection[1];
                    success = true;
                }else{
                    X++;
                }
            }
            if(success == false){
                //qDebug()<<"Strahl wurde nach rechts abgelenkt und fand keine Rückseite bei: "<<k<<x;
                if(surfacePoints[X].yEntry!=surfacePoints[X].yExit){
                    QVector<double> surfaceVector = parameterizeFromPointsStatic(X, surfacePoints[X].yEntry, X, surfacePoints[X].yExit);
                    QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                    if(intersection[1]>=surfacePoints[X].yEntry&&intersection[1]<=surfacePoints[X].yExit){
                        //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                        surfacePoints[x].lengthEntry = intersection[2];
                        surfacePoints[x].yExit = intersection[1];
                        success = true;
                    }
                }
                while(success == false && X>=x){
                    QVector<double> surfaceVector = parameterizeFromPointsStatic(X-1, surfacePoints[X-1].yEntry, X, surfacePoints[X].yEntry);
                    QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                    if(intersection[0]>=X-1&&intersection[0]<X){
                        surfacePoints[x].lengthEntry = intersection[2];
                        surfacePoints[x].yExit = intersection[1];
                        //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                        success = true;
                    }else{
                        X--;
                    }
                }
            }
        }else if(surfacePoints[x].gammaEntry<0&&surfacePoints[x].gammaEntry>-1.0){  //Strahl wird nach links abgelenkt
            int X=x;
            bool success = false;
            while(success == false && surfacePoints[X-1].yExit > 0){ //hier auch problem mit nicht existierenden werten maybe
                QVector<double> surfaceVector = parameterizeFromPointsStatic(X, surfacePoints[X].yExit, X-1, surfacePoints[X-1].yExit);
                QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                if(intersection[0]<=X&&intersection[0]>X-1){
                    surfacePoints[x].lengthEntry = intersection[2];
                    surfacePoints[x].yExit = intersection[1];
                    success = true;
                }else{
                    X--;
                }
            }
            if(success == false){
                //qDebug()<<"Strahl wurde nach links abgelenkt und fand keine Rückseite bei: "<<k<<x;
                if(surfacePoints[X].yEntry!=surfacePoints[X].yExit){
                    QVector<double> surfaceVector = parameterizeFromPointsStatic(X, surfacePoints[X].yEntry, X, surfacePoints[X].yExit);
                    QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                    if(intersection[1]>=surfacePoints[X].yEntry&&intersection[1]<=surfacePoints[X].yExit){
                        //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                        surfacePoints[x].lengthEntry = intersection[2];
                        surfacePoints[x].yExit = intersection[1];
                        success = true;
                    }
                }
                while(success == false && X<=x){
                    QVector<double> surfaceVector = parameterizeFromPointsStatic(X, surfacePoints[X].yEntry, X+1, surfacePoints[X+1].yEntry);
                    QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                    if(intersection[0]>=X&&intersection[0]<X+1){
                        surfacePoints[x].lengthEntry = intersection[2];
                        surfacePoints[x].yExit = intersection[1];
                        //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                        success = true;
                    }else{
                        X++;
                    }
                }
            }
        }else{
            qDebug()<<"Strahl wurde total reflektiert! oder so"<<x;
        }

        if(surfacePoints[x].lengthEntry<0){
            surfacePoints[x].lengthEntry=0;
            qDebug()<<"irgendwas schiefgelaufen beim rayPropagate";
        }
        //jetzt noch den Teil mit der Rückseite berechnen
        surfacePoints[x].xExitRelative = surfacePoints[x].lengthEntry*qSin(surfacePoints[x].gammaEntry);
        surfacePoints[x].xExit = surfacePoints[x].xExitRelative+surfacePoints[x].xEntry;
        //qDebug()<<"Winkel nach Eintritt in die Probe:"<<surfacePoints[x].gammaEntry;

        //qDebug()<<"Relativer Versatz in X: "<< surfacePoints[x].xExitRelative<<"Beim xEntry von"<<surfacePoints[x].xEntry ;

        double exitSlope = surfacePoints[std::round(surfacePoints[x].xExit)].slopeExit;

        //qDebug()<<"Steigung am Austrittspunkt:" <<exitSlope;

        double backExitAngle = getExitAngleAtBack(exitSlope,sampleRI,mediaRI,surfacePoints[x].gammaEntry);

        //qDebug()<<"Winkel nach Austrittspunkt: " << backExitAngle;

        pattern[x+1] = calculateRelativCameraOffset(mediaRI,surfacePoints[x].yExit*mmPerPixelInReco,surfacePoints[x].xExit*mmPerPixelInReco,backExitAngle); //Mal wieder ein Problem mit dem Vorzeichen?

        //qDebug()<<"Versatz auf Kamera:"<<pattern[x+1];
    }


    return pattern;
}

QVector<refraction::imageMoments> surface_fitting_test::shortenMomentsList(QVector<refraction::imageMoments> moments)
{
    double offset = (std::abs(moments[3].momentX-moments[0].momentX))/4.0;
    int counter = 0;
    double grayThreshold = (moments[0].graySum + moments[1].graySum + moments[2].graySum)/3;
    while(std::abs(moments[1].momentX-moments[0].momentX)<(5*offset)){
        qDebug()<<"Bild links neben der Probe wurde entfernt";
        counter++;
        moments.remove(0);
    }
    qDebug()<<counter<<"Bilder wurden links entfernt";
    qDebug()<<"Grauwert des ersten Bildes: "<<moments[0].graySum;
    qDebug()<<"Grauwert des zweiten Bildes: "<<moments[1].graySum;
    counter = 0;
    int currSize = moments.size();
    while(std::abs(moments[currSize-1].momentX-moments[currSize-2].momentX)<(5*offset)){
        qDebug()<<"Bild rechts neben der Probe wurde entfernt";
        counter++;
        moments.remove(currSize-1);
        currSize--;
    }
    for(int i = 0; i<moments.size();i++){
        if(moments[i].graySum < grayThreshold/20 ){
            moments[i].momentX = -999;
            qDebug()<<"Bild Nummer "<<i<<"entfernt, weil der Grauwert zu niedrig war!"<<moments[i].graySum;
        }
    }
    qDebug()<<counter<<"Bilder wurden rechts entfernt";
    qDebug()<<"Grauwert des letzten Bildes: "<<moments[currSize-1].graySum;
    qDebug()<<"Grauwert des vorletzten Bildes: "<<moments[currSize-2].graySum;
    qDebug()<<"Bleiben noch folgende Anzahl Bilder übrig:"<<moments.size();
    return moments;
}

void surface_fitting_test::determineTeleError(QVector<refraction::imageMoments > moments)
{
    scanPointsCCD = moments.size();
    qDebug()<<scanPointsCCD;
    qDebug()<<"Hardgecodeter Offset Per Pixel auf Sensor: "<<mmPerPixelOnSensor;
    //Ablenkung in Probe und Ablenkung auf Sensor vergleichen
    firstIndexFromLeft = getFirstIndexFromLeft(inputSurface);
    firstIndexFromRight = getFirstIndexFromRight(inputSurface);
    sampleSizePixel = (firstIndexFromRight-firstIndexFromLeft);
    double relativeScanMovementInSample = sampleSizePixel*mmPerPixelInReco;
    qDebug()<<"Der Laser wurde in der Probe um "<<relativeScanMovementInSample<<" in x-Richtung bewegt";
    double relativeScanMovementOnSensor = std::abs(moments.last().momentX - moments[0].momentX);
    qDebug()<<"Der Laser wurde auf dem Sensor um "<<relativeScanMovementOnSensor<<" in x-Richtung bewegt";
    correctedSensorRatio = relativeScanMovementInSample/relativeScanMovementOnSensor;
    qDebug()<<"Echte Ratio aufm Sensor: "<<mmPerPixelOnSensor;
    qDebug()<<"Korrigierte Ratio aufm Sensor: "<<correctedSensorRatio;
}

QVector<refraction::imageMoments> surface_fitting_test::makeListRelativeAndScaled(QVector<refraction::imageMoments> moments)
{
    double sampleSizeInMm = sampleSizePixel*mmPerPixelInReco;
    qDebug()<<"Sample Size in MM "<<sampleSizeInMm;
    double offset = moments[0].momentX; //+std::abs(moments[0].momentX-moments.last().momentX)/double(moments.size());
    qDebug()<< "Offset des ersten Punkts "<<offset;
    for(int x = 0; x<moments.size();x++){
        moments[x].momentX = (moments[x].momentX - offset)*correctedSensorRatio - x*sampleSizeInMm/(moments.size()-1);
        moments[x].momentY =  moments[x].momentY*correctedSensorRatio;
        qDebug()<<" Relativer Versatz: " << moments[x].momentX;
    }
    return moments;
}

QVector<surface_fitting_test::entryPoint> surface_fitting_test::calculateAllSlopes(QImage surface) //hier läuft noch alles korrekt
{
    //alles auf "Pixelebene und der linke Punkt ist der Nullpunkt"
    QVector<entryPoint> list(sampleSizePixel);
    for(int x = 0; x<sampleSizePixel;x++){
        list[x].xEntry = x;
        list[x].yEntry = getPolySlopeAtEntryQt(surface,x+firstIndexFromLeft,fitOrder,true)[1];
        list[x].slopeEntry = getPolySlopeAtEntryQt(surface,x+firstIndexFromLeft,fitOrder,true)[0];
        list[x].yExit = getPolySlopeAtExitQt(surface,x+firstIndexFromLeft,fitOrder,true)[1];
        list[x].slopeExit = getPolySlopeAtExitQt(surface,x+firstIndexFromLeft,fitOrder,true)[0];
        //qDebug()<<"Punkt slope: "<<x << list[x].slopeExit << "yExit: " <<list[x].yExit;
    }
    return list;
}

double surface_fitting_test::determineRelativeCameraOffset(QVector<surface_fitting_test::entryPoint> surfacePoints, int xEntry, double mediumRI, double sampleRI)
{
    entryPoint mypoint = surfacePoints[xEntry];
    mypoint.yExit = 0;
    mypoint.lengthEntry = 0;
    mypoint.xExit = 0;
    mypoint.gammaEntry = getExitAngle(surfacePoints[xEntry].slopeEntry,mediumRI,sampleRI);
    QVector<double> rayVector = parameterizeFromAngleStatic(xEntry,surfacePoints[xEntry].yEntry,mypoint.gammaEntry);
    if(mypoint.gammaEntry>=0&&mypoint.gammaEntry<1.0){  //Strahl wird nach rechts abgelenkt
        int X=xEntry;
        bool success = false;
        while(success == false && surfacePoints[X+1].yExit>0){ //kann sein, dass es hier abkackt, weil irgendwann kein X+1 mehr existiert
            QVector<double> surfaceVector = parameterizeFromPointsStatic(X, surfacePoints[X].yExit, X+1, surfacePoints[X+1].yExit);
            QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
            if(intersection[0]>=X&&intersection[0]<X+1){
                mypoint.lengthEntry = intersection[2];
                mypoint.yExit = intersection[1];
                success = true;
            }else{
                X++;
            }
        }
        if(success == false){
            //qDebug()<<"Strahl wurde nach rechts abgelenkt und fand keine Rückseite bei: "<<k<<x;
            if(surfacePoints[X].yEntry!=surfacePoints[X].yExit){
                QVector<double> surfaceVector = parameterizeFromPointsStatic(X, surfacePoints[X].yEntry, X, surfacePoints[X].yExit);
                QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                if(intersection[1]>=surfacePoints[X].yEntry&&intersection[1]<=surfacePoints[X].yExit){
                    //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                    mypoint.lengthEntry = intersection[2];
                    mypoint.yExit = intersection[1];
                    success = true;
                }
            }
            while(success == false && X>=xEntry){
                QVector<double> surfaceVector = parameterizeFromPointsStatic(X-1, surfacePoints[X-1].yEntry, X, surfacePoints[X].yEntry);
                QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                if(intersection[0]>=X-1&&intersection[0]<X){
                    mypoint.lengthEntry = intersection[2];
                    mypoint.yExit = intersection[1];
                    //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                    success = true;
                }else{
                    X--;
                }
            }
        }
    }else if(mypoint.gammaEntry<0&&mypoint.gammaEntry>-1.0){  //Strahl wird nach links abgelenkt
        int X=xEntry;
        bool success = false;
        while(success == false && surfacePoints[X-1].yExit > 0){ //hier auch problem mit nicht existierenden werten maybe
            QVector<double> surfaceVector = parameterizeFromPointsStatic(X, surfacePoints[X].yExit, X-1, surfacePoints[X-1].yExit);
            QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
            if(intersection[0]<=X&&intersection[0]>X-1){
                mypoint.lengthEntry = intersection[2];
                mypoint.yExit = intersection[1];
                success = true;
            }else{
                X--;
            }
        }
        if(success == false){
            //qDebug()<<"Strahl wurde nach links abgelenkt und fand keine Rückseite bei: "<<k<<x;
            if(surfacePoints[X].yEntry!=surfacePoints[X].yExit){
                QVector<double> surfaceVector = parameterizeFromPointsStatic(X, surfacePoints[X].yEntry, X, surfacePoints[X].yExit);
                QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                if(intersection[1]>=surfacePoints[X].yEntry&&intersection[1]<=surfacePoints[X].yExit){
                    //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                    mypoint.lengthEntry = intersection[2];
                    mypoint.yExit = intersection[1];
                    success = true;
                }
            }
            while(success == false && X<=xEntry){
                QVector<double> surfaceVector = parameterizeFromPointsStatic(X, surfacePoints[X].yEntry, X+1, surfacePoints[X+1].yEntry);
                QVector<double> intersection = getIntersectionPointStatic(rayVector,surfaceVector);
                if(intersection[0]>=X&&intersection[0]<X+1){
                    mypoint.lengthEntry = intersection[2];
                    mypoint.yExit = intersection[1];
                    //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                    success = true;
                }else{
                    X++;
                }
            }
        }
    }else{
        //qDebug()<<"Strahl wurde total reflektiert! oder so"<<xEntry;
        return -300;
    }

    //jetzt noch den Teil mit der Rückseite berechnen
    mypoint.xExitRelative = mypoint.lengthEntry*qSin(mypoint.gammaEntry);
    mypoint.xExit = mypoint.xExitRelative+mypoint.xEntry;
    //qDebug()<<"Winkel nach Eintritt in die Probe:"<<mypoint.gammaEntry;

    //qDebug()<<"Relativer Versatz in X: "<< surfacePoints[x].xExitRelative<<"Beim xEntry von"<<surfacePoints[x].xEntry ;

    double exitSlope = surfacePoints[std::round(mypoint.xExitRelative+mypoint.xEntry)].slopeExit;

    //qDebug()<<"Steigung am Austrittspunkt:" <<exitSlope << mypoint.xExit;

    double backExitAngle = getExitAngleAtBack(exitSlope,sampleRI,mediumRI,mypoint.gammaEntry);

    //qDebug()<<"Winkel nach Austrittspunkt: " << backExitAngle;

    double cameraOffset = calculateRelativCameraOffset(mediumRI,mypoint.yExit*mmPerPixelInReco,mypoint.xExitRelative*mmPerPixelInReco,backExitAngle); //Mal wieder ein Problem mit dem Vorzeichen?

    return cameraOffset;
}

QVector<double> surface_fitting_test::getFittingSampleRI(QVector<surface_fitting_test::entryPoint> surfacePoints, int xEntry, double relativeCameraOffset, double mediumRI, double expSampleRI, double riRange, double riIncriment, double excaptableOffset)
{
    double currTestRi = expSampleRI - riRange;
    double smallestDeviation = 500;
    double fittingRi = 333;
    while(currTestRi <=expSampleRI+riRange){
        double simulatedCameraPoint = determineRelativeCameraOffset(surfacePoints,xEntry,mediumRI,currTestRi);
//        qDebug()<<"Simulated CameraPoint: "<<simulatedCameraPoint;
//        qDebug()<<"relative Camera Offset: "<<relativeCameraOffset;
//        qDebug()<<"Absolut Difference: "<<std::abs(relativeCameraOffset-simulatedCameraPoint);
        if(std::abs(relativeCameraOffset-simulatedCameraPoint)<smallestDeviation){
            fittingRi = currTestRi;
            smallestDeviation = std::abs(relativeCameraOffset-simulatedCameraPoint);
        }
        currTestRi = currTestRi + riIncriment;
    }
//    while(currTestRi <=expectedSampleRI+riRange){
//        QVector<double> exit = getBackExitPointAndAngle(surfacePointsRefraction,xEntry,mediaRI,currTestRi); // alles in pixel; Ich glaube, der xEntry sollte nicht auf den exit aufaddiert werden
//        //qDebug()<<"Ausgangpunkt aus Probe in mm "<<mmPerPixelInReco*exit[1];
//        if(exit[2]>-1&&exit[2]<1){
//            double cameraPoint =calculateRelativCameraOffset(mediaRI,mmPerPixelInReco*exit[1],mmPerPixelInReco*exit[0],exit[2]);
//            if(std::abs(cameraPoint-xCameraPoint)<smallestDeviation){
//                fittingRi = currTestRi;
//                smallestDeviation = std::abs(cameraPoint-xCameraPoint);
//            }
//        }
//        currTestRi = currTestRi + riIncriment;
//    }
    if(smallestDeviation<excaptableOffset){
        qDebug()<<"Für inputX "<<xEntry<<"wurde folgender refractive index gefunden: "<<fittingRi<<"Bei einem smallest Deviation von " <<smallestDeviation;
        return {fittingRi,smallestDeviation};
    }else{
        return {333,0};
    }
}


void surface_fitting_test::saveInfoFile(QString name, QString savepath)
{
    QTime now = QTime::currentTime();
    if(!savepath.endsWith("/")){
        savepath.append("/");
    }
    QString timestring = "hh_mm";
    name.append("_").append(now.toString(timestring));
    QFile file(savepath + name + ".txt");
    if(!file.open(QIODevice::ReadWrite)) {
        qCritical()<<"Could not open file!";
        qCritical()<<file.errorString();
        return;
    }

    qInfo()<<"Writing Info file...";

    QTextStream stream(&file);

    //stream << "Das input Surface wurde geladen aus: "<<inputPathSurface<<"\n";
    if(fileListInfoSurface.size()!=0){
        stream << "Das input Surface wurde geladen aus: "<<fileListInfoSurface[0].absoluteFilePath()<<"\n";
    }else{
        stream << "Das input Surface wurde geladen aus: "<<inputPathSurface<<"\n";
    }
    stream << "Die Ordnung des Fits für die Oberfläche betrug: "<<fitOrder<<"\n";
    stream << "Die Anzahl der seitlich berücksichtigten Nachbarn für den Fit betrug: "<<slopeSigmaMT<<"\n";
    stream << "Für die Arithmetischen Mittel wurden so viele Folgepunkte berücksichtigt: "<<arithMiddleSigma<<"\n";
    stream << "Der Brechungsindex der Probe war: " << riSample <<"\n";
    stream << "Der Brechungsindex des Mediums war: " <<riMedium<<"\n";
    stream << "Anzahl der simulierten Projektionen: " << numberOfProjections <<"\n";
    if(useArrayFire){
        stream << "Es wurde ArrayFire für die Rotation der Bilder verwendent. \n";
    }
    if(useMultiThreading){
        stream << "Es wurde Multithreading für die Propagation der Strahlen verwendent. \n";
    }
    //stream << "" << <<"\n";

    file.close();
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

void surface_fitting_test::getMomentsList(QVector<refraction::imageMoments> moments)
{
    momentsList = moments;
    qDebug()<<"Liste der Momente erhalten. Erster Grauwert: "<<momentsList[0].graySum;
}

void surface_fitting_test::on_spinBox_aScan_valueChanged(int arg1)
{
    arithMiddleSigma = ui->spinBox_ariMiddleSigma->value();
    on_spinBox_rotateDisplayImageBy_valueChanged(ui->spinBox_rotateDisplayImageBy->value());
    QVector<double> slope{0,0};
    QVector<double> polySlope = getPolySlopeAtEntryQt(bufferImg,arg1,fitOrder, true);
    if(slope[1]!=999){
        //qDebug()<<"slope at"<<ui->spinBox_aScan->value()<<"is: "<< slope;
        qDebug()<<"polySlope at"<<ui->spinBox_aScan->value()<<"is: "<< polySlope;
        drawAndDisplaySlope(bufferImg,arg1,std::round(polySlope[1]),polySlope[0],20);
    }
}

void surface_fitting_test::on_spinBox_slopeSigma_valueChanged(int arg1)
{    
    QVector<double> slope{0,0};
    slope = getPolySlopeAtEntryQt(bufferImg,ui->spinBox_aScan->value(),fitOrder, true);
    if(slope[1]!=999){
        //qDebug()<<"slope at"<<ui->spinBox_aScan->value()<<"is: "<< slope;
        drawAndDisplaySlope(bufferImg,ui->spinBox_aScan->value(),slope[1],slope[0],20);
    }
    slopeSigmaMT = arg1;
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
    }
    poly = new polyFit();
    poly->deleteLater();
    riMediumMT = riMedium;
    riSampleMT = riSample;
    QElapsedTimer externalCorrectionTimer;
    externalCorrectionTimer.start();
    slopeSigmaMT = ui->spinBox_slopeSigma->value();
    slopePxlNoMT = ui->spinBox_ariMiddleSigma->value();
    QVector<surfaceInfo> surfaceList;
    QVector<newSurfaceInfo> newSurfaceList;
    bool createBScans = ui->checkBox_createBScans->isChecked();
    createTransmission = ui->checkBox_createTransmission->isChecked();
    sinogramHisto = QImage(inputSurface.width(),numberOfProjections+2,QImage::Format_Grayscale16);
    sinogramHisto.fill(0);
    transmissionHisto = sinogramHisto;
    newRotatedEntryPoints = QVector<QVector<entryPoint>>(numberOfProjections);
    if(useMultiThreading){
        for(int k = 0;k<numberOfProjections;k++){
            newRotatedEntryPoints[k] = QVector<entryPoint>(rotatedSurfacesThinnedOut[0].width());
        }
        newSurfaceList = makeNewStructVector(rotatedSurfacesThinnedOut,rotatedSurfaces,newRotatedEntryPoints);
        QtConcurrent::blockingMap(newSurfaceList,&surface_fitting_test::newPropagateRayMultiThreaded);
        qDebug()<<"Strahlen wurden MT propagiert"<<externalCorrectionTimer.restart();
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

                double gray = 0.15*propagateRayThroughHisto(rotatedHistoImages[q],x,newRotatedEntryPoints[q][x].yEntry,newRotatedEntryPoints[q][x].gammaEntry,newRotatedEntryPoints[q][x].lengthEntry);
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
            newRotatedEntryPoints[q] = QVector<entryPoint>(inputSurface.width());
            newRotatedEntryPoints[q][0].xEntry = 0;
            for(int x = 1;x<inputSurface.width();x++){
                newRotatedEntryPoints[q][x].xEntry = x;
                if(createTransmission){
                    if(getFirstValueFromTop(rotatedSurfacesThinnedOut[q],x)==0){
                        dstTrans[x]= int(65535);
                    }
                }
                if(getFirstValueFromTop(rotatedSurfacesThinnedOut[q],x-1)!=0 && getFirstValueFromTop(rotatedSurfacesThinnedOut[q],x)!=0 && getFirstValueFromTop(rotatedSurfacesThinnedOut[q],x+1)!=0){ //dadurch werden die äußersten Punkte ignoriert
                                        //polyfit option einfügen!!
                    if(usePolyFit){
                        QVector<double> slopeVec = getPolySlopeAtEntryQt(rotatedSurfaces[q],x,fitOrder,true);
                        newRotatedEntryPoints[q][x].slopeEntry = slopeVec[0];
                        newRotatedEntryPoints[q][x].yEntry = slopeVec[1];
                        newRotatedEntryPoints[q][x].gammaEntry = getExitAngle(newRotatedEntryPoints[q][x].slopeEntry,riMedium,riSample);
                    }else{
                        QVector<double>bufVec = getSlopeAtEntry(rotatedSurfacesThinnedOut[q],x,ui->spinBox_slopeSigma->value());
                        double exitAngle = getExitAngle(bufVec[1],riMedium,riSample);
                        newRotatedEntryPoints[q][x].yEntry = bufVec[0];
                        newRotatedEntryPoints[q][x].gammaEntry = exitAngle;
                        newRotatedEntryPoints[q][x].slopeEntry = bufVec[1];
                    }
                    QVector<double> rayVector = parameterizeFromAngle(x,newRotatedEntryPoints[q][x].yEntry,newRotatedEntryPoints[q][x].gammaEntry);
                    if(newRotatedEntryPoints[q][x].gammaEntry>=0&&newRotatedEntryPoints[q][x].gammaEntry<1.0){  //Strahl wird nach rechts abgelenkt
                        int X=x;
                        bool success = false;
                        while(success == false && getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X+1)>0){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X), X+1, getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X+1));
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[0]>=X&&intersection[0]<X+1){
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
                                QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                                if(intersection[1]>=getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X)&&intersection[1]<=getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X)){
                                    //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                                    newRotatedEntryPoints[q][x].lengthEntry = intersection[2];
                                    success = true;
                                }
                            }
                            while(success == false && X>=x){
                                QVector<double> surfaceVector = parameterizeFromPoints(X-1, getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X-1), X, getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X));
                                QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                                if(intersection[0]>=X-1&&intersection[0]<X){
                                    newRotatedEntryPoints[q][x].lengthEntry = intersection[2];
                                    //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                                    success = true;
                                }else{
                                    X--;
                                }
                            }
                        }
                    }else if(newRotatedEntryPoints[q][x].gammaEntry<0&&newRotatedEntryPoints[q][x].gammaEntry>-1.0){  //Strahl wird nach links abgelenkt
                        int X=x;
                        bool success = false;
                        while(success == false && getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X-1) > 0){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X), X-1, getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X-1));
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[0]<=X&&intersection[0]>X-1){
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
                                QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                                if(intersection[1]>=getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X)&&intersection[1]<=getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X)){
                                    //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                                    newRotatedEntryPoints[q][x].lengthEntry = intersection[2];
                                    success = true;
                                }
                            }
                            while(success == false && X<=x){
                                QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X), X+1, getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X+1));
                                QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                                if(intersection[0]>=X&&intersection[0]<X+1){
                                    newRotatedEntryPoints[q][x].lengthEntry = intersection[2];
                                    //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                                    success = true;
                                }else{
                                    X++;
                                }
                            }
                        }
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
    if(!continuousSimulation){
        saveInfoFile(nameRI+nrOfProjString+"_sinogramFromHisto.png",inputPathSurface);
    }

    qDebug()<<"jo, fertig mit "<< nameRI+nrOfProjString+"_sinogramFromHisto.png"<<externalCorrectionTimer.elapsed();
}


void surface_fitting_test::on_doubleSpinBox_riMedium_valueChanged(double arg1)
{
    QVector<double> slope{0,0};
    riMedium = arg1;
    riMediumMT = riMedium;
    slope = getPolySlopeAtEntryQt(bufferImg,ui->spinBox_aScan->value(),fitOrder,true);
    if(slope[1]!=999){
        //qDebug()<<"slope at"<<ui->spinBox_aScan->value()<<"is: "<< slope;
        drawAndDisplaySlope(bufferImg,ui->spinBox_aScan->value(),slope[1],slope[0],20);
    }
}


void surface_fitting_test::on_doubleSpinBox_riSample_valueChanged(double arg1)
{
    QVector<double> slope{0,0};
    riSample = arg1;
    riSampleMT = riSample;
    slope = getPolySlopeAtEntryQt(bufferImg,ui->spinBox_aScan->value(),fitOrder,true);
    if(slope[1]!=999){
        //qDebug()<<"slope at"<<ui->spinBox_aScan->value()<<"is: "<< slope;
        drawAndDisplaySlope(bufferImg,ui->spinBox_aScan->value(),slope[1],slope[0],20);
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
    //on_spinBox_aScan_valueChanged(ui->spinBox_aScan->value());
    QVector<double> slopevec = getPolySlopeAtEntryQt(bufferImg,ui->spinBox_aScan->value(),fitOrder,true);
    drawAndDisplaySlope(bufferImg,ui->spinBox_aScan->value(),slopevec[1],slopevec[0],20);
}

void surface_fitting_test::on_pushButton_testMath_clicked()
{
    firstIndexFromLeft = getFirstIndexFromLeft(inputSurface);
    firstIndexFromRight = getFirstIndexFromRight(inputSurface);
    sampleSizePixel = firstIndexFromRight-firstIndexFromLeft;

    surfacePointsRefraction = calculateAllSlopes(inputSurface);

//    QVector<double> pattern = generateRefractionPattern(surfacePointsRefraction,riMedium,riSample);
//    momentsList.clear();
//    momentsList.resize(pattern.size());
//    for(int i = 0; i<pattern.size();i++){
//        momentsList[i].momentX=pattern[i]*3.5+20;
//        momentsList[i].graySum = 100;
//    }qDebug()<<"Das generierte Pattern ist:" << pattern;


    momentsList = shortenMomentsList(momentsList);
    determineTeleError(momentsList);
    momentsList = makeListRelativeAndScaled(momentsList);
    QVector<double> fittedRI(momentsList.size());
    //hier muss der äußere Wert links und rechts im Surface-Bild bestimmt werden, der auf die Probe trifft

    qDebug()<<"Index links: "<<firstIndexFromLeft<<"Index rechts: "<<firstIndexFromRight;
    for(int i = 0; i<momentsList.size();i++){
        qDebug()<<"Moment ist: "<<momentsList[i].momentX;
    }
//    deltaPixel = (firstIndexFromRight-firstIndexFromLeft)/double(momentsList.size()-3);
//    qDebug()<<"Delta pixel beträgt:"<<deltaPixel;
//    //dann muss die Spanne dieser Werte durch die momentsList.size-2 geteilt werden, damit man den
//    for (int x = 1; x<momentsList.size()-1;x++){
//        fittedRI[x] = getFittingSampleRI(inputSurface,firstIndexFromLeft+std::round((x-1)*deltaPixel),momentsList[x].momentX,ui->doubleSpinBox_riMedium->value(),ui->doubleSpinBox_riSample->value(),0.02,0.001,2);
//        qDebug()<<"Fitted RI für Strahl "<<x<<fittedRI[x];
//        //qDebug()<<"Offset für diesen Strahl ist:"<<momentsList[x].momentX;
//        //qDebug()<<"Pixel im Bild: " << firstIndexFromLeft + std::round((x-1)*deltaPixel);
//    }


    for(int x = 0;x<momentsList.size();x++){
        QVector<double> buf = getFittingSampleRI(surfacePointsRefraction,4*x,momentsList[x].momentX,riMedium,riSample,riSearchRange,riSearchIncriment,riAcceptableOffset);
        fittedRI[x]= buf[0];
    }
    qDebug()<<"Punkte ohne Oberfläche entfernt "<<fittedRI.removeAll(999);
    fittedRI.removeAll(333);
    fittedRI.removeAll(777);
    fittedRI.removeAll(0);
    qDebug()<<fittedRI;
    double sum=0;
    for(int i = 0; i<fittedRI.size();i++){
        sum = sum + fittedRI[i];
    }
    qDebug()<<"Durchschnittlicher Wert für RI Sample ist: "<<sum/fittedRI.size();

    //    QElapsedTimer time;
    //    time.start();
//    poly = new polyFit();
//    QElapsedTimer time;
//    time.start();
//    for(int i = 0; i<inputSurface.width();i++){
//        if(getFirstValueFromTop(inputSurface,i)!=0){
//            qDebug()<<"Nummer des AScans: "<<i;
//            qDebug()<<"Fit mit Gewichtung:"<<getPolySlopeAtEntryQt(inputSurface,i,fitOrder,true);
//            qDebug()<<"Fit ohne Gewichtung:"<<getPolySlopeAtEntryQt(inputSurface,i,fitOrder,false);
//        }
//    }
//    qDebug()<<"Fit zweiter ordnung hat gedauert: "<<time.elapsed();

    //    getPolySlopeAtEntry(rotatedSurfaces[0],ui->spinBox_aScan->value());
    //    getPolySlopeAtExit(rotatedSurfaces[0],ui->spinBox_aScan->value());
    //    qDebug()<<"Fit hat gedauert: "<<time.elapsed();
        //generateRefractionPattern(inputSurface,riMedium,riSample);

    //saveInfoFile("dings",inputPathSurface);
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
    rotatedSurfaces = QVector<QImage>(numberOfProjections);
    if(useArrayFire){
        qDebug()<<"Surfaces rotated"<<bigTimer.elapsed();
        rotatedSurfaces = makeRotatedImageStack(inputPathSurface,numberOfProjections);
        rotatedSurfacesThinnedOut = rotatedSurfaces;
        if(useMultiThreading){
            QtConcurrent::blockingMap(rotatedSurfacesThinnedOut,&threadBoi::thinOutSurfaceThreaded);
        }else{
            for(int k = 0; k<numberOfProjections;k++){
                rotatedSurfacesThinnedOut[k]=thinOutSurface(rotatedSurfaces[k]);
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
            rotatedSurfaces[k] = QImage(inputSurface.size(),QImage::Format_Grayscale8);
            rotatedSurfaces[k].fill(0);
            for(int x = 0;x<inputSurface.width();x++){
                for(int y = 0;y<inputSurface.height();y++){
                    if(getColor(inputSurface,x,y)!=0){
                        rotatedPoint=rotateImagePointTo({double(x),double(y)},{double(inputSurface.width()/2.0),double(inputSurface.height()/2.0)},rotateClockwise*rotateBy,true);
                        int xRound = std::round(rotatedPoint[0]);
                        int yRound = std::round(rotatedPoint[1]);
                        for(int xGauss = xRound-2;xGauss<=xRound+2;xGauss++){
                            for(int yGauss = yRound-2;yGauss<=yRound+2;yGauss++){
                                int newInt = getColor(rotatedSurfaces[k],xGauss,yGauss) + int(200.0*getValueForGaussDistribution(rotatedPoint[0],rotatedPoint[1],xGauss,yGauss,1));
                                if(newInt>255){
                                    newInt=255;
                                    qDebug()<<"Grauwert zu hoch";
                                }
                                quint8 *dstRotSurface = (quint8*)(rotatedSurfaces[k].bits()+yGauss*rotatedSurfaces[k].bytesPerLine());
                                dstRotSurface[xGauss] = newInt;
                            }
                        }
                    }
                }
            }
            rotatedSurfacesThinnedOut[k]=thinOutSurface(rotatedSurfaces[k]);
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
    continuousSimulation = true;
    saveInfoFile("KontiSimulation",inputPathSurface);
    while(riMedium<=ui->doubleSpinBox_endRiMedium->value()){
        on_pushButton_createSinogram_clicked();
        on_pushButton_newCorrection_clicked();
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
    const QString folderpathSurfaceStack = QFileDialog::getExistingDirectory(this,tr("Surface Folder"),"G:/mSLOT/Mosqito2/");
    QDir dir(folderpathSurfaceStack);
    fileListInfoSurface = dir.entryInfoList();

    fileListInfoSurface.removeFirst();
    fileListInfoSurface.removeFirst();
    qDebug()<<fileListInfoSurface;
}


void surface_fitting_test::on_pushButton_ChooseSinogramDirectory_clicked()
{
    const QString folderpathSurfaceStack = QFileDialog::getExistingDirectory(this,tr("Sinogram PMT Folder"),fileListInfoSurface[0].absoluteFilePath());
    QDir dir(folderpathSurfaceStack);
    fileListInfoSinogramPMT = dir.entryInfoList();
    fileListInfoSinogramPMT.removeFirst();
    fileListInfoSinogramPMT.removeFirst();
}


void surface_fitting_test::on_pushButton_correctStack_clicked()
{
//    riMedium = ui->doubleSpinBox_riMedium->value();
//    riSample = ui->doubleSpinBox_riSample->value();
//    arithMiddleSigma = ui->spinBox_ariMiddleSigma->value();
//    const QString folderpathSave = QFileDialog::getExistingDirectory(this,tr("Surface Folder"),"E:/mSLOT/");
//    QDir saveDir(folderpathSave);
//    QString savePath = saveDir.absolutePath();
//    savePath.append("/");
//    qDebug()<<savePath;
//    saveInfoFile("Korrektur_von_externem_Stack",savePath);
//    if(fileListInfoSinogramPMT.size()==fileListInfoSurface.size()){
//        int stackSize = fileListInfoSinogramPMT.size();
//        for ( int i = 0; i<stackSize;i++){
//            QImage bufSinogramPMT, bufSurface, bufSinogramPD;
//            if(bufSinogramPMT.load(fileListInfoSinogramPMT[i].absoluteFilePath())&&bufSurface.load(fileListInfoSurface[i].absoluteFilePath())){
//                numberOfProjections = bufSinogramPMT.height()-2;
//                QImage correctedSinogram = correctExternalSinogram(bufSinogramPMT,bufSurface, fileListInfoSurface[i].absoluteFilePath(),riMedium,riSample);
//                QString name = "korrigiertesSinogram_";
//                name.append(QString::number(i)).append(".png");
//                correctedSinogram.save(savePath + name);
//            }else{
//                qDebug()<<"Loading images Failed!";
//            }
//        }
//    }else{
//        qDebug()<<"Number of Surface and Sinograms did not match";
//    }
    riMedium = ui->doubleSpinBox_riMedium->value();
    riSample = ui->doubleSpinBox_riSample->value();
    arithMiddleSigma = ui->spinBox_ariMiddleSigma->value();
    const QString folderpathSavePMT = QFileDialog::getExistingDirectory(this,tr("Save Folder for PMT"),fileListInfoSurface[0].absoluteFilePath());
    QDir saveDirPMT(folderpathSavePMT);
    QString savePathPMT = saveDirPMT.absolutePath();
    savePathPMT.append("/");
    qDebug()<<savePathPMT;
    const QString folderpathSavePD = QFileDialog::getExistingDirectory(this,tr("Save Folder for PD"),fileListInfoSurface[0].absoluteFilePath());
    QDir saveDirPD(folderpathSavePD);
    QString savePathPD = saveDirPD.absolutePath();
    savePathPD.append("/");
    qDebug()<<savePathPD;
    saveInfoFile("Korrektur_von_externem_Stack",savePathPMT);
    if(fileListInfoSinogramPMT.size()==fileListInfoSurface.size()&&fileListInfoSurface.size()==fileListInfoSinogramPD.size()){
        int stackSize = fileListInfoSinogramPMT.size();
        for ( int i = 0; i<stackSize;i++){
            QImage bufSinogramPMT, bufSurface, bufSinogramPD;
            if(bufSinogramPMT.load(fileListInfoSinogramPMT[i].absoluteFilePath())&&bufSurface.load(fileListInfoSurface[i].absoluteFilePath())&&bufSinogramPD.load(fileListInfoSinogramPD[i].absoluteFilePath())){
                numberOfProjections = bufSinogramPMT.height()-2;
                QImage correctedSinogramPD = bufSinogramPD;
                correctedSinogramPD.fill(0);
                QImage correctedSinogramPMT = correctedSinogramPD;
                correctExternalSinogramPDandPMT(bufSinogramPD,bufSinogramPMT,correctedSinogramPD,correctedSinogramPMT,bufSurface,fileListInfoSurface[i].absoluteFilePath(),riMedium,riSample);
                QString namePMT = "korrigiertesSinogramPMT_";
                QString namePD = "korrigiertesSinogramPD_";
                namePMT.append(QString::number(i)).append(".png");
                namePD.append(QString::number(i)).append(".png");
                correctedSinogramPMT.save(savePathPMT + namePMT);
                correctedSinogramPD.save(savePathPD + namePD);
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


void surface_fitting_test::on_spinBox_ariMiddleSigma_valueChanged(int arg1)
{
    QVector<double> slope{0,0};
    arithMiddleSigma = arg1;
    slopePxlNoMT = arg1;
    on_spinBox_rotateDisplayImageBy_valueChanged(ui->spinBox_rotateDisplayImageBy->value());
    slope = getPolySlopeAtEntryQt(bufferImg,ui->spinBox_aScan->value(),fitOrder,true);
    if(slope[1]!=999){
        //qDebug()<<"slope at"<<ui->spinBox_aScan->value()<<"is: "<< slope;
        drawAndDisplaySlope(bufferImg,ui->spinBox_aScan->value(),slope[1],slope[0],20);
    }
}

void surface_fitting_test::on_checkBox_usePolyFit_stateChanged(int arg1)
{
    usePolyFit = arg1;
    usePoly = arg1;
}

void surface_fitting_test::on_spinBox_fitOrder_valueChanged(int arg1)
{
    fitOrder = arg1;
    fitOrderMT = arg1;
    on_spinBox_aScan_valueChanged(ui->spinBox_aScan->value());
}

void surface_fitting_test::on_pushButton_choosePdSinogram_clicked()
{
    const QString folderpathSurfaceStack = QFileDialog::getExistingDirectory(this,tr("Sinogram PD Folder"),fileListInfoSurface[0].absoluteFilePath());
    QDir dir(folderpathSurfaceStack);
    fileListInfoSinogramPD = dir.entryInfoList();
    fileListInfoSinogramPD.removeFirst();
    fileListInfoSinogramPD.removeFirst();
}

void surface_fitting_test::on_pushButton_analyseCameraSlot_clicked()
{
    if(inputSurface.width()==0){
        qDebug()<<"No reference Surface was loaded!!";
        return;
    }
    if(momentsList.size()==0){
        qDebug()<<"No Camera-SLOT Measurement was loaded!!";
        return;
    }

    riSearchRange = ui->doubleSpinBox_riRange->value();
    riSearchIncriment = ui->doubleSpinBox_searchIncriment->value();
    riAcceptableOffset = ui->doubleSpinBox_riAcceptableOffset->value();
    mmPerPixelInReco = ui->doubleSpinBox_mmPerPixelinReco->value();

    firstIndexFromLeft = getFirstIndexFromLeft(inputSurface);
    firstIndexFromRight = getFirstIndexFromRight(inputSurface);
    sampleSizePixel = firstIndexFromRight-firstIndexFromLeft;

    surfacePointsRefraction = calculateAllSlopes(inputSurface);

    QVector<refraction::imageMoments> momentsTemp = momentsList;
    momentsTemp = shortenMomentsList(momentsTemp);
    determineTeleError(momentsTemp);
    momentsTemp = makeListRelativeAndScaled(momentsTemp);
    QVector<double> fittedRI(momentsTemp.size());
    QVector<double> deviations(momentsTemp.size());
    //hier muss der äußere Wert links und rechts im Surface-Bild bestimmt werden, der auf die Probe trifft

    qDebug() << "SampleSize in Pixel: "<<sampleSizePixel<<"Momentslist.size: "<<momentsTemp.size() << "Faktor Pixel in Reco pro aufgenommenem Bild: "<<double(sampleSizePixel)/double(momentsTemp.size()-2);
    for(int x = 1;x<momentsTemp.size()-1;x++){
        int xInImage = std::round(double((x)*sampleSizePixel)/double(momentsTemp.size()-3)-double(sampleSizePixel)/double(momentsTemp.size()-2));
        qDebug()<<"xInImage: "<<xInImage << x;
        QVector<double> buf = getFittingSampleRI(surfacePointsRefraction,xInImage,momentsTemp[x].momentX,riMedium,riSample,riSearchRange,riSearchIncriment,riAcceptableOffset);
        fittedRI[x]= buf[0];
        deviations[x]=buf[1];
    }
    qDebug()<<"Punkte ohne Oberfläche entfernt "<<fittedRI.removeAll(999);
    fittedRI.removeAll(333);
    fittedRI.removeAll(777);
    fittedRI.removeAll(0);
    qDebug()<<fittedRI;
    double sum=0;
    double highestDeviation = 0;
    for(int i = 0; i<fittedRI.size();i++){
        sum = sum + fittedRI[i];
        if(deviations[i]>highestDeviation){
            highestDeviation=deviations[i];
        }
    }
    qDebug()<<"Durchschnittlicher Wert für RI Sample ist: "<<sum/fittedRI.size();
    qDebug()<<"Größte Abweichung war: " << highestDeviation;
}

void surface_fitting_test::on_doubleSpinBox_riRange_valueChanged(double arg1)
{
    riSearchRange = arg1;
}

void surface_fitting_test::on_doubleSpinBox_searchIncriment_valueChanged(double arg1)
{
    riSearchIncriment = arg1;
}

void surface_fitting_test::on_doubleSpinBox_riAcceptableOffset_valueChanged(double arg1)
{
    riAcceptableOffset = arg1;
}

void surface_fitting_test::on_doubleSpinBox_mmPerPixelinReco_valueChanged(double arg1)
{
    mmPerPixelInReco = arg1;
}

void surface_fitting_test::on_pushButton_saveCameraSlot_clicked()
{
    const QString folderpathSaveCCD = QFileDialog::getExistingDirectory(this,tr("Save Folder for CCD Analysis"),inputPathSurface);
    QDir saveDirCCD(folderpathSaveCCD);
    QString savePathCCD = saveDirCCD.absolutePath();
    savePathCCD.append("/");
    qDebug()<<savePathCCD;

    if(inputSurface.width()==0){
        qDebug()<<"No reference Surface was loaded!!";
        return;
    }
    if(momentsList.size()==0){
        qDebug()<<"No Camera-SLOT Measurement was loaded!!";
        return;
    }

    riSearchRange = ui->doubleSpinBox_riRange->value();
    riSearchIncriment = ui->doubleSpinBox_searchIncriment->value();
    riAcceptableOffset = ui->doubleSpinBox_riAcceptableOffset->value();
    mmPerPixelInReco = ui->doubleSpinBox_mmPerPixelinReco->value();

    firstIndexFromLeft = getFirstIndexFromLeft(inputSurface);
    firstIndexFromRight = getFirstIndexFromRight(inputSurface);
    sampleSizePixel = firstIndexFromRight-firstIndexFromLeft;

    surfacePointsRefraction = calculateAllSlopes(inputSurface);

    QVector<refraction::imageMoments> momentsTemp = momentsList;
    momentsTemp = shortenMomentsList(momentsTemp);
    determineTeleError(momentsTemp);
    momentsTemp = makeListRelativeAndScaled(momentsTemp);
    QVector<double> fittedRI(momentsTemp.size());
    QVector<double> deviations(momentsTemp.size());
    //hier muss der äußere Wert links und rechts im Surface-Bild bestimmt werden, der auf die Probe trifft

    qDebug() << "SampleSize in Pixel: "<<sampleSizePixel<<"Momentslist.size: "<<momentsTemp.size() << "Faktor Pixel in Reco pro aufgenommenem Bild: "<<double(sampleSizePixel)/double(momentsTemp.size()-2);
    for(int x = 1;x<momentsTemp.size()-1;x++){
        int xInImage = std::round(double((x)*sampleSizePixel)/double(momentsTemp.size()-3)-double(sampleSizePixel)/double(momentsTemp.size()-2));
        qDebug()<<"xInImage: "<<xInImage << x;
        QVector<double> buf = getFittingSampleRI(surfacePointsRefraction,xInImage,momentsTemp[x].momentX,riMedium,riSample,riSearchRange,riSearchIncriment,riAcceptableOffset);
        fittedRI[x]= buf[0];
    }
    QVector <double> fittedRiCopy = fittedRI;
    qDebug()<<"Punkte ohne Oberfläche entfernt "<<fittedRI.removeAll(999);
    fittedRI.removeAll(333);
    fittedRI.removeAll(777);
    fittedRI.removeAll(0);
    qDebug()<<fittedRI;
    double sum=0;
    for(int i = 0; i<fittedRI.size();i++){
        sum = sum + fittedRI[i];
        }

    qDebug()<<"Durchschnittlicher Wert für RI Sample ist: "<<sum/fittedRI.size();

    QTime now = QTime::currentTime();
    if(!savePathCCD.endsWith("/")){
        savePathCCD.append("/");
    }
    QString timestring = "hh_mm";
    QString name = "Camera_SLOT_Analysis_";
    name.append(ui->lineEdit_cameraSlotAnalyseName->text());
    name.append("_").append(now.toString(timestring));
    QFile file(savePathCCD + name + ".txt");
    if(!file.open(QIODevice::ReadWrite)) {
        qCritical()<<"Could not open file!";
        qCritical()<<file.errorString();
        return;
    }

    qInfo()<<"Writing Info file...";

    QTextStream stream(&file);

    //stream << "Das input Surface wurde geladen aus: "<<inputPathSurface<<"\n";
    if(fileListInfoSurface.size()!=0){
        stream << "Das input Surface wurde geladen aus: "<<fileListInfoSurface[0].absoluteFilePath()<<"\n";
    }else{
        stream << "Das input Surface wurde geladen aus: "<<inputPathSurface<<"\n";
    }
    stream << "Die Ordnung des Fits für die Oberfläche betrug: "<<fitOrder<<"\n";
    stream << "Die Anzahl der seitlich berücksichtigten Nachbarn für den Fit betrug: "<<slopeSigmaMT<<"\n";
    stream << "Für die Arithmetischen Mittel wurden so viele Folgepunkte beruecksichtigt: "<<arithMiddleSigma<<"\n";
    stream << "Der Brechungsindex des Mediums war: " <<riMedium<<"\n";
    stream << "Der Brechungsindex der Probe wurde gesucht um: " << riSample <<"\n";
    stream << "Der Brechungsindex wurde mit folgendem Sigma gesucht: " << riSearchRange<<"\n";
    stream << "Der Brechungsindex wurde mit folgender Schrittgroesse abgetastet: "<<riSearchIncriment<< "\n";
    stream << "Folgende Ungenauigkeit wurde akzeptiert: "<<riAcceptableOffset<<"\n";
    stream << "Folgende Brechungsindize wurden fuer jeweilige Bilder bestimmt: \n";
    stream << "Aufbau der Tabelle: \n";
    stream << "Gefitteter RI; Grauwert des Punktes; Relative Ablenkung in X; Massenschwerpunkt Y; Bildnummer;\n ";
    for(int i = 0;i<fittedRiCopy.size();i++){
        if(fittedRiCopy[i]>=riSample-riSearchRange&&fittedRiCopy[i]<=riSample+riSearchRange){
            stream << fittedRiCopy[i] << ";"<< momentsTemp[i].graySum << ";"<< momentsTemp[i].momentX<<";"<< momentsTemp[i].momentY<<";"<<i<<";\n";
        }else{
            stream << QString::number(0,1)<<";"<< momentsTemp[i].graySum <<";"<< momentsTemp[i].momentX<<";"<< momentsTemp[i].momentY<<";"<<i<<";\n";
        }

    }

    //stream << "" << <<"\n";

    file.close();
}
