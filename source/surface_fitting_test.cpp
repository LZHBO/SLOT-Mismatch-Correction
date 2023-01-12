#include "header/surface_fitting_test.h"
#include "ui_surface_fitting_test.h"

surface_fitting_test::surface_fitting_test(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::surface_fitting_test)
{
    ui->setupUi(this);
}

surface_fitting_test::~surface_fitting_test()
{
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
    if(!QDir(inputPathSurface).exists()){
        QDir().mkdir(inputPathSurface);
    }
    if(!QDir(inputPathSinogram).exists()){
        QDir().mkdir(inputPathSinogram);
    }
    bufferImg = QImage(inputSurface.size(),QImage::Format_Grayscale8);
    bufferImg = thinOutSurface(inputSurface);
    displayImageLeft(inputSurface);
    numberOfProjections = inputSinogram.height()-2;
    riMedium = ui->doubleSpinBox_riMedium->value();
    riSample = ui->doubleSpinBox_riSample->value();
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

QImage surface_fitting_test::setPixelColor8Bit(QImage image, int x, int y, int value)
{
    quint8 *dstColor = (quint8*)(image.bits()+y*image.bytesPerLine());
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
    QVector<double> d{yReturn,Sxy/Sxx};
    return d; // Vorzeichenwechsel scheint falsch herum, weil der Nullpunkt von Bildern oben links ist
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
    QVector<double> d{yReturn,Sxy/Sxx};
    return d; // Vorzeichenwechsel scheint falsch herum, weil der Nullpunkt von Bildern oben links ist
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
    for(int y = 0;y<image.height();y++){
        if(getColor(image,X,y)!=0){
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
    QImage output = QImage(image.size(),QImage::Format_Grayscale8);
    output.fill(0);
    for(int x = 0;x<image.width();x++){
        QVector<int> ascan = fillVectorWithAscan(image,x);
        int y=getFirstValueFromTop(image,x);
        if(y!=0){
            double middle = getArithmicMiddle(ascan,y,arithMiddleSigma);
            output = setPixelColor8Bit(output,x,std::round(middle),255);
        }
    }
    for(int x = 0;x<image.width();x++){
        QVector<int> ascan = fillVectorWithAscan(image,x);
        int y=getFirstValueFromBottom(image,x);
        if(y!=0){
            double middle = getArithmicMiddle(ascan,y,-arithMiddleSigma);
            output = setPixelColor8Bit(output,x,std::round(middle),255);
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

void surface_fitting_test::correctExternalSinogram(QImage sinogram, QImage surface, double mediumRI, double sampleRI)  //Beim Übergang von der letzten zur ersten Proj geht noch was schief... Grauwerte haben einen scheinbar festen Offset
{
    QElapsedTimer externalCorrectionTimer;
    externalCorrectionTimer.start();
    int slopeSigma = ui->spinBox_slopeSigma->value();
    int numberOfAngles = sinogram.height()-2;
    rotatedSurfacesThinnedOut = QVector<QImage>(numberOfAngles);
    rotatedEntryPoints = QVector<QVector<QVector<double>>>(numberOfAngles);
    for(int k = 0;k<numberOfAngles;k++){
        QVector<double>rotatedPoint=QVector<double>(2);
        double rotateBy = (2*M_PI/double(numberOfAngles))*k;
        rotatedSurfacesThinnedOut[k] = QImage(inputSurface.size(),QImage::Format_Grayscale8);
        rotatedSurfacesThinnedOut[k].fill(0);
        for(int x = 0;x<inputSurface.width();x++){
            for(int y = 0;y<inputSurface.height();y++){
                if(getColor(inputSurface,x,y)!=0){
                    rotatedPoint=rotateImagePointTo({double(x),double(y)},{double(inputSurface.width()/2.0),double(inputSurface.height()/2.0)},rotateBy,true);
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
        rotatedEntryPoints[k] = QVector<QVector<double>>(inputSurface.width());
        rotatedEntryPoints[k][0]={0,0,0,0};
        //Strahl durch die "Projektion" propagieren
        for(int x = 1;x<inputSurface.width();x++){
            rotatedEntryPoints[k][x]={0,0};
            if(getFirstValueFromTop(rotatedSurfacesThinnedOut[k],x-1)!=0 && getFirstValueFromTop(rotatedSurfacesThinnedOut[k],x)!=0 && getFirstValueFromTop(rotatedSurfacesThinnedOut[k],x+1)!=0){ //dadurch werden die äußersten Punkte ignoriert
                QVector<double>bufVec = getSlopeAtEntry(rotatedSurfacesThinnedOut[k],x,slopeSigma);
                double exitAngle = getExitAngle(bufVec[1],mediumRI,sampleRI);
                rotatedEntryPoints[k][x] = {bufVec[0],exitAngle};
                rotatedEntryPoints[k][x].append(0);
                if(exitAngle>=0&&exitAngle<1.0){  //Strahl wird nach rechts abgelenkt
                    int X=x;
                    bool success = false;
                    while(success == false && getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X+1)>0){
                        QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X), X+1, getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X+1));
                        QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                        QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                        if(intersection[0]>=X&&intersection[0]<X+1){
                            rotatedEntryPoints[k][x][2] = intersection[2];
                            success = true;
                        }else{
                            X++;
                        }
                    }
                    if(success == false){
                        qDebug()<<"Strahl wurde nach rechts abgelenkt und fand keine Rückseite bei: "<<k<<x;
                        if(getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X)!=getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X)){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X), X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[1]>=getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X)&&intersection[1]<=getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X)){
                                qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                                rotatedEntryPoints[k][x][2] = intersection[2];
                                success = true;
                            }
                        }
                        while(success == false && X>=x){
                            QVector<double> surfaceVector = parameterizeFromPoints(X-1, getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X-1), X, getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[0]>=X&&intersection[0]<X+1){
                                rotatedEntryPoints[k][x][2] = intersection[2];
                                qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
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
                            success = true;
                        }else{
                            X--;
                        }
                    }
                    if(success == false){
                        qDebug()<<"Strahl wurde nach links abgelenkt und fand keine Rückseite bei: "<<k<<x;
                        if(getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X)!=getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X)){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X), X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[1]>=getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X)&&intersection[1]<=getFirstValueFromBottom(rotatedSurfacesThinnedOut[k],X)){
                                //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                                rotatedEntryPoints[k][x][2] = intersection[2];
                                success = true;
                            }
                        }
                        while(success == false && X<=x){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X), X+1, getFirstValueFromTop(rotatedSurfacesThinnedOut[k],X+1));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[0]>=X&&intersection[0]<X+1){
                                rotatedEntryPoints[k][x][2] = intersection[2];
                                //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                                success = true;
                            }else{
                                X++;
                            }
                        }
                    }
                }else{
                    qDebug()<<"Strahl wurde total reflektiert!"<<k<<x;
                }

                rotatedEntryPoints[k][x].append(bufVec[1]);
                // Wert in ein Sinogram malen
                if(rotatedEntryPoints[k][x][2]<0){
                    rotatedEntryPoints[k][x][2]=0;
                    qDebug()<<"irgendwas schiefgelaufen beim rayPropagate";
                }

            }else{
                rotatedEntryPoints[k][x] = {0,0,0,0};
            }
        }
        qDebug()<<"Rays propagated for Input Surface "<<k<<"of "<<numberOfAngles<<", time: "<<externalCorrectionTimer.elapsed();
    }
    QElapsedTimer reTimer;
    reTimer.start();
    QImage rearrangedExternalSinogram = QImage(inputSinogram.size(),QImage::Format_Grayscale16);
    rearrangedSinogramFails = QImage(inputSinogram.size(),QImage::Format_RGB16);
    rearrangedSinogramFails.fill(0);
    rearrangedExternalSinogram.fill(0);
    QVector<QVector<int>> travelledOnBackside;
    for(int p = 0; p<numberOfAngles;p++){
        qDebug()<<"Correction started for: "<<p<<reTimer.elapsed();
        quint16 *dstArrSino = (quint16*)(rearrangedExternalSinogram.bits()+p*rearrangedExternalSinogram.bytesPerLine());
        for(int x = 0; x<inputSurface.width();x++){
            if(rotatedEntryPoints[p][x][0]!=0){
                rearrangedSinogramFails.setPixelColor(x,p,Qt::white);
                if(rotatedEntryPoints[p][x][1]==0){                                                          //bei senkrechten Strahlen wird nichts verschoben!
                    quint16 *dstSinoStraight = (quint16*)(inputSinogram.bits()+p*inputSinogram.bytesPerLine());
                    rearrangedSinogramFails.setPixelColor(x,p,Qt::gray);
                    dstArrSino[x]=dstSinoStraight[x];
                }else{
                    double deltaTheta = getDeltaThetaForPartner(rotatedEntryPoints[p][x][3],riMedium,riSample);
                    if(deltaTheta!=999){
                        double deltaProj = deltaTheta*double(numberOfAngles)/(2*M_PI);
                        QVector<double> buf = rotateImagePointTo({double(x),rotatedEntryPoints[p][x][0]},{double(inputSurface.width()/2),double(inputSurface.height()/2)},deltaTheta,true);
                        double absNewX = buf[0];
                        double value = newBilinInterpolFromSinogram(inputSinogram,absNewX,double(p)+deltaProj);
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
                        if(accountForReflection==true){
                            value = value/getTransmissionGrade(riMedium,riSample,rotatedEntryPoints[p][x]);
                        }
                        dstArrSino[x] = std::round(value);
                    }else{
                        travelledOnBackside.append({p,x});
                        rearrangedSinogramFails.setPixelColor(x,p,Qt::red);
                    }
                }

            }
        }
    } //Am Ende noch die letzten beiden Zeilen aus der ersten kopieren
    quint16 *dstArrSinoBot = (quint16*)(rearrangedExternalSinogram.bits()+numberOfAngles*rearrangedExternalSinogram.bytesPerLine());
    quint16 *dstArrSinoTop = (quint16*)(rearrangedExternalSinogram.bits());
    for(int x = 0;x<inputSinogram.width();x++){
        dstArrSinoBot[x] = dstArrSinoTop[x];
    }
    quint16 *dstArrSinoBot2 = (quint16*)(rearrangedExternalSinogram.bits()+(numberOfAngles+1)*rearrangedExternalSinogram.bytesPerLine());
    for(int x = 0;x<inputSinogram.width();x++){
        dstArrSinoBot2[x] = dstArrSinoTop[x];
    }
    for(int r = 0;r<travelledOnBackside.size();r++){ // Wenn Punkte bei Korrektur auf die Rückseite gewandert sind, wird hier der Wert "von der anderen Seite" kopiert
        int fromX = inputSinogram.width()-travelledOnBackside[r][1]-1;
        quint16 *dstArrSinoSource = (quint16*)(rearrangedExternalSinogram.bits()+((travelledOnBackside[r][0]+numberOfAngles/2)%numberOfAngles)*rearrangedExternalSinogram.bytesPerLine());
        quint16 *dstArrSinoFailed = (quint16*)(rearrangedExternalSinogram.bits()+(travelledOnBackside[r][0])*rearrangedExternalSinogram.bytesPerLine());
        if(dstArrSinoSource[fromX]!=0){
            rearrangedSinogramFails.setPixelColor(travelledOnBackside[r][1],travelledOnBackside[r][0],Qt::yellow);
            dstArrSinoFailed[travelledOnBackside[r][1]] = dstArrSinoSource[fromX];
        }
    }
    QString nrOfProjString2 = QString::number(numberOfAngles);
    QString nameRI2 = QString::number(riMedium,'g',4);
    nameRI2.append("_");
    nameRI2.append(QString::number(riSample,'g',4));
    nameRI2.append("_");
    rearrangedSinogramFails.save(inputPathSinogram+"\\"+nameRI2+nrOfProjString2+"_sinogramRearrangedFailsFromExternal.png");
    rearrangedExternalSinogram.save(inputPathSinogram+"\\"+nameRI2+nrOfProjString2+"_sinogramRearrangedFromExternal.png");
    rearrangedSinogramFails.fill(0);
    qDebug()<<"Sinogram fertig sortiert!";
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
        double gamma = -(qAtan(slope) - qAsin((n1*qSin(qAtan(slope)))/n2));
        return gamma;
    }
}

double surface_fitting_test::getValueForGaussDistribution(double gammaX, double gammaY, int x, int y, double sigma)
{
    double value = (1/(2*M_PI*pow(sigma,2)))*exp(((-1)/(2*pow(sigma,2)))*(pow(x-gammaX,2)+pow(y-gammaY,2)));
    return value;
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
    double result = y1+(x2-x)*(y2-y1)/(x2-x1);
    return result;
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
    double result = bilinInterpolation(q11,q12,q21,q22,x1,x2,y1,y2,x,y);
    return result;
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

double surface_fitting_test::getTransmissionGrade(double riMedium, double riSample, QVector<double> rotatedEntryPoint)
{
    if(riMedium==riSample){
        return 1.0;
    }else if(rotatedEntryPoint[3]==0){ //Wenn die Oberfläche senkrecht ist, muss diese Formel verwendet werden
        return 1.0 - pow((riMedium-riSample)/(riMedium+riSample),2);
    }else{
        double alpha = qAtan(rotatedEntryPoint[3]);
        double beta = alpha - rotatedEntryPoint[1];
        double tp = 1.0 - pow((qTan(alpha-beta)/qTan(alpha+beta)),2);
        double ts = 1.0 - pow((-qSin(alpha-beta)/qSin(alpha+beta)),2);
        return (tp+ts)/2.0;
    }
}

QImage surface_fitting_test::offsetImage(QImage sinogram, int offset)
{
    if(offset == 0){
        return sinogram;
    }else if(offset>0){
        return sinogram;
    }
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
        qDebug()<<getDeltaThetaForPartner(slope[1],riMedium,riSample);
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
    nameRI = QString::number(riMedium,'g',3);
    nameRI.append("_");
    nameRI.append(QString::number(riSample,'g',3));
    nameRI.append("_");
    double higherRI = riMedium;
    if(higherRI<riSample){
        higherRI = riSample;
    }
    bool createBScans = ui->checkBox_createBScans->isChecked();
    createTransmission = ui->checkBox_createTransmission->isChecked();
    sinogramHisto = QImage(inputSurface.width(),numberOfProjections+2,QImage::Format_Grayscale16);
    sinogramHisto.fill(0);
    transmissionHisto = sinogramHisto;
    rotatedEntryPoints = QVector<QVector<QVector<double>>>(numberOfProjections);
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
        rotatedEntryPoints[q][0]={0,0,0};
        for(int x = 1;x<inputSurface.width();x++){
            rotatedEntryPoints[q][x]={0,0};
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
                if(exitAngle>=0&&exitAngle<1.0){  //Strahl wird nach rechts abgelenkt
                    int X=x;
                    bool success = false;
                    while(success == false && getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X+1)>0){
                        QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X), X+1, getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X+1));
                        QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                        QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                        if(intersection[0]>=X&&intersection[0]<X+1){
                            rotatedEntryPoints[q][x][2] = intersection[2];
                            success = true;
                        }else{
                            X++;
                        }
                    }
                    if(success == false){
                        qDebug()<<"Strahl wurde nach rechts abgelenkt und fand keine Rückseite bei: "<<q<<x;
                        if(getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X)!=getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X)){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X), X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[1]>=getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X)&&intersection[1]<=getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X)){
                                qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                                rotatedEntryPoints[q][x][2] = intersection[2];
                                success = true;
                            }
                        }
                        while(success == false && X>=x){
                            QVector<double> surfaceVector = parameterizeFromPoints(X-1, getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X-1), X, getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[0]>=X&&intersection[0]<X+1){
                                rotatedEntryPoints[q][x][2] = intersection[2];
                                qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
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
                            success = true;
                        }else{
                            X--;
                        }
                    }
                    if(success == false){
                        qDebug()<<"Strahl wurde nach links abgelenkt und fand keine Rückseite bei: "<<q<<x;
                        if(getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X)!=getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X)){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X), X, getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[1]>=getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X)&&intersection[1]<=getFirstValueFromBottom(rotatedSurfacesThinnedOut[q],X)){
                                //qDebug()<<"Strahl hat zwischen oben und unten Partner gefunden!";
                                rotatedEntryPoints[q][x][2] = intersection[2];
                                success = true;
                            }
                        }
                        while(success == false && X<=x){
                            QVector<double> surfaceVector = parameterizeFromPoints(X, getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X), X+1, getFirstValueFromTop(rotatedSurfacesThinnedOut[q],X+1));
                            QVector<double> rayVector = parameterizeFromAngle(x,bufVec[0],exitAngle);
                            QVector<double> intersection = getIntersectionPoint(rayVector,surfaceVector);
                            if(intersection[0]>=X&&intersection[0]<X+1){
                                rotatedEntryPoints[q][x][2] = intersection[2];
                                //qDebug()<<"Strahl hat auf Oberseite Parter gefunden";
                                success = true;
                            }else{
                                X++;
                            }
                        }
                    }
                }else{
                    qDebug()<<"Strahl wurde total reflektiert!"<<q<<x;
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
                double gray = 0.45*propagateRayThroughHisto(rotatedHistoImages[q],x,rotatedEntryPoints[q][x][0],rotatedEntryPoints[q][x][1],rotatedEntryPoints[q][x][2]);
                if(gray>65535){
                    qDebug()<<"Aufaddierter Grauwert zu groß für 16bit: "<<q<<x<<gray;
                    gray = 65535;
                }
                dstHisto[x]= int(gray);
                if(createTransmission){
                    if(rotatedEntryPoints[q][x][1]==1||rotatedEntryPoints[q][x][1]==-1){
                        dstTrans[x]= int(0);
                    }else{
                        int buf = (65535 - gray)*qCos(rotatedEntryPoints[q][x][1]*10.0);
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
    ui->pushButton_correctSinogram->setEnabled(true);
    qDebug()<<"jo";
}


void surface_fitting_test::on_doubleSpinBox_riMedium_valueChanged(double arg1)
{
    QVector<double> slope{0,0};
    riMedium = arg1;
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
                QVector<double> newPixel = rotateImagePointTo({double(x),double(y)},{double(inputSurface.width()/2.0),double(inputSurface.height()/2.0)},qDegreesToRadians(double(arg1)),true);
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


void surface_fitting_test::on_pushButton_correctSinogram_clicked()
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
            if(rotatedEntryPoints[p][x][0]!=0){
                rearrangedSinogramFails.setPixelColor(x,p,Qt::white);
                if(rotatedEntryPoints[p][x][1]==0){                                                          //bei senkrechten Strahlen wird nichts verschoben!
                    quint16 *dstSinoStraight = (quint16*)(sinogramHisto.bits()+p*sinogramHisto.bytesPerLine());
                    rearrangedSinogramFails.setPixelColor(x,p,Qt::gray);
                    dstArrSino[x]=dstSinoStraight[x];
                }else if(rotatedEntryPoints[p][x][1]<0){            // Probe wird im Uhrzeigersinn gedreht, um Partner zu finden; x wird größer
                    QVector<double> newPos;
                    double theta = 0;
                    deltaProj = 0;
                    double gamma = 0;
                    bool searchDone = false;
                    while(searchDone==false){
                        deltaProj++;
                        theta = (2*M_PI/double(numberOfProjections))*deltaProj;
                        newPos = rotateImagePointTo({double(x),rotatedEntryPoints[p][x][0]},{double(inputSurface.width()/2.0),double(inputSurface.height()/2.0)},theta,true);
                        if(deltaProj>150){
                            rearrangedSinogramFails.setPixelColor(x,p,Qt::darkRed);
                            qDebug()<<"Punkt ist weggelaufen :"<<p<<x;
                            dstArrSino[x] = 0;
                            searchDone = true;
                        }
                        else if(riMedium<riSample && std::abs(linInterpolation(int(newPos[0]),int(newPos[0])+1,getFirstValueFromTop(rotatedSurfacesThinnedOut[(p+deltaProj+numberOfProjections)%numberOfProjections],int(newPos[0])),getFirstValueFromTop(rotatedSurfacesThinnedOut[(p+deltaProj+numberOfProjections)%numberOfProjections],int(newPos[0])+1),newPos[0])-newPos[1])>5){
                            rearrangedSinogramFails.setPixelColor(x,p,Qt::darkMagenta);
                            qDebug()<<"Punkt auf Rückseite gelandet :"<<p<<x;
                            qDebug()<<rotatedEntryPoints[p][x];
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
                        else if(rotatedEntryPoints[(p+deltaProj+numberOfProjections)%numberOfProjections][int(newPos[0])][2]!=0&&rotatedEntryPoints[(p+deltaProj+numberOfProjections)%numberOfProjections][int(newPos[0])+1][2]!=0){
                            gamma = linInterpolation(int(newPos[0]),int(newPos[0])+1,rotatedEntryPoints[(p+deltaProj+numberOfProjections)%numberOfProjections][int(newPos[0])][1],rotatedEntryPoints[(p+deltaProj+numberOfProjections)%numberOfProjections][int(newPos[0])+1][1],newPos[0]);
                            if(theta >= std::abs(gamma)){
                                newPos = rotateImagePointTo({double(x),rotatedEntryPoints[p][x][0]},{double(inputSurface.width()/2.0),double(inputSurface.height()/2.0)},-gamma,true);
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
                }else if(rotatedEntryPoints[p][x][1]>0){            // Probe wird gegen den Uhrzeigersinn gedraht, um Partner zu finden; x wird kleiner
                    QVector<double> newPos;
                    double theta = 0;
                    deltaProj = 0;
                    double gamma = 0;
                    bool searchDone = false;
                    while(searchDone==false){
                        deltaProj--;
                        theta = (2*M_PI/double(numberOfProjections))*deltaProj;
                        newPos = rotateImagePointTo({double(x),rotatedEntryPoints[p][x][0]},{double(inputSurface.width()/2.0),double(inputSurface.height()/2.0)},theta,true);
                        if(std::abs(deltaProj)>150){
                            qDebug()<<"Punkt ist weggelaufen :"<<p<<x;
                            dstArrSino[x] = 0;
                            rearrangedSinogramFails.setPixelColor(x,p,Qt::red);
                            searchDone = true;
                        }
                        else if(riMedium<riSample&&std::abs(linInterpolation(int(newPos[0]),int(newPos[0])+1,getFirstValueFromTop(rotatedSurfacesThinnedOut[(p+deltaProj+numberOfProjections)%numberOfProjections],int(newPos[0])),getFirstValueFromTop(rotatedSurfacesThinnedOut[(p+deltaProj+numberOfProjections)%numberOfProjections],int(newPos[0])+1),newPos[0])-newPos[1])>5){
                            rearrangedSinogramFails.setPixelColor(x,p,Qt::darkMagenta);
                            qDebug()<<"Punkt auf Rückseite gelandet :"<<p<<x;
                            qDebug()<<rotatedEntryPoints[p][x];
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
                        else if(rotatedEntryPoints[(p+deltaProj+numberOfProjections)%numberOfProjections][int(newPos[0])][2]!=0&&rotatedEntryPoints[(p+deltaProj+numberOfProjections)%numberOfProjections][int(newPos[0])+1][2]!=0){
                            gamma = linInterpolation(int(newPos[0]),int(newPos[0])+1,rotatedEntryPoints[(p+deltaProj+numberOfProjections)%numberOfProjections][int(newPos[0])][1],rotatedEntryPoints[(p+deltaProj+numberOfProjections)%numberOfProjections][int(newPos[0])+1][1],newPos[0]);
                            if(std::abs(theta) >= std::abs(gamma)){
                                newPos = rotateImagePointTo({double(x),rotatedEntryPoints[p][x][0]},{double(inputSurface.width()/2.0),double(inputSurface.height()/2.0)},-gamma,true);
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
    QVector<double> slope = getSlopeAtEntry(bufferImg,ui->spinBox_aScan->value(),ui->spinBox_slopeSigma->value());
    auto test = getTransmissionGrade(riMedium,riSample,{10.0,getExitAngle(slope[1],riMedium,riSample),3,slope[1]});
    qDebug()<<test;
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
    for(int k = 0;k<numberOfProjections;k++){
        QVector<double>rotatedPoint=QVector<double>(2);
        double rotateBy = (2*M_PI/double(numberOfProjections))*k;
        rotatedSurfacesThinnedOut[k] = QImage(inputSurface.size(),QImage::Format_Grayscale8);
        rotatedSurfacesThinnedOut[k].fill(0);
        for(int x = 0;x<inputSurface.width();x++){
            for(int y = 0;y<inputSurface.height();y++){
                if(getColor(inputSurface,x,y)!=0){
                    rotatedPoint=rotateImagePointTo({double(x),double(y)},{double(inputSurface.width()/2.0),double(inputSurface.height()/2.0)},rotateBy,true);
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
                    QVector<double> newPixel = rotateImagePointTo({double(x),double(y)},{double(inputHisto.width()/2.0),double(inputHisto.height()/2.0)},rotationAngleHisto,true);
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

void surface_fitting_test::on_pushButton_continousSimulation_clicked()
{
    while(riMedium<=ui->doubleSpinBox_endRiMedium->value()){
        on_pushButton_createSinogram_clicked();
        on_pushButton_newCorrection_clicked();
        rotatedEntryPoints.resize(0);
        sinogramHisto.fill(0);
        QString riString = QString::number(riMedium,'g',4);
        qDebug()<<"Simulation of following Medium RI is fnished:"<<riString;
        riMedium = riMedium + 0.01;
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
            if(rotatedEntryPoints[p][x][0]!=0){
                rearrangedSinogramFails.setPixelColor(x,p,Qt::white);
                if(rotatedEntryPoints[p][x][1]==0){                                                          //bei senkrechten Strahlen wird nichts verschoben!
                    quint16 *dstSinoStraight = (quint16*)(sinogramHisto.bits()+p*sinogramHisto.bytesPerLine());
                    rearrangedSinogramFails.setPixelColor(x,p,Qt::gray);
                    dstArrSino[x]=dstSinoStraight[x];
                }else{
                    double deltaTheta = getDeltaThetaForPartner(rotatedEntryPoints[p][x][3],riMedium,riSample);
                    if(deltaTheta!=999){
                        double deltaProj = deltaTheta*double(numberOfProjections)/(2*M_PI);
                        QVector<double> buf = rotateImagePointTo({double(x),rotatedEntryPoints[p][x][0]},{double(inputSurface.width()/2.0),double(inputSurface.height()/2.0)},deltaTheta,true);
                        double absNewX = buf[0];
                        double value = newBilinInterpolFromSinogram(sinogramHisto,absNewX,double(p)+deltaProj);
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
    correctExternalSinogram(inputSinogram,inputSurface,riMedium,riSample);
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

