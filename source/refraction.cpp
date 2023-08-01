#include "header/refraction.h"
#include "ui_refraction.h"

refraction::refraction(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::refraction)
{
    ui->setupUi(this);
}

refraction::~refraction()
{
    delete ui;
}

void refraction::on_pushButton_browse_clicked()
{
    QString file_name = QFileDialog::getOpenFileName(this,tr("Open File"), "F:/mSLOT/MSQ_C3", tr("Images (*.bmp *.png *.xpm *.jpg *.tif *.tiff"));

    if(!file_name.isEmpty()){
        QMessageBox::information(this, "...", file_name);
        matOriginal = cv::imread(file_name.toStdString(),cv::IMREAD_GRAYSCALE);
        cv::imshow("Display window", matOriginal);
        cv::Moments mom = moments(matOriginal);
        double centerX = mom.m10/mom.m00;
        double centerY = mom.m01/mom.m00;
        qDebug()<<"Moment des Bildes in X: "<<centerX;
        qDebug()<<"Moment des Bildes in Y: "<<centerY;

        cv::Mat out;

        cv::threshold(matOriginal,out,20,255,cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        cv::findContours(out,contours,hierarchy,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        cv::drawContours(matOriginal,contours,0,100,2);

        cv::imshow("contours",matOriginal);
    }
}


void refraction::on_pushButton_loadStack_clicked()
{
    const QString folderpathGaussList = QFileDialog::getExistingDirectory(this,tr("Gauss Folder"),"F:/mSLOT/MSQ_C3");
    QDir dir(folderpathGaussList);
    gaussList = dir.entryInfoList();
    while(!gaussList.first().absoluteFilePath().endsWith(".jpg")){
        qDebug()<<"Erster Eintrag entfernt, weil es kein Bild war!";
        gaussList.removeFirst();
    }
    while(!gaussList.last().absoluteFilePath().endsWith(".jpg")){
        qDebug()<<"Letzter Eintrag entfernt, weil es kein Bild war!";
        gaussList.removeLast();
    }
    qDebug()<<gaussList;
    cv::Mat img;
    cv::Mat weights = cv::Mat::zeros(1024,1280,cv::IMREAD_GRAYSCALE);
    QVector<QVector<double>> momentsList(gaussList.size());
    int stackSize = gaussList.size();
    for ( int i = 0; i<stackSize;i++){
        QString file_name = gaussList[i].absoluteFilePath();
        img = cv::imread(file_name.toStdString(),cv::IMREAD_GRAYSCALE);
        cv::Moments mom = moments(img);
        momentsList[i]={mom.m10/mom.m00,mom.m01/mom.m00};
        cv::Point point = {int(momentsList[i][0]),int(momentsList[i][1])};
        cv::circle(weights,point,1,200,-1);
        qDebug()<<"Moment in X bei Bild "<<i+1<<mom.m10/mom.m00;
    }
    emit sendMomentsList(momentsList);
    //qDebug()<<momentsList;
    //cv::imshow("Display window",weights);
}
