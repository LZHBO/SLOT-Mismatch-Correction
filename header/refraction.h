#ifndef REFRACTION_H
#define REFRACTION_H

#include <QWidget>
#include <QDebug>
#include <QVector>
#include <QFileDialog>
#include <QMessageBox>
#include <QPixmap>
#include <QImage>
#include <QVector>
#include <QQueue>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

namespace Ui {
class refraction;
}

class refraction : public QWidget
{
    Q_OBJECT

public:
    explicit refraction(QWidget *parent = nullptr);
    ~refraction();

private slots:
    void on_pushButton_browse_clicked();

    void on_pushButton_loadStack_clicked();

private:
    Ui::refraction *ui;
    cv::Mat matOriginal;
    cv::Mat matProc;

    QImage qimgOriginal;
    QImage qimgProcessed;

    QFileInfoList gaussList;
};

#endif // REFRACTION_H
