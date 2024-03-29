#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <header/surface_fitting_test.h>
#include <header/dejitter.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_openSurfaceFitting_clicked();

    void on_pushButton_openDejitter_clicked();

private:
    Ui::MainWindow *ui;
    surface_fitting_test *fit;
    dejitter *dej;
};
#endif // MAINWINDOW_H
