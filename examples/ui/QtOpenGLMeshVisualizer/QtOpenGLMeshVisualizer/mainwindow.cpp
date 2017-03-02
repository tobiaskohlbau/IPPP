#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->openGLWidget->addVisualiser(_visualizer);
    //_visualizer.setTriangles();
}

MainWindow::~MainWindow()
{
    delete ui;
}
