#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "secondwindow.h"
#include "ui_secondwindow.h"
#include <QFileDialog>
#include <iostream>
#include <string>
#include <QPainter>
#include <QPixmap>
#include <vector>
#include <cmath>
#include <fstream>

SecondWindow::SecondWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SecondWindow)
{
    ui->setupUi(this);

 //   ui->cm_pix_lineEdit->setInputMask("09.00");
 //   ui->cm_pix_lineEdit->setText("60.00");

 //   ui->open_track_folder_button->setEnabled(false);
 //   ui->open_FP_folder_button->setEnabled(false);
 //   ui->open_out_folder_button->setEnabled(false);

    memset(&ImgOut, 0, sizeof(ImgOut));
}

SecondWindow::~SecondWindow()
{
    delete ui;
}
/*
void SecondWindow::on_cm_pix_lineEdit_editingFinished()
{
    //QString input = ui->cm_pix_lineEdit->text();
    //double d = input.toDouble();
//    std::cout << d << std::endl;
    //this->track->set_cm_in_pix(d);
}
*/
void SecondWindow::setFloorName(QString name)
{
    //ui->labelFloorName->setText(name);

}

void SecondWindow::on_sceneChanged()
{
    this->track->mtx.lock();

    double Y_MAX_PIX = 100 * (this->track->plan_venue_y) / (this->track->cm_in_pix);

    double x_pix = 100 * this->ImgOut.OutPos[0] / (this->track->cm_in_pix);
    double y_pix = Y_MAX_PIX - 100 * this->ImgOut.OutPos[1] / (this->track->cm_in_pix);
    double rad = 10/this->track->cm_in_pix;
    double r = 2 * 2/this->track->cm_in_pix;

   // QGraphicsScene* sc = this->track->track_scenes;
    int16_t integer_floor = (int16_t)this->ImgOut.OutPos[2];

    QBrush particle_color = Qt::black;
    //QBrush particle_color = Qt::green;

    if (integer_floor < 0)
    {
        particle_color = Qt::red;
        integer_floor = 0;
    }


    if (ImgOut.Particle_count != this->track->particles_count)
    {
        for (int i=0; i < this->track->particles_count; i = i + 1)
        {
           //sc[integer_floor].removeItem(&this->track->particles[i]);
        }
        //delete []  this->track->particles;
        //this->track->particles = new QGraphicsEllipseItem[ImgOut.Particle_count];
        this->track->particles_count = ImgOut.Particle_count;
        for (int i=0; i < this->track->particles_count; i = i + 1)
        {
           //sc[integer_floor].addItem(&this->track->particles[i]);
        }
    }

    for (int i = 0; i < this->track->particles_count; i+= 1)
    {
        uint16_t particle_floor = (int)(ImgOut.OutParticleArray[i].z);

        double particleX_pix = 100 * ImgOut.OutParticleArray[i].x / (this->track->cm_in_pix);
        double particleY_pix = Y_MAX_PIX - 100 * ImgOut.OutParticleArray[i].y / (this->track->cm_in_pix);

        //this->track->particles[i].setRect(particleX_pix-r, particleY_pix-r, 2*r, 2*r);
        //this->track->particles[i].setPen(Qt::NoPen);
        //this->track->particles[i].setBrush(particle_color);
        //this->track->particles[i].setVisible(particle_floor == integer_floor);
    }


    std::cout << this->ImgOut.OutPos[0] << " , " << this->ImgOut.OutPos[1] << " , " << integer_floor << std::endl;
    // POSITIONS
    //sc[integer_floor].addEllipse(x_pix-rad, y_pix-rad, rad*2.0, rad*2.0, QPen(), QBrush(Qt::blue));

    //ui->trackView->setScene(&sc[integer_floor]);
    //ui->trackView->update();
    //ui->trackView->show();


//    this->track->mtx.tryLock();
    this->track->mtx.unlock();
}


