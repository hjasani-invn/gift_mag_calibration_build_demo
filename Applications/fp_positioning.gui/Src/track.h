#ifndef TRACK_H
#define TRACK_H

#include <QGraphicsScene>
#include <QMainWindow>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFile>
#include <QGraphicsItem>
#include <QMutex>
#include <QStackedWidget>
#include <QMainWindow>

#include <string>
#include <iostream>
#include <stdio.h>

#include "Fppe.hpp"
#include "myscene.h"
#include "VenueEx.hpp"

//#include "mainwindow.h"
//#include "secondwindow.h"

struct OutPut_for_image
{
    double time;
    Fppe::Particle* OutParticleArray;
    int Particle_count;
    double OutPos[3];
    bool initialized = false;
};


class Track : public QObject
{
Q_OBJECT
public:
    Track();
    ~Track();

    void set_cm_in_pix(double new_cm_in_pix);

    void set_grid(double grid_max_X, double grid_max_Y, double cell_size, unsigned int grid_floor_count);

    void get_settings_from_json(QMainWindow *mw, QString json_filename);

    //void get_settings_from_json();

    void set_floor_count(unsigned int floor_count);

    void test_images();

    QJsonDocument loadJson(QString fileName);

    //void set_updaters(bool WiFi, bool BLE, bool Mag);

    QVector<MyScene*> scenes_for_floors; // array of pointers to scenes, one for each floor
    QVector<MyScene*> scenes_for_mock_floors_up; // array of pointers to scenes, one for each floor
    QVector<MyScene*> scenes_for_mock_floors_down; // array of pointers to scenes, one for each floor

    QGraphicsItemGroup *particle_group;
    bool group_added_to_scene;

    unsigned int floor_count;
    double plan_venue_x;
    double plan_venue_y;
    double cell_size;

    double cm_in_pix;
    //QJsonDocument json;
    QString json_filename;

    QVector<QVector<QGraphicsEllipseItem*>*> particles_for_floors;
    QVector<QVector<QGraphicsEllipseItem*>*> particles_for_mock_floors_up;
    QVector<QVector<QGraphicsEllipseItem*>*> particles_for_mock_floors_down;

    int particles_count;
    int *particles_count_for_floor;
    QMutex mtx;

    //QString image_path;
    //bool has_image;
    QVector<QString*> image_path_vector;
    QVector<bool> has_image_vector;

    //QStackedWidget *windows_list;
    //QVector<SecondWindow*> windows_vector;

    QVector<QMainWindow*> windows_vector;
    QVector<QMainWindow*> mock_windows_vector_up;
    QVector<QMainWindow*> mock_windows_vector_down;


signals:
    void sceneChanged();

private:


//    Fppe::Venue ven;
};

#endif // TRACK_H
