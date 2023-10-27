#include <QJsonArray>
#include "track.h"
#include "secondwindow.h"
//#include "ui_secondwindow.h"


Track::Track()
{
    //this->track_scenes = new QGraphicsScene();
    this->cm_in_pix = 10.0;
    this->floor_count = 1;
    this->json_filename = "null";
    this->plan_venue_x = 0;
    this->plan_venue_y = 0;
    this->cell_size = 1;

    this->group_added_to_scene = false;

    this->particle_group = new QGraphicsItemGroup();
    //this->has_image = false;
    //this->image_path = "";

    //this->windows_list = new QStackedWidget();

    particles_count = 0;
    //particles = NULL;



}

Track::~Track()
{
    delete this->particle_group;
    //delete this->track_scenes;
}

void Track::set_cm_in_pix(double new_cm_in_pix)
{
    this->cm_in_pix = new_cm_in_pix;
}

QJsonDocument Track::loadJson(QString fileName)
{
    QFile jsonFile(fileName);
    jsonFile.open(QFile::ReadOnly);
//        return QJsonDocument().fromJson(jsonFile.readAll());
    //QJsonDocument doc = QJsonDocument().fromJson(jsonFile.readAll());

    QByteArray byteArray = jsonFile.readAll();

    jsonFile.close();

    QJsonDocument doc = QJsonDocument().fromJson(byteArray);

    return doc;
}

void Track::get_settings_from_json(QMainWindow *mw, QString json_filename)
{
    //this->json_filename = json_filename; // saving the name to have the ability to edit it later

    /*this->json =*/ //this->loadJson(json_filename);
    QJsonDocument json = this->loadJson(json_filename);

    QJsonObject jsonObject = json.object();

    QJsonObject venue = jsonObject["venue"].toObject();
    //QJsonValue vx = venue["plan_size_x"];

    this->floor_count = venue["floors_count"].toInt();
    if( ! ( venue["plan_size_x"].isNull() ) )
        this->plan_venue_x = venue["plan_size_x"].toDouble();
    else
        this->plan_venue_x = venue["size_x"].toDouble();
    if( ! ( venue["plan_size_y"].isNull() ) )
        this->plan_venue_y = venue["plan_size_y"].toDouble();
    else
        this->plan_venue_y = venue["size_y"].toDouble();
    this->cell_size = jsonObject["magnetic_cellsize"].toDouble();

    QJsonArray  floors = venue["floors"].toArray();
    QJsonArray  floor = floors[0].toArray();

    this->image_path_vector.clear();
    this->scenes_for_floors.clear();

    this->image_path_vector.append(new QString( floor[1].toString() ));

    mw->setWindowTitle (QString("           ") += floor[0].toString());

    this->windows_vector.append(mw);

    this->scenes_for_floors.append(new MyScene(mw));

    QGraphicsEllipseItem * ellipce = new (QGraphicsEllipseItem);
    QVector<QGraphicsEllipseItem*> *ellipces_vector = new QVector<QGraphicsEllipseItem*>();

    this->particles_for_floors.append( ellipces_vector);

    SecondWindow *sw;
    for(int n = 1; n < this->floor_count; n++)
    {
        floor = floors[n].toArray();

        sw = new SecondWindow();

        //sw->setFloorName(floor[0].toString());
        sw->setWindowTitle (QString("           ") += floor[0].toString());

        this->image_path_vector.append(new QString( floor[1].toString() ));

        QRect r = sw->geometry();
        sw->setMinimumHeight(r.height());
        sw->setMinimumWidth(r.width());
        sw->show();
     //   this->windows_list->addWidget(sw);

        this->windows_vector.append(sw);
        this->scenes_for_floors.append(new MyScene(sw));
        this->particles_for_floors.append( new QVector<QGraphicsEllipseItem*>());

    }

    test_images();
    windows_vector[0]->activateWindow();
}

void Track::test_images()
{
    QImage image;
    bool valid;
    for (int n = 0; n < image_path_vector.size(); n++)
    {
        valid = image.load(*image_path_vector[n]);

        if (valid)
        {
            this->has_image_vector.append(true);
        }
        else
        {
            this->has_image_vector.append(false);
            std::cout << "Wrong image file  " << image_path_vector[n] <<  ". Please select a png, jpg or bmp file." << std::endl;
        }
    }
}


void Track::set_grid(double grid_max_X, double grid_max_Y, double cell_size, unsigned int grid_floor_count)
{
    this->floor_count = grid_floor_count;

    for (unsigned int floor = 0; floor < grid_floor_count; ++floor)
    {
        if (this->has_image_vector[floor])
        {
            QImage image;
            bool valid = image.load(*this->image_path_vector[floor]);

            if (valid)
            {
                QPixmap background = QPixmap::fromImage(image);
               // this->scenes_for_floors[floor]->setBackground(image);
                this->scenes_for_floors[floor]->addPixmap(background);
            }
        }
// Grid
//        for (int iter_x = 0; iter_x < pix_grid_max_X; iter_x += pix_cell_size)
//        {
//            this->track_scene[floor].addLine(iter_x, 0, iter_x, pix_grid_max_Y, QPen(QColor(0, 0, 125, 127)));
//        }

//        for (int iter_y = 0; iter_y < pix_grid_max_Y; iter_y += pix_cell_size)
//        {
//            this->track_scene[floor].addLine(0, iter_y, pix_grid_max_X, iter_y, QPen(QColor(0, 0, 125, 127)));
//        }
    }
    emit this->sceneChanged();

}






