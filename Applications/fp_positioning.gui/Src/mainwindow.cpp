#include <QFileDialog>
#include <QMessageBox>
#include <iostream>
#include <string>
#include <QPainter>
#include <QPixmap>
#include <vector>
#include <cmath>
#include <fstream>

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "secondwindow.h"
#include "ui_secondwindow.h"

static  QColor const colors[7] = { Qt::black, Qt::gray, Qt::blue, Qt::green, Qt::yellow, Qt::magenta, Qt::red };

MainWindow::MainWindow(QWidget *parent) :
QMainWindow(parent),
ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    memset(&ImgOut, 0, sizeof(ImgOut));

    particales_isualization = ParticalesVisualization::NO;
    ui->laststep_radioButton->setChecked(true);

    thread = NULL;
    task = NULL;

    QString applicationPath = QCoreApplication::applicationDirPath();
    settingsFileName = applicationPath + "/" + "settings.ini";

    QSettings settings(settingsFileName, QSettings::IniFormat);
    settings.beginGroup("Colors");
    int index = settings.value("trackColor").toInt();
    settings.endGroup();
    trackColor = colors[index];

    settings.beginGroup("Track");
    int i = settings.value("diameter").toInt();
    ui->positionDiameter_spinBox->setValue(i);
    settings.endGroup();


    ui->message_frame->setHidden(true);

    ui->SetTrackMode_comboBox->addItem("Magnetic", 0);
    ui->SetTrackMode_comboBox->addItem("Mixed", 1);
    ui->SetTrackMode_comboBox->addItem("Started", 2);

    //ui->SetParticleColor_comboBox->setHidden(true);
    //ui->SetParticleColor_Label->setHidden(true);

    ui->SetTrackColor_comboBox->addItem("Black", 0);
    ui->SetTrackColor_comboBox->addItem("Gray", 1);
    ui->SetTrackColor_comboBox->addItem("Blue", 2);
    ui->SetTrackColor_comboBox->addItem("Green", 3);
    ui->SetTrackColor_comboBox->addItem("Yellow", 4);
    ui->SetTrackColor_comboBox->addItem("Magenta", 5);
    ui->SetTrackColor_comboBox->addItem("Red", 6);

    ui->SetTrackColor_comboBox->setCurrentIndex(index);


    //    particleMinColor = Qt::cyan;
    //    particleMinColor = Qt::blue;
    //    particleMaxColor = Qt::blue;
    //    particleMaxColor = Qt::red;

    //int rc, gc, bc;
    //particleMinColor.getRgb(&rc, &gc, &bc);

    //particleMinColor.getRgbF(&particleMinRed, &particleMinGreen, &particleMinBlue);
    //particleMaxColor.getRgbF(&particleMaxRed, &particleMaxGreen, &particleMaxBlue);
}

MainWindow::~MainWindow()
{
    delete ui;
}

QString MainWindow::folder_from_file(const QString& file_name)
{
    int n = file_name.size() - 1;
    for (; n >= 0; --n) {
        if (file_name.at(n) == "/") {
            return file_name.left(n + 1);
        }
    }
    return ".";
}

QString MainWindow::get_working_folder()
{
    QSettings settings(settingsFileName, QSettings::IniFormat);
    settings.beginGroup("WorkingFolder");
    QString folder = settings.value("folder", ".").toString();
    settings.endGroup();
    return folder;
}

void MainWindow::set_working_folder(const QString& folder)
{
    QSettings settings(settingsFileName, QSettings::IniFormat);
    settings.beginGroup("WorkingFolder");
    settings.setValue("folder", folder);
    settings.endGroup();
}

void MainWindow::on_loadJSON_button_clicked()
{
    on_sceneChanged_counter = 0;
    QString folder = get_working_folder();
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose json file"), folder, tr("JSON (*.json)"));

    this->json_filename = filename;
    if (filename != NULL)
    {
        folder = folder_from_file(filename);

        QDir::setCurrent(folder);

        set_working_folder(folder);

        ui->labelFloorName->setText(filename);

        this->track->get_settings_from_json(this, filename); // JSON name is saved after that as "this->track->json_filename"
        this->track->set_grid(this->track->plan_venue_x, this->track->plan_venue_y, 1.0, this->track->floor_count);
    }
    isWorking = false;
    //ui->exit_button
}

void MainWindow::on_start_button_clicked()
{

    /*
    if( isWorking && (thread != NULL))
    {
    thread->quit();
    thread == NULL;
    }
    */
    if (isWorking && (task != NULL))
        task->stop();

    isWorking = !isWorking;
    if (isWorking)
        ui->start_button->setText("Stop");
    else
    {
        ui->start_button->setText("Start");
        return;
    }

    on_sceneChanged_counter = 0;
    std::string working_folder = get_working_folder().toStdString();

    if (working_folder.back() != '\\')
    {
        working_folder = working_folder + "\\";
    }
    QString conlose_log = QString::fromUtf8(working_folder.c_str()) + QString::fromUtf8("conlose.log");
    freopen(conlose_log.toLocal8Bit().data(), "w", stdout);

    thread = new QThread();
    //    std::string main_json = this->track->json_filename.toUtf8().constData();
    std::string main_json = this->json_filename.toUtf8().constData();
    //Task *task = new Task(main_json, &ImgOut, this->track);
    mPause = new QWaitCondition();
    task = new Task(main_json, &ImgOut, this->track, mPause);
    //   task->setQMainWindow(this);
    task->setTrackForMode(mTrackForMode);

    task->moveToThread(thread);

    connect(thread, SIGNAL(started()), task, SLOT(doWork()));
    connect(task, SIGNAL(file_error_message(QString, QString, QString, bool)), this, SLOT(on_file_error_message(QString, QString, QString, bool)), Qt::BlockingQueuedConnection);
    connect(task, SIGNAL(file_error_message_last(bool)), this, SLOT(on_file_error_message_last(bool)), Qt::BlockingQueuedConnection);
//    connect(task, SIGNAL(workFinished()), thread, SLOT(quit()));
    connect(task, SIGNAL(workFinished()), this, SLOT(on_task_quit()));
    connect(task, SIGNAL(sceneChanged(OutPut_for_image *)), this, SLOT(on_sceneChanged()));
    connect(thread, SIGNAL(finished()), task, SLOT(deleteLater()));
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    /*
    connect( this->track, SIGNAL(sceneChanged()), this, SLOT(on_sceneChanged()) );
    */

    isPaused = false;
    task->setPause(isPaused);

    thread->start();
}

void MainWindow::on_task_quit()
{
    ui->start_button->setText("Start");
    isWorking = false;

    thread->quit();
    //thread == NULL;
}

void MainWindow::on_pause_button_clicked()
{
    isPaused = !isPaused;
    task->setPause(isPaused);
    if (!isPaused)
    {
        mPause->wakeAll();
        ui->pause_button->setText("Pause");
    }
    else
        ui->pause_button->setText("Continue");
}

void MainWindow::on_exit_button_clicked()
{
    //QApplication::closeAllWindows();
    QApplication::quit();
}

void MainWindow::on_no_radioButton_toggled()
{
    bool state = ui->no_radioButton->isChecked();
    if (state)
        particales_isualization = ParticalesVisualization::NO;
}

void MainWindow::on_laststep_radioButton_toggled()
{
    bool state = ui->laststep_radioButton->isChecked();
    if (state)
        particales_isualization = ParticalesVisualization::LAST_STEP;
}

void MainWindow::on_allsteps_radioButton_toggled()
{
    bool state = ui->allsteps_radioButton->isChecked();
    if (state)
        particales_isualization = ParticalesVisualization::ALL_STEPS;

}

void MainWindow::on_SetTrackColor_comboBox_currentIndexChanged(int index)
{
    trackColor = colors[index];
    QSettings settings(settingsFileName, QSettings::IniFormat);
    settings.beginGroup("Colors");
    settings.setValue("trackColor", QString::number(index));
    settings.endGroup();
}

void MainWindow::on_SetParticleColor_comboBox_currentIndexChanged(int index)
{
    particleColor = colors[index];
}

void MainWindow::on_positionDiameter_spinBox_valueChanged(int i)
{
    QSettings settings(settingsFileName, QSettings::IniFormat);
    settings.beginGroup("Track");
    settings.setValue("diameter", QString::number(i));
    settings.endGroup();
}

void MainWindow::on_SetTrackMode_comboBox_currentIndexChanged(int index)
{
    mTrackForMode = index;
}

void MainWindow::setFloorName(QString name)
{
    ui->labelFloorName->setText(name);
}

void MainWindow::on_sceneChanged()
{
    //static int on_sceneChanged_counter = 0;
    static double w = 0;
    static double h = 0;
    static int16_t previous_floor = 0;
    double scale_x = 1;
    double scale_y = 1;

    double x_pix = 0;
    double y_pix = 0;
    QRect trackViewRect = ((MainWindow*)(this->track->windows_vector[0]))->ui->trackView->geometry();
    if (on_sceneChanged_counter > 1)
    {
        scale_x = (this->track->plan_venue_x) / w;
        scale_y = (this->track->plan_venue_y) / h;
    }
    this->track->mtx.lock();

    //    double Y_MAX_PIX = 100 * (this->track->venue_y) / (this->track->cm_in_pix);

    if (on_sceneChanged_counter > 1)
    {
        ui->timeValue->setText(QString::number(this->ImgOut.time / 1000));
        ui->particlesValue_Label->setText(QString::number(this->ImgOut.Particle_count));

        x_pix = this->ImgOut.OutPos[0] / scale_x;
        y_pix = h - this->ImgOut.OutPos[1] / scale_y;
    }
    //    double rad = 5; //this->track->cm_in_pix;
    //double rad = 0.2 / scale_x; //this->track->cm_in_pix;
    //rad = (rad > 1) ? rad : 1;
    double rad = ((double)(ui->positionDiameter_spinBox->value()))/2; // radius in pixels
    double r = 1; // * 2/this->track->cm_in_pix;

    //std::cout << scale_x << "    " << scale_y << std::endl;

    int16_t current_floor = (int16_t)floor(this->ImgOut.OutPos[2] + 0.5);

    on_sceneChanged_counter++;
    /*
    std::cout << on_sceneChanged_counter << "    ";
    std::cout << this->ImgOut.OutPos[0] << "    ";
    std::cout << this->ImgOut.OutPos[1] << "    ";
    std::cout << x_pix << "    ";
    std::cout << y_pix << "  !  ";

    std::cout << this->ImgOut.OutPos[2] << "    ";
    std::cout << current_floor << "    ";
    std::cout << this->ImgOut.Particle_count << "    ";
    std::cout << std::endl;
    */

//std::cout << "\n" << on_sceneChanged_counter << "\n";
//std::cout.flush();
    // remove particles and trackfrom previous work
    if (on_sceneChanged_counter == 1)
    {
        for (unsigned int n = 0; n < this->track->floor_count; n++)
        {
            QGraphicsScene *scene = this->track->scenes_for_floors[n];
            QList<QGraphicsItem *>  listItems = scene->items();
            if (listItems.count() > 0)
            {
                QList<QGraphicsItem *>::iterator it = listItems.begin();
                //it++;
                while (listItems.count() > 1)
                {
                    scene->removeItem(*it);
                    it = listItems.erase(it);
                }
            }
        }
    }

    QBrush particle_brush = QBrush(particleColor);

    if (current_floor >= 0 && current_floor < this->track->floor_count)
    {
        //std::cout << ImgOut.Particle_count << "    "<< this->track->particles_count << "    " << std::endl;;
        if (this->ImgOut.Particle_count > 0)
        {
            if (particales_isualization == ParticalesVisualization::LAST_STEP)
            {
                // remove particles from previous step
                for (unsigned int n = 0; n < this->track->floor_count; n++)
                {
                    QGraphicsScene *scene = this->track->scenes_for_floors[n];
                    QVector<QGraphicsEllipseItem*> *particles = this->track->particles_for_floors[n];
                    for (int i = 0; i <particles->size(); i++)
                    {
                        scene->removeItem((*particles)[i]);
                    }
                    this->track->particles_for_floors[n]->clear();
                }
            }
            this->track->particles_count = ImgOut.Particle_count;
            if (particales_isualization != ParticalesVisualization::NO)
            {
                double max_particle_weight = 0;
                double min_particle_weight = 1;

                for (int i = 0; i < this->track->particles_count; i++)
                {
                    // std::cout << this->ImgOut.OutParticleArray[i].w << "    ";
                    // std::cout << std::endl;
                    if (this->ImgOut.OutParticleArray[i].w > max_particle_weight)
                        max_particle_weight = this->ImgOut.OutParticleArray[i].w;
                    if (this->ImgOut.OutParticleArray[i].w < min_particle_weight)
                        min_particle_weight = this->ImgOut.OutParticleArray[i].w;
                }

                double delta_particle_weight = max_particle_weight - min_particle_weight;
                double rc, gc, bc;
                double dr = particleMaxRed - particleMinRed;
                double dg = particleMaxGreen - particleMinGreen;
                double db = particleMaxBlue - particleMinBlue;
                double shift = 0;

                for (int i = 0; i < this->track->particles_count; i++)
                {
                    int16_t particale_floor = (int16_t)floor(this->ImgOut.OutParticleArray[i].z + 0.5);

                    if (particale_floor >= 0 && particale_floor < this->track->floor_count)
                    {
                        {
                            QGraphicsEllipseItem *new_partical = new QGraphicsEllipseItem;

                            if (delta_particle_weight > 1e-6)
                            {
                                shift = (this->ImgOut.OutParticleArray[i].w - min_particle_weight) / delta_particle_weight;
                                if (shift < 0.05)
                                {
                                    particleColor = Qt::black;
                                }
                                if (shift > 0.05 && shift < 0.2)
                                {
                                    particleColor = Qt::blue;
                                }
                                if (shift > 0.2 && shift < 0.4)
                                {
                                    particleColor = Qt::cyan;
                                }
                                if (shift > 0.4 && shift < 0.6)
                                {
                                    particleColor = Qt::green;
                                }
                                if (shift > 0.6 && shift < 0.8)
                                {
                                    particleColor = Qt::magenta;
                                }
                                if (shift > 0.8)
                                {
                                    particleColor = Qt::red;
                                }
                            }
                            else
                            {
                                particleColor = QColor(Qt::black);
                            }

                            particle_brush.setColor(particleColor);

                            double particleX_pix = ImgOut.OutParticleArray[i].x / scale_x;
                            double particleY_pix = h - ImgOut.OutParticleArray[i].y / scale_y;
                            new_partical->setRect(particleX_pix - r, particleY_pix - r, 2 * r, 2 * r);
                            new_partical->setPen(Qt::NoPen);
                            new_partical->setBrush(particle_brush);

                            new_partical->setVisible(true);

                            this->track->scenes_for_floors[particale_floor]->addItem(new_partical);
                            this->track->particles_for_floors[particale_floor]->append(new_partical);
                        }
                    }
                }
            }

            // show position
            if (on_sceneChanged_counter > 1)
                this->track->scenes_for_floors[current_floor]->addEllipse(x_pix - rad, y_pix - rad, rad*2.0, rad*2.0, QPen(), trackColor);
        }

        /*
        ui->trackView->setScene(&sc[integer_floor]);
        ui->trackView->update();
        ui->trackView->show();
        */

        //    QRectF scene_size = this->track->scenes_for_floors[current_floor]->sceneRect();
        if (on_sceneChanged_counter == 1)
        {
            w = this->track->scenes_for_floors[current_floor]->sceneRect().width();
            h = this->track->scenes_for_floors[current_floor]->sceneRect().height();
        }
        // for test only
        //  this->track->scenes_for_floors[current_floor]->addLine(0, 0,  800, 800, QPen());
        //this->track->scenes_for_floors[current_floor]->addRect(00, 00,  200, 200, QPen(), QBrush(Qt::blue));

        ((MainWindow*)(this->track->windows_vector[0]))->ui->trackView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

        //this->track->scenes_for_floors[current_floor]->setSceneRect(100,100,
        //        ((MainWindow*)(this->track->windows_vector[0]))->ui->trackView->frameSize().width()-100,
        //        ((MainWindow*)(this->track->windows_vector[0]))->ui->trackView->frameSize().height()-100);
        //this->track->scenes_for_floors[current_floor]->addLine(0, 0,  8000, 8000, QPen());


        ((MainWindow*)(this->track->windows_vector[0]))->ui->trackView->setSceneRect(0, 0, w, h);
        ((MainWindow*)(this->track->windows_vector[0]))->ui->trackView->setScene(this->track->scenes_for_floors[0]);
        //    ((MainWindow*)(this->track->windows_vector[0]))->ui->trackView->setScene(&sc);
        //this->ui->trackView->setScene(&sc);
        //((MainWindow*)(this->track->windows_vector[0]))->ui->trackView->update();
        for (int n = 1; n < this->track->floor_count; n++)
        {
            ((SecondWindow*)(this->track->windows_vector[n]))->ui->trackView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
            //        ((SecondWindow*)(this->track->windows_vector[n]))->ui->trackView->setSceneRect(0,0,((SecondWindow*)(this->track->windows_vector[n]))->ui->trackView->frameSize().width(),((SecondWindow*)(this->track->windows_vector[n]))->ui->trackView->frameSize().height());
            ((SecondWindow*)(this->track->windows_vector[n]))->ui->trackView->setSceneRect(0, 0, w, h);
            ((SecondWindow*)(this->track->windows_vector[n]))->ui->trackView->setScene(this->track->scenes_for_floors[n]);
            // ((SecondWindow*)(this->track->windows_vector[n]))->ui->trackView->update();
        }
        //  make current floor window active (in front)
        if (previous_floor != current_floor)
        {
            this->track->windows_vector[current_floor]->activateWindow();
            previous_floor = current_floor;
        }
    }

    //    this->track->mtx.tryLock();
    this->track->mtx.unlock();
}

void MainWindow::on_file_error_message(QString start_message,
    QString file_name,
    QString end_message,
    bool program_exit)
{
    std::cout << (start_message.toStdString()) << "    " << (file_name.toStdString()) << "    " << (end_message.toStdString()) << std::endl;
    // QString message = start_message + QString("\n") + file_name + QString("\n") + end_message;
    QString message1 = start_message + QString("  ") + file_name + QString("  ") + end_message;
    // QMessageBox::StandardButton resBtn = QMessageBox::information( this,  QString::fromUtf8(""), message, QMessageBox::Ok);

    if (ui->message_frame->isHidden())
    {
        ui->message_PlainTextEdit->clear();
    }

    ui->message_frame->setHidden(false);
    ui->message_PlainTextEdit->appendPlainText(message1);
    //ui->messages_close_Button
    if (program_exit)
        exit(1); // exit if can't open input file with increments and magnetic data
}

void MainWindow::on_file_error_message_last(bool program_exit)
{
    //  QMessageBox::StandardButton resBtn = QMessageBox::information( this,  QString::fromUtf8(""), "Close", QMessageBox::Ok);
    this->program_exit = false;
    if (ui->message_frame->isHidden())
    {
        mPause->wakeAll();
        if (program_exit)
            exit(1); // exit if can't open input file with increments and magnetic data
    }
    else
        this->program_exit = program_exit;
}

void MainWindow::on_messages_finish_pushButton_clicked()
{
    ui->message_frame->setHidden(true);
    if (this->program_exit)
        exit(1); // exit if can't open input file with increments and magnetic data
    mPause->wakeAll();

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    /*
    QMessageBox::StandardButton resBtn = QMessageBox::question( this, APP_NAME,
    tr("Are you sure?\n"),
    QMessageBox::Cancel | QMessageBox::No | QMessageBox::Yes,
    QMessageBox::Yes);
    if (resBtn != QMessageBox::Yes)
    {
    event->ignore();
    } else
    {
    event->accept();
    }
    */
    QApplication::quit();
}


