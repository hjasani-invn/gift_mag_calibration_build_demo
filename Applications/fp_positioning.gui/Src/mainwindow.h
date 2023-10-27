#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "task.h"

namespace Ui {
class MainWindow;
}

enum class ParticalesVisualization
{
    NO = 0,
    LAST_STEP = 1,
    ALL_STEPS = 2
};


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void set_track(Track* track)
    {
        this->track = track;
        connect( this->track, SIGNAL(sceneChanged()), this, SLOT(on_sceneChanged()) );

    }

    void setFloorName(QString name);

    OutPut_for_image ImgOut;

    QString json_filename;

    Ui::MainWindow *ui;

    QGraphicsScene sc;

public slots:
    void on_sceneChanged();
    void on_file_error_message(QString start_message,
                               QString file_name,
                               QString end_message,
                               bool program_exit);
    void on_file_error_message_last( bool program_exit);
    void on_task_quit();

private slots:

   // void on_cm_pix_lineEdit_editingFinished();
    void on_loadJSON_button_clicked();
    void on_start_button_clicked();
    void on_pause_button_clicked();
    void on_exit_button_clicked();
    void on_no_radioButton_toggled();
    void on_laststep_radioButton_toggled();
    void on_allsteps_radioButton_toggled();
    void on_SetTrackColor_comboBox_currentIndexChanged(int index);
    void on_SetParticleColor_comboBox_currentIndexChanged(int index);
    void on_SetTrackMode_comboBox_currentIndexChanged(int index);
    void on_messages_finish_pushButton_clicked();
    void on_positionDiameter_spinBox_valueChanged(int i);

private:
    QPixmap track_pixmap;
    Track* track;
    Task *task;
    QThread *thread;
    bool isPaused;
    bool isWorking;
    QWaitCondition *mPause;

    ParticalesVisualization particales_isualization ;

    QColor  particleColor;

    QColor  particleMinColor;
    double  particleMinRed;
    double  particleMinGreen;
    double  particleMinBlue;

    QColor  particleMaxColor;
    double  particleMaxRed;
    double  particleMaxGreen;
    double  particleMaxBlue;

    QColor  trackColor;

    QString settingsFileName;

    int mTrackForMode;

    bool program_exit;

    int on_sceneChanged_counter;

    QString folder_from_file(const QString& file_name);
    QString get_working_folder();
    void set_working_folder(const QString& folder);

protected:
    virtual void	closeEvent(QCloseEvent * event);

};

#endif // MAINWINDOW_H
