#ifndef SECONDWINDOW_H
#define SECONDWINDOW_H

#include <QMainWindow>
#include "track.h"
#include "task.h"

namespace Ui {
class SecondWindow;
}

class SecondWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit SecondWindow(QWidget *parent = 0);
    ~SecondWindow();

    void set_track(Track* track)
    {
        this->track = track;
    }

    void setFloorName(QString name);


    OutPut_for_image ImgOut;

    Ui::SecondWindow *ui;

public slots:
    void on_sceneChanged();

private slots:

    //void on_cm_pix_lineEdit_editingFinished();


private:
    QPixmap track_pixmap;
    Track* track;
};

#endif // SECONDWINDOW_H
