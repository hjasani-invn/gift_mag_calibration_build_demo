#ifndef TASK_H
#define TASK_H

#include <QtCore>
#include "FP_console.hpp"

#include "track.h"

class Task : public QObject // QWidget //
{
Q_OBJECT
public:
    Task(std::string &JSON, OutPut_for_image *Out, Track *track, QWaitCondition *pause);
    ~Task();

    void setPause(bool pause);

    void setTrackForMode(int mode);

    void stop();

    OutPut_for_image *Output_Struct;

public slots:
    void doWork();

signals:
//    void sceneChanged();
    void sceneChanged(OutPut_for_image *OutStruct);
    void workFinished();
    void file_error_message(QString start_message,
                               QString file_name,
                               QString end_message,
                               bool program_exit);

    void file_error_message_last(bool program_exit);

private:
   // double venue_x;
   // double venue_y;
    std::string mainJSON;
    uint_fast16_t floor_count;
    std::string fp_path;
    std::string track_path;
    std::string out_path;
    bool WiFi_enabled;
    bool BLE_enabled;
    bool Mag_enabled;
    Track *mTrack;
    bool drawed;

    bool isPaused;
    QWaitCondition *mPause;
    QMutex mPauseMutex;;

    int mTrackForMode;

    bool mStop;
};


#endif // TASK_H
