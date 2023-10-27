 #include "mainwindow.h"
#include "secondwindow.h"
#include <QApplication>
#include <QJsonObject>
//#include "track.h"


int main( int argc, char* argv[] )
{

    QStringList paths = QCoreApplication::libraryPaths();
    paths.append(".");
    paths.append("imageformats");
    paths.append("platforms");
    paths.append("sqldrivers");
    QCoreApplication::setLibraryPaths(paths);

    QApplication app(argc, argv);
    MainWindow w;
    w.show();

    Track* track = new Track();
    w.set_track(track);

    return app.exec();
}
