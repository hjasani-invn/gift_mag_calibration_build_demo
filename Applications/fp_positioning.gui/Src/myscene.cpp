#include <QGraphicsView>

#include "myscene.h"

MyScene::MyScene(QObject *parent) : QGraphicsScene(parent), has_imageBackground(0)
{
//this->imageBackground = NULL;
}

void MyScene::setBackground(QImage image)
{
 this->imageBackground = image;
 this->has_imageBackground = true;
}

void MyScene::drawBackground(QPainter * painter, const QRectF & rect )
{
    //if ( this->imageBackground != NULL )

    if ( this->has_imageBackground )
    {
        painter->save();
        painter->drawImage( rect, (this->imageBackground) );
        painter->restore();
        //this->has_imageBackground = false;
    }
}
