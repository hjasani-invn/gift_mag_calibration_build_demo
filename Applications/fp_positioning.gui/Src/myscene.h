#ifndef MYSCENE_H
#define MYSCENE_H
#include <QGraphicsScene>
#include <QPainter>


class MyScene : public QGraphicsScene
{
public:
    MyScene(QObject *parent = NULL);
    virtual ~MyScene(){}
    void setBackground(QImage image);

protected:
   void drawBackground(QPainter * painter, const QRectF & rect );

private:
//    QImage *imageBackground;
    QImage imageBackground;
    bool   has_imageBackground;
};

#endif // MYSCENE_H
