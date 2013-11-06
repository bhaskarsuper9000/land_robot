#ifndef MOUSE_H
#define MOUSE_H

#include <QGraphicsItem>

//! [0]
class Mouse : public QGraphicsItem
{
public:
    Mouse();
	Mouse(float a);
    QRectF boundingRect() const;
    QPainterPath shape() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
	qreal angle;
	void setAngle(float a);
protected:
    void advance(float x,float y,float z,float yaw);

private:
    
    qreal speed;
    qreal mouseEyeDirection;
    QColor color;
};
//! [0]
class Line : public QGraphicsItem
{

public:
Line();
Line(int x1,int y1,int x2,int y2);
	int x1,y1,x2,y2;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
	QRectF boundingRect() const;
};
#endif
