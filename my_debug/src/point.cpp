#include <my_debug/point.h>

#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOption>

#include <math.h>

Point::Point()
{

}

QRectF Point::boundingRect() const
{
    qreal adjust = 0.5;
    return QRectF(-18 - adjust, -22 - adjust, 36 + adjust, 60 + adjust);
}

void Point::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
	painter->setBrush(Qt::black);
	painter->drawEllipse(0, 0, 8, 8);
	painter->setBrush(Qt::NoBrush);
	//painter->drawPath(path);
}
