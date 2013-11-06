#ifndef POINT_H
#define POINT_H

#include <QGraphicsItem>

class Point : public QGraphicsItem
{
public:
Point();
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
	QRectF boundingRect() const;

};
//! [0]

#endif
