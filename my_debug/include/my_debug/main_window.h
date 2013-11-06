#ifndef MAINWINDOW_H 
#define MAINWINDOW_H
#include <QtGui>
#include <math.h>
#include <QLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include <QGraphicsScene>
#include <QPainter>
#include <QMainWindow>
#include <QStyleOption>
#include <QTimer>
#include<my_debug/mouse.h>
#include<my_debug/input.h>
#include<std_msgs/String.h>
#include<std_msgs/Float32.h>
#include <ros/ros.h>
#include <QObject>
#include <QGridLayout>

class MainWindow :public QMainWindow
{
	Q_OBJECT

	public:
		QWidget* mainWidget;
		QGraphicsScene* scene;
		QGridLayout* mainLayout;
		Mouse* mouse;
		QGraphicsView* view;
		ros::Subscriber sub ;
		QTimer* timer;		
	
		MainWindow();
		~MainWindow();
		
		void chatterCallback(const my_debug::input &msg);
		float xin,yin,yawin,xf,yf,yawf;
		int argc; char** argv;
		ros::NodeHandle n;

	public slots :
		void spinOnce();
};

#endif
