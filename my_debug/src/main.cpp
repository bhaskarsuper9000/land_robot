#include <my_debug/mouse.h>
#include <my_debug/point.h>
#include <QtGui>
#include <math.h>
#include <QLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOption>
#include<ros/ros.h>
#include <my_debug/input.h>
#include <my_debug/main_window.h>

using namespace std;
float xin,yin,yawin,xf,yf,yawf;
Mouse* mouse = new Mouse;

int main(int argc, char **argv)
{
	QApplication app(argc, argv);
	cin >> xin >> yin >> yawin;ros::init(argc, argv, "listener");
	MainWindow *mainwindow  = new MainWindow;
	mainwindow->argc = argc; mainwindow->argv=argv;
	
    


	mainwindow->show();	
	return app.exec();
}
