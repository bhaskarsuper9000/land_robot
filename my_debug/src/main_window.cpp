#include<my_debug/main_window.h>

void MainWindow::chatterCallback(const my_debug::input& msg)
{
	ROS_INFO("I heard: %f %f %f %f",msg.x,msg.y,msg.z,msg.yaw);
	xf = msg.x,yf=msg.y,yawf=msg.yaw;
	Line l(xin,yin,xf,yf);
	xin = xf; yin=yf;yawin =yawf;
	mouse->setRotation(yawf);
	mouse->setPos(xf,yf);
	return;
}

void MainWindow::spinOnce()
{
	ros::spinOnce();
}   

MainWindow::~MainWindow(){
}
MainWindow::MainWindow()
{
	scene = new QGraphicsScene();
   	scene->setSceneRect(-300, -300, 600, 600);
    scene->setItemIndexMethod(QGraphicsScene::NoIndex);
	mainWidget= new QWidget;
	this->setCentralWidget(mainWidget);
	mainLayout = new QGridLayout();

	mouse = new Mouse;
	mouse->setRotation(yawin);
	mouse->setPos(xin,yin);
	scene->addItem(mouse);


 	view = new QGraphicsView(scene);
	mainLayout->addWidget(view,0,0);
	
	mainWidget->setLayout(mainLayout);
	sub = n.subscribe("chatter", 1,&MainWindow::chatterCallback,this);
   	timer= new QTimer;
    //QObject::connect(timer, SIGNAL(timeout()),this,SLOT(spinOnce()));
	connect(timer,SIGNAL(timeout()), this, SLOT(spinOnce())); 
	timer->setSingleShot(false);
    timer->start(100);
}
