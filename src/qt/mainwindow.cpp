#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <random>
double qt_time = 0;
double qt_time_dt =0.001;

typedef enum { JOINT_POS, JOINT_VEL, JOINT_TORQ ,EXT_WRENCH} ENUM_DISPLAY_FLAG;
int display_flag = JOINT_POS;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    GraphInit();
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::GraphInit()
{

    // include this section to fully disable antialiasing for higher performance:
    /*
    customPlot->setNotAntialiasedElements(QCP::aeAll);
    QFont font;
    font.setStyleStrategy(QFont::NoAntialias);
    customPlot->xAxis->setTickLabelFont(font);
    customPlot->yAxis->setTickLabelFont(font);
    customPlot->legend->setFont(font);
    */
    QVector<QColor> colors;
    colors.append(QColor(255, 0, 0));      // 빨간색
    colors.append(QColor(0, 255, 0));      // 녹색
    colors.append(QColor(0, 0, 255));      // 파란색
    colors.append(QColor(255, 165, 0));    // 오렌지색
    colors.append(QColor(128, 0, 128));    // 자주색
    colors.append(QColor(0, 128, 128));    // 청록색
    colors.append(QColor(255, 192, 203));  // 핑크색

    for(int i =0;i<JOINTNUM;i++){
        ui->widget_RealTimeGraph->addGraph();
        ui->widget_RealTimeGraph->graph(i)->setPen(QPen(colors.at(i)));
    }
    
 
    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");
    ui->widget_RealTimeGraph->xAxis->setTicker(timeTicker);
    ui->widget_RealTimeGraph->axisRect()->setupFullAxesBox();
    ui->widget_RealTimeGraph->yAxis->setRange(-3.141592*2.0, 3.141592*2.0);
    ui->widget_RealTimeGraph->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->widget_RealTimeGraph->legend->setVisible(true);
    ui->widget_RealTimeGraph->legend->setBrush(QBrush(QColor(255,255,255,150)));
    ui->widget_RealTimeGraph->axisRect()->insetLayout()->setInsetAlignment(0,Qt::AlignLeft|Qt::AlignTop);
    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->widget_RealTimeGraph->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->widget_RealTimeGraph->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->widget_RealTimeGraph->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->widget_RealTimeGraph->yAxis2, SLOT(setRange(QCPRange)));

    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    connect(&DataTimer, SIGNAL(timeout()), this, SLOT(RealtimeDataSlot()));
    connect(&PhysicsTimer, SIGNAL(timeout()), this, SLOT(RealtimePhysicsSlot()));
    //DataTimer.setTimerType(Qt::PreciseTimer);
    DataTimer.start(1); // Interval 0 means to refresh as fast as possible
    

}
void MainWindow::RealtimePhysicsSlot(){
    qt_time+=qt_time_dt;
}
void MainWindow::RealtimeDataSlot()
{
    static QTime time(QTime::currentTime());
    qsrand(QTime::currentTime().msecsSinceStartOfDay());

      // calculate two new data points:
      double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
      static double lastPointKey = 0;
    	//std::lock_guard<std::mutex> lock(g_pages_mutex); 
        for(int j =0;j<JOINTNUM;j++){
            switch(display_flag){
                case JOINT_POS: ui->widget_RealTimeGraph->graph(j)->addData(key, info.act.q[j]);break;
                case JOINT_VEL: ui->widget_RealTimeGraph->graph(j)->addData(key, info.act.q_dot[j]);break;
                case JOINT_TORQ: ui->widget_RealTimeGraph->graph(j)->addData(key, info.act.tau[j]);break;
                case EXT_WRENCH: if(j<6){ui->widget_RealTimeGraph->graph(j)->addData(key, info.F_sensor[j]);break;}
            }
        }
            
      ui->widget_RealTimeGraph->xAxis->setRange(key,5,Qt::AlignRight);
      ui->widget_RealTimeGraph->replot();

      // calculate frames per second:
      static double lastFpsKey;
      static int frameCount;
      ++frameCount;
      if (key-lastFpsKey > 1) // average fps over 2 seconds
      {
        ui->statusBar->showMessage(
              QString("%1 FPS, Total Data points: %2")
              .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
              .arg(ui->widget_RealTimeGraph->graph(0)->data()->size()+ui->widget_RealTimeGraph->graph(1)->data()->size())
              , 0);
        lastFpsKey = key;
        frameCount = 0;
      }
     
}

void MainWindow::on_pushButton_Position_clicked()
{
    display_flag = JOINT_POS;
    double MAX_POS = M_PI*2;
    ui->widget_RealTimeGraph->yAxis->setRange(-MAX_POS, MAX_POS);
}

void MainWindow::on_pushButton_Velocity_clicked()
{
    display_flag = JOINT_VEL;
    double MAX_VEL = 30;
    ui->widget_RealTimeGraph->yAxis->setRange(-MAX_VEL, MAX_VEL);
}
void MainWindow::on_pushButton_Torque_clicked()
{
    display_flag = JOINT_TORQ;
    double MAX_TORQ = 500;

    ui->widget_RealTimeGraph->yAxis->setRange(-MAX_TORQ, MAX_TORQ);
}

void MainWindow::on_pushButton_ExtWrench_clicked()
{
    std::random_device rd;
    std::mt19937 gen(rd()); // Mersenne Twister 19937 엔진 사용 (또는 다른 엔진 선택)
    std::uniform_real_distribution<double> dis(-10, 10.0); // -10에서 10 사이의 부동 소수점 수 생성
    display_flag=EXT_WRENCH;
    double MAX_WRENCH = 10;

    ui->widget_RealTimeGraph->yAxis->setRange(-MAX_WRENCH, MAX_WRENCH);
    //std::lock_guard<std::mutex> lock(g_pages_mutex); 
    info.act.F[0] = dis(gen);
    info.act.F[1] = dis(gen);
    info.act.F[2] = dis(gen);
    info.act.F[3] = dis(gen);
    info.act.F[4] = dis(gen);
    info.act.F[5] = dis(gen);    
}

