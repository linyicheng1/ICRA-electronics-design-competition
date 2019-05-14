#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "iostream"
#include "stdlib.h"
#include <QProcess>
#include <string>
using namespace std;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    num='0';
    ui->setupUi(this);
}
MainWindow::~MainWindow()
{
    delete ui;
}
/*
 * sale Robot
 * gmapping--------start the gmapping and usb program
 * save map -------save the map(more than one time)
 * stop slam-------stop the gmapping
 * begin-----------begin the final program
 * end-------------stop the final program
 * restartUSB -----start usb communicate
 * */
void MainWindow::on_gmapping_clicked()
{
    std::cout<<"gmapping begin !!"<<std::endl;
     system("gnome-terminal -x bash -c 'source /home/cv/Desktop/linyicheng/ydlidar/build/devel/setup.bash;roslaunch ydlidar view_gmapping.launch '&");
}
void MainWindow::on_save_map_clicked()
{
    cout<<"save map"<<endl;
//    for(int i=0;i<5;i++)
//    {
//        std::cout<<"save map "<<vi<<std::endl;
        //@TODO save map
//        QProcess *proc = new QProcess;
        system("gnome-terminal -x bash -c 'source /opt/ros/kinetic/setup.bash ;cd /home/cv/Desktop/linyicheng/RoboRTS/tools/map&&rosrun map_server map_saver '&");
//    }
}

void MainWindow::on_end_gmapping_clicked()
{
    system("gnome-terminal -x bash -c 'sudo route del default gw 192.168.1.1;sudo route add default gw 192.168.43.0'&");

}

void MainWindow::on_begin_clicked()
{
    system("gnome-terminal -x bash -c 'source /opt/ros/kinetic/setup.bash&&source /home/cv/Desktop/linyicheng/ydlidar/build/devel/setup.bash;cd /home/cv/Desktop/linyicheng/RoboRTS&&export ROBORTS_PATH=${PWD}&& >>~/.bashrc&&source ~/.bashrc;cd /home/cv/Desktop/linyicheng/RoboRTS/tools/script&&./run.sh -s'&");
}

void MainWindow::on_end_clicked()
{
    system("gnome-terminal -x bash -c 'source /opt/ros/kinetic/setup.bash&&source /home/cv/Desktop/linyicheng/ydlidar/build/devel/setup.bash;cd /home/cv/Desktop/linyicheng/RoboRTS&&export ROBORTS_PATH=${PWD}&& >>~/.bashrc&&source ~/.bashrc;cd /home/cv/Desktop/linyicheng/RoboRTS/build/modules/driver/serial;./serial_com_node'&");
//system("gnome-terminal -x bash -c 'cd Desktop/linyicheng/RoboRTS/build/modules/driver/serial;./serial_com_node'&");
}

void MainWindow::on_restart_clicked()
{

     system("gnome-terminal -x bash -c 'source /opt/ros/kinetic/setup.bash ;cd /home/cv/Desktop/linyicheng/TCP_Communicate/build&&./TCP_Communicate'&");
}

void MainWindow::on_star_clicked()
{
    string name;
    char name_char[146];
    name.insert(0,"gnome-terminal -x bash -c 'source /opt/ros/kinetic/setup.bash ;cd /home/cv/Desktop/linyicheng/RoboRTS/build/modules/driver/camera;./camera_node '&");
    name.insert(144,&num);
    strcpy(name_char,name.c_str());
    num++;
    system(name_char);
     cout<<"star the position !!"<<name<<std::endl;
}
