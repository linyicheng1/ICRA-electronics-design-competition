#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    system("gnome-terminal -x bash -c 'florence'&");
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowFlags(Qt::FramelessWindowHint);
    w.setWindowFlags(Qt::WindowStaysOnTopHint);

    w.show();

    return a.exec();
}
