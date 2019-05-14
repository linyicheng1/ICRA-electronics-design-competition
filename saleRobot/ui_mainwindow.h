/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *gmapping;
    QPushButton *save_map;
    QPushButton *begin;
    QPushButton *end;
    QPushButton *end_gmapping;
    QPushButton *restart;
    QPushButton *star;
    QMenuBar *menuBar;
    QMenu *menuSALERobot;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(257, 632);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gmapping = new QPushButton(centralWidget);
        gmapping->setObjectName(QStringLiteral("gmapping"));
        gmapping->setGeometry(QRect(50, 50, 150, 50));
        save_map = new QPushButton(centralWidget);
        save_map->setObjectName(QStringLiteral("save_map"));
        save_map->setGeometry(QRect(50, 125, 150, 50));
        begin = new QPushButton(centralWidget);
        begin->setObjectName(QStringLiteral("begin"));
        begin->setGeometry(QRect(50, 275, 150, 50));
        end = new QPushButton(centralWidget);
        end->setObjectName(QStringLiteral("end"));
        end->setGeometry(QRect(50, 350, 150, 50));
        end_gmapping = new QPushButton(centralWidget);
        end_gmapping->setObjectName(QStringLiteral("end_gmapping"));
        end_gmapping->setGeometry(QRect(50, 200, 150, 50));
        restart = new QPushButton(centralWidget);
        restart->setObjectName(QStringLiteral("restart"));
        restart->setGeometry(QRect(50, 425, 150, 50));
        star = new QPushButton(centralWidget);
        star->setObjectName(QStringLiteral("star"));
        star->setGeometry(QRect(50, 500, 150, 50));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 257, 22));
        menuSALERobot = new QMenu(menuBar);
        menuSALERobot->setObjectName(QStringLiteral("menuSALERobot"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuSALERobot->menuAction());

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        gmapping->setText(QApplication::translate("MainWindow", "gmapping", Q_NULLPTR));
        save_map->setText(QApplication::translate("MainWindow", "save map", Q_NULLPTR));
        begin->setText(QApplication::translate("MainWindow", "begin", Q_NULLPTR));
        end->setText(QApplication::translate("MainWindow", "end", Q_NULLPTR));
        end_gmapping->setText(QApplication::translate("MainWindow", "end gmapping", Q_NULLPTR));
        restart->setText(QApplication::translate("MainWindow", "restart", Q_NULLPTR));
        star->setText(QApplication::translate("MainWindow", "star", Q_NULLPTR));
        menuSALERobot->setTitle(QApplication::translate("MainWindow", "SALERobot", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
