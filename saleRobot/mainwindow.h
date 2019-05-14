#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    char num;
private slots:
    void on_gmapping_clicked();

    void on_save_map_clicked();

    void on_end_gmapping_clicked();

    void on_begin_clicked();

    void on_end_clicked();

    void on_restart_clicked();

    void on_star_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
