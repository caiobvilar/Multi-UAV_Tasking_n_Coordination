#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "drawingboard.h"
#include "problem.h"
#include <iostream>
#include <memory>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(std::shared_ptr<problem> instance, QWidget *parent = nullptr);
    ~MainWindow();
public slots:
    void clickedGraph(QMouseEvent *event);

private slots:
    void on_comboBox_currentIndexChanged(int index);


    void on_comboPlanner_currentIndexChanged(int index);

    void on_pushButton_clicked();

    void on_solveButton_clicked();

    void on_clearButton_clicked();

    void on_actionImage_dir_triggered();

    void solution_check();
private:
    Ui::MainWindow *ui;
    DrawingBoard *draw;
    QStringList backgrounds;
    int map_index;
    std::shared_ptr<problem>prob_instance;
    QTimer timer;
    
};



#endif // MAINWINDOW_H
