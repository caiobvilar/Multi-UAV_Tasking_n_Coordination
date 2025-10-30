#ifndef DRAWINGBOARD_H
#define DRAWINGBOARD_H

#include "qcustomplot.h"
#include <QWidget>
#include <QDebug>
#include <QtCore>
#include <QVector>
#include <QColor>
#include <iostream>
#include <vector>
#include <QTimer>
#include <wykobi.hpp>
using namespace std;

class DrawingBoard:public QWidget
{
    Q_OBJECT
    using POINT = wykobi::point2d<double> ;
private:
    QCustomPlot *customPlot;
    QVector<double> qx, qy, ux, uy;
    QVector<QLineF> poly;
    int numLayers;
public:
    DrawingBoard(QCustomPlot *plot, QWidget *parent = nullptr);
    void addPoints(double x, double y);
    void plot();
    const QVector<QLineF> getPolygon();

    void setInitialPosition(vector<POINT>& candidates);
    void showPolygon(const wykobi::polygon<double, 2>&convex_hull_polygon, const QColor& color = Qt::red, bool connect = true);
    /**
     * @brief the key idea is to start a timer to gradually visualize the motion
     * @param traj
     * @param safe_distance
     * @param colors
     */
    void showMotion(const std::vector<wykobi::polygon<double, 2>>&traj,int safe_distance, const QVector<QColor>& colors);

private:
    int max_step_, current_step_,actionLayer_, safe_distance_;
    std::vector<wykobi::polygon<double, 2>> plans_;
    QVector<QColor> color_;
    QTimer *traj_timer_;

public slots:
     void clearData();
     void makePolygon();
     void motionUpdate();
     void repeatSim();

protected:
     void setPointSize(int size = 15, const QColor & color = Qt::darkBlue, int layer = 0);
     void setLineSize(QCPGraph* graph, const QColor& color);
     void plotPolygon(const QColor& color = Qt::blue);

signals:
};

#endif // DRAWINGBOARD_H
