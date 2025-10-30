#include "drawingboard.h"

DrawingBoard::DrawingBoard(QCustomPlot *plot, QWidget *parent):QWidget(parent),customPlot(plot)
{


    customPlot->setBackground(Qt::GlobalColor::transparent);
    customPlot->setAttribute(Qt::WA_OpaquePaintEvent, false);

    customPlot->xAxis->setRange(0, 100);
    customPlot->yAxis->setRange(0, 100);
    customPlot->xAxis->setTickLabelColor(Qt::black);
    customPlot->yAxis->setTickLabelColor(Qt::black);

    customPlot->addGraph();
    setPointSize();
    numLayers = 1;
    traj_timer_ = new QTimer();
    connect(traj_timer_,SIGNAL(timeout()),this, SLOT(motionUpdate()));


}

void DrawingBoard::addPoints(double x, double y)
{
//    customPlot->graph(0)->addData(x,y);
    qx.append(x);
    qy.append(y);
    plot();
}

void DrawingBoard::plot()
{
    customPlot->graph(0)->setData(qx, qy);
    customPlot->replot();
    //    customPlot->update();
}

const QVector<QLineF> DrawingBoard::getPolygon()
{
    return poly;
}

void DrawingBoard::setInitialPosition(vector<POINT>& candidates)
{
    ux.clear(); uy.clear();
    copy(qx.begin(), qx.end(), back_inserter(ux));
    copy(qy.begin(), qy.end(), back_inserter(uy));
    qx.clear();
    qy.clear();
    for(auto &c:candidates)
    {
        qx.append(c.x);
        qy.append(c.y);
    }

    setPointSize(15,Qt::green);
    plot();


}

void DrawingBoard::clearData()
{
    // stopping timer
    max_step_ = current_step_;
    traj_timer_->stop();


//    qDebug()<< "clear button is pressed";
    qx.clear();
    qy.clear();
    poly.clear();
    for(int j = numLayers; j>0; j--)
    {
        customPlot->removeGraph(j);

    }
    plot();
    setPointSize();
    numLayers = 1;
//    qDebug()<<"num of layers "<< numLayers;
}

void DrawingBoard::makePolygon()
{

    if(qx.empty())return;
    // connect pair of points to create a line

    int N = qx.size()-1;
    for(int i=0; i<N; ++i)
    {
        QLineF l(QPointF(qx[i], qy[i]), QPointF(qx[i+1], qy[i+1]));
        poly.append(l);
    }
    // connect last point to first
    QLineF lastLine(QPointF(qx[N], qy[N]), QPointF(qx[0], qy[0]));
    poly.append(lastLine);
    plotPolygon();

}

void DrawingBoard::setPointSize(int size, const QColor& color, int layer)
{
    QCPScatterStyle myScatter;
      myScatter.setShape(QCPScatterStyle::ssCircle);
      myScatter.setPen(QPen(color));
      myScatter.setBrush(color);
      myScatter.setSize(size);
      customPlot->graph(layer)->setLineStyle(QCPGraph::lsNone);
      customPlot->graph(layer)->setScatterStyle(myScatter);

}

void DrawingBoard::setLineSize(QCPGraph* graph, const QColor& color)
{
    QPen myPen;
    int seriesWt = 4;
    myPen = graph->pen();
    myPen.setWidth(seriesWt);
    myPen.setBrush(color);
    graph->setPen(myPen);



}

void DrawingBoard::plotPolygon(const QColor& color)
{
    // plot a line in each layer

    for(auto &l:poly)
    {
        QVector<double> x{l.p1().x(),l.p2().x()};
        QVector<double> y{l.p1().y(),l.p2().y()};
        customPlot->addGraph();
        setLineSize(customPlot->graph(numLayers), color);
        customPlot->graph(numLayers++)->setData(x, y);
        customPlot->replot();
    }


}

void DrawingBoard::showPolygon(const wykobi::polygon<double, 2>&convex_hull_polygon, const QColor& color, bool connect) {
    poly.clear();
    int N = convex_hull_polygon.size()-1;
    for(int i = 0; i<N; ++i)
    {
        auto A = convex_hull_polygon[i];
        auto B = convex_hull_polygon[i + 1];
        QLineF l(QPointF(A.x, A.y), QPointF(B.x, B.y));
        poly.push_back(l);

    }
    // connect last point to first
    if(connect)
    {
        N = poly.size() - 1;
        QLineF lastLine(poly[N].p2(), poly[0].p1());
        poly.append(lastLine);
    }
    plotPolygon(color);

}

void DrawingBoard::showMotion(const std::vector<wykobi::polygon<double, 2>> &traj, int safe_distance, const QVector<QColor>& colors) {


    int N = traj.size();
    max_step_ = current_step_ = 0;
    actionLayer_ = numLayers;
    safe_distance_ = safe_distance;
    for (int i = 0; i < N; ++i) {
        customPlot->addGraph();
        numLayers++;
        max_step_ = max(int(traj[i].size()),max_step_);
    }
    plans_.clear();
    color_.clear();
    std::copy(traj.begin(),traj.end(),back_inserter(plans_));
    std::copy(colors.begin(),colors.end(),back_inserter(color_));
    qDebug()<<"[gui]: showing motion for steps "<< max_step_;
    traj_timer_->start(100);


}

void DrawingBoard::motionUpdate() {

    if(max_step_<=current_step_)
    {
        traj_timer_->stop();
        return;
    }
    int N = plans_.size();
    for(int robot =0; robot< N; ++robot)
    {
        int step = min(int(plans_[robot].size()-1),current_step_);
        auto J = plans_[robot][step];
        QVector<double>xx,yy;
        xx.push_back(J.x);
        yy.push_back(J.y);
        setPointSize(10, color_[robot], actionLayer_ + robot);
        customPlot->graph(actionLayer_ + robot)->setData(xx, yy);
//        qDebug()<<"[step]:"<< step <<" x = " << J.x << " y=  "<< J.y;

    }
    customPlot->replot();
    // collision detect
    for(int robot =0; robot< N-1; ++robot) {
        int step = min(int(plans_[robot].size()-1),current_step_);
        auto J1 = plans_[robot][step];
        int step1 = min(int(plans_[robot+1].size()-1),current_step_);
        auto J2 = plans_[robot+1][step1];
        if(distance(J1, J2)<safe_distance_)
        {
            current_step_ = max_step_;
            break;
        }
    }
    ++current_step_;
}

void DrawingBoard::repeatSim() {

    QVector<double> qx_, qy_;
    copy(ux.begin(), ux.end(), back_inserter(qx_));
    copy(uy.begin(), uy.end(), back_inserter(qy_));
    clearData();
    copy(qx_.begin(), qx_.end(), back_inserter(qx));
    copy(qy_.begin(), qy_.end(), back_inserter(qy));
    plot();

}

