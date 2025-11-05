#include "background.h"

#include <QThread>

Background::Background(QObject *parent) : QObject(parent)
{
    m_cmdButton = DEFAULT;

    m_pdf = std::make_shared<ProblemDefinition>();
    m_robots = m_pdf->flattenCoords();
    m_solver = new solver_interface(MINTURN, m_pdf, this);

    connect(m_solver, SIGNAL(polyX(QVector<double>)), this, SLOT(setLineX(QVector<double>)));
    connect(m_solver, SIGNAL(polyY(QVector<double>)), this, SLOT(setLineY(QVector<double>)));
    connect(m_solver, SIGNAL(setRobots(QVector<double>)), this, SLOT(setRobots(QVector<double>)));
}

Background::~Background()
{
    //    delete m_solver;
}

QString Background::cmdButton()
{
    if (m_cmdButton == CLEAR)
        return "clear";
    else
        return "okay";
}

void Background::setcmdButton(const QString &cmdButton)
{
    if (cmdButton == "clear")
    {
        m_cmdButton = CLEAR;
        m_mousePoints.clear();
        setRobots(m_pdf->flattenCoords());
    }
    else
    {
        m_cmdButton = OKAY;
        //        qDebug() << cmdButton;

        if (m_mousePoints.size() < 2)
            return;

        m_pdf->addInstance(m_mousePoints);
        // pass them to solver
        try
        {
            m_cThread = new QThread;
            m_solver->setup(m_cThread);
            m_solver->moveToThread(m_cThread);
            m_cThread->start();
        }
        catch (std::exception &e)
        {
            qDebug() << e.what();
        }

    } // --------------------------------------------- X --------------------------------------------
}

QString Background::mousePoint()
{
    return "";
}

void Background::setmousePoint(const QString &mousePoint)
{
    auto data = mousePoint.split(",");
    auto point = Point2D{data[0].toDouble(), data[1].toDouble()};
    m_mousePoints.emplace_back(point * 100);
    //    qDebug() << mousePoint;
}

QVector<double> Background::drawLineX()
{
    return m_lineX;
}

QVector<double> Background::drawLineY()
{
    return m_lineY;
}

QVector<double> Background::drawRobots()
{
    return m_robots;
}

void Background::setLineX(const QVector<double> &line)
{
    qDebug() << "setLineX received " << line;
    m_lineX.clear();
    std::copy(line.begin(), line.end(), std::back_inserter(m_lineX));
    emit drawLineXChanged();
}

void Background::setLineY(const QVector<double> &line)
{
    qDebug() << "setLineY received " << line;
    m_lineY.clear();
    std::copy(line.begin(), line.end(), std::back_inserter(m_lineY));
    emit drawLineYChanged();
}

void Background::setRobots(const QVector<double> &positions)
{
    m_robots.clear();
    std::copy(positions.begin(), positions.end(), std::back_inserter(m_robots));
    emit drawRobotsChanged();
}
