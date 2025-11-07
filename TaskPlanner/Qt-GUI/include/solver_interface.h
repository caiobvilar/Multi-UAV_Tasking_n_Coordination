#ifndef SOLVER_INTERFACE_H
#define SOLVER_INTERFACE_H

#include <QObject>
#include <QThread>
#include <QVector>
#include <QDebug>

#include "problem_definition.h"
#include "algo/taskallocation.h"

#include "algo/MinTurnDecomposition.h"
#include "algo/HorizontalAreaDecomposition.h"
#include "algo/TriangularDecomposition.h"
#include "algo/SweepPathCalipers.h"

enum METHOD
{
    MINTURN = 1,
    HORIZONTAL,
    TRIANGULAR
};

class solver_interface : public QObject
{
    Q_OBJECT
public:
    using DataType = std::vector<wykobi::polygon<double, 2>>;
    explicit solver_interface(const METHOD &method, pdfPtr pdf, QObject *parent = nullptr);

    void setup(QThread *cThread);
    void solve(ProblemDefinition &pdf);
    void clear()
    {
        m_decomposedArea.clear();
        m_sweepPaths.clear();
    }

    void getArea(vector<Point2D> &data, int index)
    {
        for (std::size_t i = 0; i < m_decomposedArea[index].size(); ++i)
            data.emplace_back(Point2D{m_decomposedArea[index][i].x, m_decomposedArea[index][i].y});
    }

    void getPath(vector<Point2D> &data, int index)
    {
        for (std::size_t i = 0; i < m_sweepPaths[index].size(); ++i)
        {
            data.emplace_back(Point2D{m_sweepPaths[index][i].x, m_sweepPaths[index][i].y});
        }
    }

signals:
    void polyX(QVector<double>);
    void polyY(QVector<double>);
    void setRobots(QVector<double>);

public slots:
    void solve();

private:
    METHOD m_method;
    DataType m_decomposedArea, m_sweepPaths;
    pdfPtr m_pdf;

protected:
    DataType option_parser(const wykobi::polygon<double, 2> roi, const vector<double> &uav_batteries, const METHOD &method) const;
    std::vector<double> length(const DataType &sweep_paths) const;

    void publish();
};

#endif // SOLVER_INTERFACE_H
