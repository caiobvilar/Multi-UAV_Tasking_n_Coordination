#include "solver_interface.h"

solver_interface::solver_interface(const METHOD &method, pdfPtr pdf, QObject *parent) : m_method(method), m_pdf(pdf)
{
}

void solver_interface::setup(QThread *cThread)
{
    connect(cThread, SIGNAL(started()), this, SLOT(solve()));
}

void solver_interface::solve(ProblemDefinition &pdf)
{
    try
    {
        qDebug() << "[Solver Interface]: process started";
        clear();
        m_decomposedArea = option_parser(pdf.roi, pdf.capacities(), m_method);
        SweepPathCalipers path(pdf.capacities(), pdf.roi);
        m_sweepPaths = path.solve(m_decomposedArea, true);
        publish();
    }
    catch (std::exception &e)
    {
        qDebug() << e.what();
    }
}

void solver_interface::solve()
{
    try
    {
        qDebug() << "[Solver Interface]: thread started with " << m_pdf->size();
        clear();
        m_decomposedArea = option_parser(m_pdf->roi, m_pdf->capacities(), m_method);
        SweepPathCalipers path(m_pdf->capacities(), m_pdf->roi);
        m_sweepPaths = path.solve(m_decomposedArea, true);
        publish();
    }
    catch (std::exception &e)
    {
        qDebug() << e.what();
    }
}

solver_interface::DataType solver_interface::option_parser(const wykobi::polygon<double, 2> roi, const vector<double> &capacities, const METHOD &method) const
{
    vector<wykobi::polygon<double, 2>> result;
    switch (method)
    {
    case MINTURN:
    {
        //            cout <<"[Method]: MINTURN" << endl;
        TaskPlanning::MinTurnDecomposition algo(capacities);
        result = algo.solve(roi);
        break;
    }
    case HORIZONTAL:
    {
        //            cout <<"[Method]: HORIZONTAL" << endl;
        TaskPlanning::HorizontalAreaDecomposition algo(capacities);
        result = algo.solve(roi);
        break;
    }
    case TRIANGULAR:
        //            cout <<"[Method]: TRIANGULAR" << endl;
        TaskPlanning::TriangularDecomposition algo(capacities);
        result = algo.solve(roi);
        break;
    }

    //    assert(result.size() == capacities.size() && " result does not comply");

    for (std::size_t i = 0; i < result.size(); ++i)
    {
        auto polygon = result[i];
        std::vector<wykobi::point2d<double>> convex_hull;
        wykobi::algorithm::convex_hull_melkman<wykobi::point2d<double>>(
            polygon.begin(),
            polygon.end(),
            std::back_inserter(convex_hull));
        wykobi::polygon<double, 2> convex_hull_polygon = wykobi::make_polygon<double>(convex_hull);
        result[i] = convex_hull_polygon;
    }
    return result;
}

std::vector<double> solver_interface::length(const solver_interface::DataType &sweep_paths) const
{
    std::vector<double> res;
    if (m_method == TRIANGULAR)
        return res;

    int count = sweep_paths.size();
    res.resize(count);
    // computing path length
    double total_path_length = 0;
    for (auto &poly : sweep_paths)
    {
        double path_length = 0;
        for (int i = 0; i < poly.size() - 1; ++i)
        {
            arma::vec2 a{poly[i].x, poly[i].y};
            arma::vec2 b{poly[i + 1].x, poly[i + 1].y};
            path_length += arma::norm(a - b);
        }
        int index = (int)sweep_paths.size() - count--;
        total_path_length += path_length;
        res[index] = path_length;
        printf("[PathLength] %d : %lf m\n", index, path_length);
    }
    printf("[PathLength] total path length : %lf m\n", total_path_length);
    return res;
}

void solver_interface::publish()
{

    qDebug() << "[solver interface] publishing messages";
    for (int i = 0; i < m_pdf->size(); ++i)
    {
        std::vector<Point2D> area;
        getArea(area, i);
        QVector<double> x, y;
        for (auto &p : area)
        {
            auto pp = p / 100.0;
            x.push_back(pp.x);
            y.push_back(pp.y);
        }
        // we only need middle polygon to split the target region
        if (i % 2 != 0)
        {
            emit polyX(x);
            emit polyY(y);
        }

        m_pdf->addDecomposedROI(area);
    }
    // alocate task
    TaskAllocation tasks(*m_pdf);
    QVector<double> allocations;
    for (int i = 0; i < m_pdf->size(); ++i)
    {
        auto alocat = tasks(i).second;
        std::cout << "[Allocation] Robot " << char(i + 'A') << " = " << alocat << std::endl;
        allocations.push_back(alocat.x);
        allocations.push_back(alocat.y);
    }
    emit setRobots(allocations);
}
