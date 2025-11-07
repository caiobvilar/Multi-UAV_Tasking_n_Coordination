#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <vector>
using namespace std;

MainWindow::MainWindow(std::shared_ptr<problem> instance, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), prob_instance(instance)
{
    ui->setupUi(this);
    draw = new DrawingBoard(ui->widget);

    //    connect(ui->clearButton, SIGNAL(clicked()),draw,SLOT(clearData()));
    connect(&timer, SIGNAL(timeout()), this, SLOT(solution_check()));
    connect(ui->makeButton, SIGNAL(clicked()), draw, SLOT(makePolygon()));
    connect(ui->widget, SIGNAL(mousePress(QMouseEvent *)), SLOT(clickedGraph(QMouseEvent *)));
    connect(ui->repeatButton, SIGNAL(clicked()), draw, SLOT(repeatSim()));

    ui->comboBox->setVisible(false);
    timer.start(1000);
    QStringList planner;
    planner << "MinTurn" << "Triangle" << "Horizontal";
    ui->comboPlanner->addItems(planner);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::clickedGraph(QMouseEvent *event)
{
    QPoint point = event->pos();
    double x, y;
    x = ui->widget->xAxis->pixelToCoord(point.x());
    y = ui->widget->yAxis->pixelToCoord(point.y());
    draw->addPoints(x, y);
    {
        std::lock_guard<std::mutex> lk(prob_instance->m);
        prob_instance->add_vertex(x, y);
    }
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    qDebug() << backgrounds[index];
    ui->map->setStyleSheet(backgrounds[index]);
    map_index = index;
}

void MainWindow::on_pushButton_clicked()
{
    // get UAV parameters
    int num_uavs = ui->uav_numbers->text().toInt();
    int safety = ui->safety->text().toInt();
    QString batteries, footprints, velocities;
    batteries = ui->batteries->text();
    footprints = ui->sensor_footprints->text();
    velocities = ui->velocities->text();

    // covertion
    auto csvParser = [](QString &data)
    {
        QStringList items = data.split(",");
        vector<double> result;
        for (auto &d : items)
            result.push_back(d.toDouble());
        //        copy(result.begin(), result.end(), ostream_iterator<double>(cout," "));
        return result;
    };
    prob_instance->add_batteries(csvParser(batteries));
    prob_instance->add_footprints(csvParser(footprints));
    prob_instance->add_velocities(csvParser(velocities));

    // update problem instance
    prob_instance->formulate(num_uavs, safety);

    // visualization
    draw->setInitialPosition(prob_instance->initial_positions_);
    draw->showPolygon(prob_instance->roi_);
}

void MainWindow::on_actionImage_dir_triggered()
{
    // explore filesystem
    QString dirname = QFileDialog::getOpenFileName(
        this,
        tr("All files"),
        ".",
        "all files (*.*)");
    QMessageBox::information(this, tr("All files"), dirname);

    QFileInfo dirInfo(dirname);
    // get all image files
    QDir directory = dirInfo.dir();
    QStringList images = directory.entryList(QStringList() << "*.jpg" << "*.JPG" << "*.png" << "*.PNG", QDir::Files);

    qDebug() << images;
    ui->comboBox->setVisible(true);

    QStringList maps;
    for (auto filename : images)
    {
        QString bkg = "background-image:url(); background-position: center; ";
        bkg.insert(21, directory.absolutePath() + "/" + filename);
        backgrounds << bkg;

        QFileInfo file(filename);
        maps << file.fileName();
    }
    ui->comboBox->addItems(maps);
    backgrounds.clear();
}

void MainWindow::on_clearButton_clicked()
{
    draw->clearData();
    prob_instance->reset();
    qDebug() << "resetting";
}

void MainWindow::solution_check()
{
    if (!prob_instance->processed)
        return;
    qDebug() << "[gui] solution recived " << prob_instance->decompose_areas.size();
    int index = 0;
    QVector<QColor> colors;
    colors.push_back(Qt::darkCyan);
    colors.push_back(Qt::gray);
    colors.push_back(Qt::magenta);
    colors.push_back(Qt::yellow);
    colors.push_back(Qt::darkRed);
    colors.push_back(Qt::red);
    colors.push_back(Qt::green);
    colors.push_back(Qt::blue);

    QString vel = "";
    int num = prob_instance->veolcity_limits_.size();
    for (auto &v : prob_instance->veolcity_limits_)
    {
        vel += QString::number(v);
        if (--num > 0)
            vel += ",";
    }
    ui->velocities->setText(vel);

    for (const auto &area : prob_instance->decompose_areas)
    {
        if (area.size() >= 2)
        {
            draw->showPolygon(area, Qt::green);
            if (static_cast<size_t>(index) < prob_instance->sweep_paths.size() && prob_instance->sweep_paths[index].size() > 0)
            {
                draw->showPolygon(prob_instance->sweep_paths[index], colors[index % colors.size()], false);
            }
        }
        ++index;
    }
    prob_instance->processed = false;
}

void MainWindow::on_solveButton_clicked()
{
    // send data to the worker thread
    {
        std::lock_guard<std::mutex> lk(prob_instance->m);
        prob_instance->ready = true;
        std::cout << "[gui] problem is ready for solving\n";
    }
    prob_instance->cv.notify_all();
}

void MainWindow::on_executeButton_clicked()
{
    // send data to the worker thread
    prob_instance->execute();
}

void MainWindow::on_comboPlanner_currentIndexChanged(int index)
{
    prob_instance->plannerType = static_cast<PLANNER_TYPE>(index);
}
