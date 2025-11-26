#include <QApplication>
#include <ros/ros.h>
#define LOGGER_ONCE
#include "mainwindow.h"
#include "solver.h"
#include "logger.h"

using namespace std;
int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    ros::init(argc, argv, "coverage_tasking");
    ros::NodeHandle nh;

    auto prob = make_shared<solver>();
    auto log = make_shared<logger>(prob);
    MainWindow window(prob);

    window.show();

    // log->Start();
    // prob->Start();

    int ret = a.exec();

    prob->STOP = true;
    // terminate worker thread
    {
        std::lock_guard<std::mutex> lk(prob->m);
        prob->ready = true;
        prob->logging = true;
        std::cout << "main() sent kill signals \n";
    }
    prob->cv.notify_all();
    return ret;
}