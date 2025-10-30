#include <QApplication>
#define V_REP_INEGRATION
#include "mainwindow.h"
#include "solver.h"
#include "logger.h"

using namespace std;
int main(int argc, char *argv[]) {

    auto prob = make_shared<solver>();
    auto log = make_shared<logger>(prob);
    QApplication a(argc, argv);
    MainWindow w(prob->getPtr());

    log->Start();
    prob->Start();

    w.show();
    int ret =  a.exec();

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