//
// Created by robor on 4/22/2020.
//

#ifndef AREACOVERAGE_SOLVER_H
#define AREACOVERAGE_SOLVER_H

#include <iostream>
#include <thread>
#include "problem.h"
#include <QDebug>

class solver :public problem{
public:
    solver();
    ~solver();
    void solve();
    void Start();
    bool STOP, logging;

private:
    std::thread solver_thread;
};


#endif //AREACOVERAGE_SOLVER_H
