#pragma once
#include<iostream>
#include<hg_hri/Gripper.h>
#include<hg_hri/Robot.h>
using namespace std;

class Modules{
public:
    Modules(){
        // this->robot = new Robot;
        this->gripper = new Gripper;
    }

    void run();
    void catch_obj();

    void track_next();

    void track_record();

    ~Modules(){
        // delete robot;
        delete gripper;
    }

private:
    // Robot *robot;
    Gripper *gripper;
};