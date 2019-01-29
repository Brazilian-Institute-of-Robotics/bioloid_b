#include <ros/ros.h>
#include "pid.hpp"

bir::PID::PID(double Kp, double Ki, double Kd, int interval)
    : _kP(Kp), _kI(Ki), _kD(Kd), _interval(interval)
{
    _pidSetPoint = 0,
    _pidLastError = 0,
    _pidIntError = 0;
    _enabled = false;
    _lastTime = ros::Time::now().toNSec();
    _maxWindUp = 1.0;
    _minWindUp = -1.0;
}

double bir::PID::run(double input){
    double P,I,D, PIDR;

    //* Proportional Action - 비례 *//
    double pidError = _pidSetPoint - (input); 
    unsigned long pidDeltaTime;
    if(!_enabled){ 
        pidDeltaTime = 0;
        _enabled = true;
    } else {
        pidDeltaTime = ros::Time::now().toNSec() - _lastTime;
    }

    _lastTime = ros::Time::now().toNSec();
    
    //* Integral Action - 적분 *//
    _pidIntError += ( (pidError + _pidLastError)/2 * pidDeltaTime * 0.000000001);
    if(pidError == 0) _pidIntError = 0;
    if(_pidIntError > _maxWindUp){
        _pidIntError = _maxWindUp;
    } else if (_pidIntError < _minWindUp){
        _pidIntError = _minWindUp;
    }
    
    //* Diferencial Action - 미분*//
    double pidDifError; 
    if(pidDeltaTime) {
        pidDifError = (pidError - _pidLastError)  / (pidDeltaTime * 0.000000001);
    } else {
        pidDifError = 0;
    }

    _pidLastError = pidError;

    P = (pidError) * _kP;
    I = _pidIntError * _kI;
    D = (pidDifError) * _kD;

    PIDR = P + I + D;
    
    return (PIDR);
}

void bir::PID::setSetpoint(double pidSetPoint){ //Set the Reference to PID
    _pidSetPoint = pidSetPoint;
}