#ifndef MANUFATURA_ADITIVA_PID_HPP
#define MANUFATURA_ADITIVA_PID_HPP
#include <ros/ros.h>

namespace bir{
    class PID{
        private:

            double _kP, _kI, _kD;
            double _pidSetPoint, 
                           _pidLastError,
                           _pidIntError;
            u_int64_t _lastTime;
            bool _enabled;
            double _maxWindUp, _minWindUp;

        public:
            int _interval;
            PID(double Kp, double Ki, double Kd, int interval);
            double run(double);
            void setSetpoint(double);
    };
}

#endif