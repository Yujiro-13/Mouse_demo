#ifndef MOTION_HPP
#define MOTION_HPP

#include <iostream>
#include "structs.hpp"

class Motion
{
    public:
        void run();
        void turn();
        void stop();
        void back();
        void slalom();
    protected:
};

#endif // MOTION_HPP