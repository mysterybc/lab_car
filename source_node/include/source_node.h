#pragma once

#include "ros/ros.h"

#define pi 3.14

enum class State{
    NOT_CONFIG    = 0,
    HAVE_CONFIG   = 1,
    CONNECTING    = 2,
    CONNECTED     = 3,
    RUNNING       = 4,
    PAUSED        = 5,
    ERROR_RUNNING = 6,
    EXIT          = 7,
    STOP          = 8
};

enum class Cmd{
    START   = 0,
    PAUSE   = 1,
    STOP    = 2,
    RESUME  = 3,
    EXIT    = 4,
};

enum class Error{
    NO_ERROR = 0,
    CONFIG_FAILED = 1
};

