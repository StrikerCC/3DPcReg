//
// Created by cheng on 9/8/21.
//
#include "utils.h"

bool CloseEnough(float num1, float num2) {
    return std::abs(num1-num2) < 0.001;
}

bool CloseEnough(double num1, double num2) {
    return CloseEnough((float) num1, (float) num2);
}

bool CloseEnough(int num1, int num2) {
    return CloseEnough((float) num1, (float) num2);
}