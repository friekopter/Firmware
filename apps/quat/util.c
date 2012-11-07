#include "util.h"


void *aqCalloc(size_t count, size_t size) {
    //heapUsed += count * size;
    return calloc(count, size);
}



int constrainInt(int i, int lo, int hi) {
    if (i < lo)
       return	lo;
    if (i > hi)
       return	hi;
    return i;
}

float constrainFloat(float i, float lo, float hi) {
    if (i < lo)
       return	lo;
    if (i > hi)
       return	hi;
    return i;
}


void utilFilterReset(utilFilter_t *f, float setpoint) {
    f->z1 = setpoint;
}

void utilFilterReset3(utilFilter_t *f, float setpoint) {
    utilFilterReset(&f[0], setpoint);
    utilFilterReset(&f[1], setpoint);
    utilFilterReset(&f[2], setpoint);
}

void utilFilterInit(utilFilter_t *f, float dt, float tau, float setpoint) {
    f->tc = dt / tau;
    utilFilterReset(f, setpoint);
}

void utilFilterInit3(utilFilter_t *f, float dt, float tau, float setpoint) {
    utilFilterInit(&f[0], dt, tau, setpoint);
    utilFilterInit(&f[1], dt, tau, setpoint);
    utilFilterInit(&f[2], dt, tau, setpoint);
}

inline float utilFilter3(utilFilter_t *f, float signal) {
    return utilFilter(&f[0], utilFilter(&f[1], utilFilter(&f[2], signal)));
}

inline float utilFilter(utilFilter_t *f, float signal) {
    register float z1;

    z1 = f->z1 + (signal - f->z1) * f->tc;

    f->z1 = z1;

    return z1;
}
