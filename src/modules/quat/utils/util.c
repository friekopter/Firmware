#include "util.h"

#define IMU_STATIC_STD		0.05f						// Standard deviation

uint32_t heapUsed, dataSramUsed;
uint32_t *ccmHeap[UTIL_CCM_HEAP_SIZE] __attribute__((section(".ccm")));

void *aqCalloc(size_t count, size_t size) {
    heapUsed += count * size;
    printf("heap used: %d\n",heapUsed);
    return calloc(count, size);
}
// allocates memory from 64KB CCM
void *aqDataCalloc(uint16_t count, uint16_t size) {
    uint32_t words;

    // round up to word size
    words = (count*size + sizeof(int)-1) / sizeof(int);

    if ((dataSramUsed + words) > UTIL_CCM_HEAP_SIZE) {
    	fprintf(stderr, "Out of data SRAM!\n");
    }
    else {
    	dataSramUsed += words;
    }

    return (void *)(ccmHeap + dataSramUsed - words);
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

// wait for lack of movement
void utilQuasiStatic(int n, float acc_x, float acc_y, float acc_z) {
    float stdX, stdY, stdZ;
    float vX[n];
    float vY[n];
    float vZ[n];
    int i, j;

    i = 0;
    j = 0;
    do {

	vX[j] = acc_x;
	vY[j] = acc_y;
	vZ[j] = acc_z;
	j = (j + 1) % n;

	if (i >= n) {
	    arm_std_f32(vX, n, &stdX);
	    arm_std_f32(vY, n, &stdY);
	    arm_std_f32(vZ, n, &stdZ);
	}

	i++;
    } while (i <= n || (stdX + stdY + stdZ) > IMU_STATIC_STD);
}
