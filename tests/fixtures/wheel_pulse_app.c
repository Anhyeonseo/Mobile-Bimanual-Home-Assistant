#define main unused_inspection_main
#include "mobile_inspection_app.c"
#undef main
#include "mobile_wheel_pulse.h"
static unsigned pulses, bases;
bool MobileWheelPulse_Consumed(void) {return pulses!=0;}
void MobileWheelPulse_Run(uint8_t id,int direction,MobileWheelPulseResult *x) {
    assert(id==7&&direction==1);pulses++;
    *x=(MobileWheelPulseResult){.id=id,.stage="TEST"};
}
void MobileWheelPulse_RunBase(char motion,MobileBasePulseResult *x) {
    assert(motion=='F');bases++;*x=(MobileBasePulseResult){.motion=motion,.stage="TEST"};
}
int main(void) {
    SingleArmApp_Init(&host,&servo);
    assert(reads==0&&pulses==0&&strstr(output,"mobile-wheel-pulse-v4"));output[0]=0;
    feed("INFO\n");assert(reads==0&&pulses==0);output[0]=0;
    feed("PULSE 10 +\nPULSE 7 + extra\nPULSE 7 1\nPULSE 7 +xxxxxxxxxxxxxxxx\nPULSE 7\001 +\nPREPARE_WHEELS\n");
    assert(reads==0&&pulses==0);output[0]=0;
    feed("BASE X\nBASE F extra\nBASE 7\n");assert(bases==0);output[0]=0;
    feed("BASE F\n");assert(bases==1&&pulses==0&&reads==0);output[0]=0;
    feed("PULSE 7 +\r\n");assert(pulses==1&&strstr(output,"pulse_ok"));
    return 0;
}
