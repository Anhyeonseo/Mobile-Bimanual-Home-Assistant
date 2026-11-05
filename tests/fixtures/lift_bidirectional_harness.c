#define main legacy_main
#include "lift_teleop_harness.c"
#undef main
static unsigned seq;
static void drive_next(char motion){char p[60];snprintf(p,sizeof(p),"D 1234ABCD %08X %c",++seq,motion);packet(p);}
static void move_by(int delta){int p=(regs[56]|(regs[57]<<8));p=(p+delta+4096)%4096;regs[56]=p;regs[57]=p>>8;tick+=20;SingleArmApp_Process();}
static void arm_again(void){seq=0;packet("ARM 1234ABCD");assert(strstr(output,"\"type\":\"armed\""));output[0]=0;}
int main(int argc,char **argv){
    assert(argc==2);unsigned test=atoi(argv[1]);scenario=99;
    regs[3]=9;regs[4]=3;regs[5]=10;regs[33]=1;regs[55]=1;regs[56]=5;
    SingleArmApp_Init(&host,&servo);feed("INFO\n");assert(strstr(output,"lift-teleop-v5"));
    packet("ARM 1234ABCD");assert(writes==0&&strstr(output,"BOTTOM_REFERENCE_REQUIRED"));output[0]=0;
    packet("REF 76543210");assert(writes==0&&strstr(output,"BOTTOM_REFERENCED"));output[0]=0;
    if(test==4){SingleArmApp_Init(&host,&servo);packet("ARM 1234ABCD");assert(writes==0&&strstr(output,"BOTTOM_REFERENCE_REQUIRED"));return 0;}
    if(test==5){tick+=201;SingleArmApp_Process();assert(strstr(output,"ENCODER_CONTINUITY_LOST"));output[0]=0;feed("STATUS\n");assert(strstr(output,"\"reference_id\":0"));return 0;}
    arm_again();drive_next('U');assert(regs[46]==100&&regs[47]==128&&regs[40]==1);output[0]=0;
    if(test==0){
        move_by(-20);drive_next('Z');assert(strstr(output,"\"relative_counts\":-20"));feed("STOP\n");stopped();output[0]=0;
        move_by(-20);feed("STATUS\n");assert(strstr(output,"\"relative_counts\":-40"));output[0]=0;
        arm_again();drive_next('Z');assert(strstr(output,"\"relative_counts\":-40"));output[0]=0;
        drive_next('D');assert(strstr(output,"LOWER_GUARD"));stopped();
    }else if(test==1){
        for(unsigned i=0;i<300;i++){output[0]=0;move_by(-100);if(i<299)drive_next('U');}
        assert(strstr(output,"UPPER_GUARD"));stopped();
    }else if(test==2){
        for(unsigned i=0;i<10;i++){move_by(-100);drive_next('U');output[0]=0;}
        feed("STOP\n");stopped();output[0]=0;arm_again();drive_next('D');
        for(unsigned i=0;i<8;i++){output[0]=0;move_by(100);if(i<7)drive_next('D');}
        assert(strstr(output,"LOWER_GUARD"));stopped();
    }else if(test==3){
        move_by(-100);drive_next('D');assert(strstr(output,"LOWER_GUARD")||strstr(output,"DIRECTION_CHANGE_REQUIRES_STOP"));stopped();
    }else if(test==6){tick+=401;SingleArmApp_Process();stopped();assert(strstr(output,"HOST_TIMEOUT"));
    }else if(test==7){move_by(20);stopped();assert(strstr(output,"UNEXPECTED_REVERSE_MOTION"));
    }else if(test==8){feed("STOP\n");stopped();output[0]=0;move_by(-100);feed("STATUS\n");assert(strstr(output,"\"relative_counts\":-100"));
    }else if(test==9){HAL_UART_ErrorCallback(&host);SingleArmApp_Process();stopped();assert(strstr(output,"HOST_RX_ERROR")&&strstr(output,"\"reference_id\":0"));
    }else if(test==10){feed("STOP\n");stopped();output[0]=0;packet("REF 11111111");assert(strstr(output,"REFERENCE_NOT_ALLOWED"));
    }else if(test==11){feed("STOP\n");stopped();output[0]=0;packet("TEST 1234ABCD");seq=0;drive_next('U');for(unsigned i=0;i<70&&!strstr(output,"TEST_PULSE_COMPLETE");i++){tick+=20;drive_next('U');}stopped();assert(strstr(output,"TEST_PULSE_COMPLETE"));
    }else if(test==12){regs[33]=0;drive_next('U');assert(strstr(output,"MODE_CHANGED")&&strstr(output,"\"reference_id\":0"));
    }else if(test==13){scenario=4;drive_next('U');stopped();assert(strstr(output,"POSITION_READ_FAILED"));
    }else if(test==14){regs[40]=0;drive_next('U');assert(strstr(output,"TORQUE_CHANGED"));
    }else assert(0);
    puts(output);return 0;
}
