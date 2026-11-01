#include "single_arm_app.h"
#include "servo_bus.h"
#include "host_uart_rx.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static UART_HandleTypeDef host,servo;
static uint8_t regs[72];
static uint8_t *rx_byte;
static const char *delayed_input;
static unsigned writes,nonzero,scenario,injected;
static unsigned telemetry_reads;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
static void queue_bytes(const char *s);
static uint32_t tick;
static char output[32768];
static ServoBusDiagnostics diagnostics;
const ServoBusDiagnostics *ServoBus_GetDiagnostics(void){return &diagnostics;}
uint32_t HAL_GetTick(void){return tick;}
void HAL_Delay(uint32_t ms){
    tick+=ms;
    if(delayed_input){const char *s=delayed_input;delayed_input=NULL;queue_bytes(s);}
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n){
    assert(u==&host&&n==1);rx_byte=p;return HAL_OK;
}
void MobileServoOutput_OnUartError(UART_HandleTypeDef *u){(void)u;}
void MobileHoldOutput_OnUartError(UART_HandleTypeDef *u){(void)u;}
void MobileServoFeedback_OnUartError(UART_HandleTypeDef *u){(void)u;}
void ServoBus_HandleUartError(UART_HandleTypeDef *u){(void)u;}
void HostUartTx_OnError(UART_HandleTypeDef *u){(void)u;}
static void queue_bytes(const char *s){
    for(;*s;s++){assert(rx_byte);tick++;*rx_byte=(uint8_t)*s;HAL_UART_RxCpltCallback(&host);}
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t wait){
    (void)wait;assert(u==&host&&strlen(output)+n<sizeof(output));strncat(output,(const char *)p,n);return HAL_OK;
}
void ServoBus_Init(UART_HandleTypeDef *u,ServoStopRequestedFn a,ServoReadFailureFn b){assert(u==&servo&&!a&&!b);}
HAL_StatusTypeDef Servo_ReadData(uint8_t id,uint8_t addr,uint8_t n,uint8_t *v){
    assert(id==10);tick++;
    if(addr==56){
        assert(n==15);
        if(scenario==36 && telemetry_reads++){
            diagnostics.reason=SERVO_BUS_FAILURE_RX_TIMEOUT;return HAL_TIMEOUT;
        }
    }
    if((scenario==30||scenario==34) && nonzero && addr==33 && !injected){
        injected++;diagnostics.reason=SERVO_BUS_FAILURE_RX_TIMEOUT;diagnostics.hal_status=HAL_TIMEOUT;return HAL_TIMEOUT;
    }
    if(scenario==31 && nonzero && addr==40 && !injected){
        injected++;diagnostics.reason=SERVO_BUS_FAILURE_RX_TIMEOUT;diagnostics.hal_status=HAL_TIMEOUT;return HAL_TIMEOUT;
    }
    if(scenario==34 && injected && addr==56){
        diagnostics.reason=SERVO_BUS_FAILURE_UART;diagnostics.hal_status=HAL_ERROR;return HAL_ERROR;
    }
    if(scenario==4 && nonzero && addr==56 && !injected){injected++;return HAL_TIMEOUT;}
    if(scenario==5 && nonzero && addr==46 && !injected){injected++;return HAL_TIMEOUT;}
    if(scenario==6 && addr==55 && !regs[55] && !injected){injected++;return HAL_TIMEOUT;}
    if(scenario==11 && nonzero && addr==56){regs[58]=1;}
    memcpy(v,regs+addr,n);return HAL_OK;
}
HAL_StatusTypeDef Servo_WriteData(uint8_t id,uint8_t addr,const uint8_t *v,uint8_t n){
    assert(id==10);tick++;writes++;
    if(addr==33){assert(n==1&&v[0]==1&&regs[55]==0&&regs[40]==0);}
    else if(addr==55){assert(n==1&&v[0]<=1);}
    else if(addr==40){assert(n==1&&v[0]<=1);if(v[0])assert(regs[33]==1&&regs[46]==0&&regs[47]==0);}
    else {
        assert(addr==46&&n==2&&v[1]==0&&(v[0]==0||v[0]==100));
        assert(regs[33]==1); /* never write a position-mode goal */
        if(v[0]){assert(regs[40]==1);nonzero++;}
        regs[40]=1; /* observed zero-speed write torque side effect */
        if(scenario==7&&v[0]&&!injected){injected++;tick+=450;}
    }
    memcpy(regs+addr,v,n);
    if(scenario==24&&addr==33)return HAL_TIMEOUT;
    return HAL_OK;
}
static unsigned crc(const char *s){unsigned c=65535;for(;*s;s++){c^=(unsigned char)*s<<8;for(unsigned i=0;i<8;i++)c=((c&32768)?(c<<1)^0x1021:c<<1)&65535;}return c;}
static void feed(const char *s){queue_bytes(s);do{SingleArmApp_Process();}while(HostUartRx_Count());}
static void packet(const char *prefix){char line[80];snprintf(line,sizeof(line),"%s %04X\n",prefix,crc(prefix));feed(line);}
static void stopped(void){assert(regs[46]==0&&regs[47]==0&&regs[40]==1);}
int main(int argc,char **argv){
    assert(argc==2);scenario=atoi(argv[1]);regs[3]=9;regs[4]=3;regs[5]=10;regs[55]=1;regs[56]=200;
    if(scenario==12)tick=UINT32_MAX-200;
    if(scenario==21){regs[56]=240;regs[57]=15;}
    if(scenario==22){regs[56]=5;}
    if(scenario==35||scenario==36){
        regs[60]=0x21;regs[61]=0x43;regs[62]=121;regs[63]=42;
        regs[65]=8;regs[69]=0xdc;regs[70]=0xfe;
    }
    SingleArmApp_Init(&host,&servo);feed("INFO\nSNAP\n");assert(writes==0&&strstr(output,"lift-teleop-v4"));
    if(scenario==35){
        assert(strstr(output,"\"load_raw\":17185")&&strstr(output,"\"voltage_raw\":121")&&strstr(output,"\"temperature_c\":42"));
        assert(strstr(output,"\"current_raw\":65244")&&strstr(output,"\"motor_status_raw\":8")&&strstr(output,"\"telemetry_valid\":true"));
        puts(output);return 0;
    }
    output[0]=0;
    if(scenario==36){
        feed("SNAP\n");assert(writes==0&&strstr(output,"\"telemetry_valid\":false"));
        assert(strstr(output,"\"failure_register\":56")&&strstr(output,"\"failure_hal_status\":3"));
        assert(strstr(output,"\"telemetry_age_ms\":4294967295"));puts(output);return 0;
    }
    if(scenario==13){regs[40]=1;packet("PREP 1234ABCD");assert(writes==0&&strstr(output,"PREPARE_ID_MODE_TORQUE_LOCK"));return 0;}
    if(scenario==14){packet("ARM 1234ABCD");assert(writes==0&&strstr(output,"ARM_ID_MODE_LOCK_GOAL"));return 0;}
    if(scenario==26){
        char later[80];snprintf(later,sizeof(later),"ECHO 00000002 %04X\n",crc("ECHO 00000002"));
        delayed_input=later;packet("ECHO 00000001");
        assert(writes==0&&strstr(output,"\"nonce\":1")&&strstr(output,"\"nonce\":2"));return 0;
    }
    if(scenario==29){
        char stale[80];snprintf(stale,sizeof(stale),"PREP 1234ABCD %04X\n",crc("PREP 1234ABCD"));
        queue_bytes(stale);tick+=401;SingleArmApp_Process();
        assert(writes==0&&strstr(output,"STALE_HOST_FRAME"));return 0;
    }
    packet("PREP 1234ABCD");
    if(scenario==6||scenario==24){assert(regs[55]==1&&nonzero==0&&regs[40]==0&&strstr(output,"MODE_CHANGE_FAILED"));return 0;}
    assert(regs[33]==1&&regs[40]==0&&regs[46]==0&&regs[55]==1&&nonzero==0);
    assert(strstr(output,"\"type\":\"prepared\""));output[0]=0;
    packet(scenario==8?"TEST 1234ABCD":"ARM 1234ABCD");assert(strstr(output,"\"type\":\"armed\""));output[0]=0;
    packet("D 1234ABCD 00000001 D");
    if(scenario==4||scenario==5||scenario==7){stopped();assert(strstr(output,"\"fault\":true"));return 0;}
    assert(regs[46]==100&&regs[40]==1);output[0]=0;
    if(scenario==0||scenario==11)feed("STOP\n");
    else if(scenario==1)packet("D 1234ABCD 00000001 D");
    else if(scenario==2)feed("D 1234ABCD 00000002 D 0000\n");
    else if(scenario==3)packet("D FFFFFFFF 00000002 D");
    else if(scenario==8){tick+=151;SingleArmApp_Process();assert(strstr(output,"TEST_PULSE_COMPLETE"));}
    else if(scenario==9){tick+=401;SingleArmApp_Process();assert(strstr(output,"HOST_TIMEOUT"));}
    else if(scenario==10){regs[56]=232;regs[57]=3;packet("D 1234ABCD 00000002 Z");assert(strstr(output,"ENCODER_CONTINUITY_LOST"));}
    else if(scenario==12){packet("D 1234ABCD 00000002 Z");assert(!strstr(output,"\"fault\":true"));feed("STOP\n");}
    else if(scenario==15){packet("D 1234ABCD 00000002 U");}
    else if(scenario==16){packet("ARM 1234ABCD");}
    else if(scenario==17){feed("\001");}
    else if(scenario==18){regs[40]=0;packet("D 1234ABCD 00000002 D");}
    else if(scenario==19){regs[56]=250;packet("D 1234ABCD 00000002 D");assert(strstr(output,"\"relative_counts\":50"));feed("STOP\n");}
    else if(scenario==20){regs[56]=140;packet("D 1234ABCD 00000002 D");feed("STOP\n");}
    else if(scenario==21){regs[56]=12;regs[57]=0;packet("D 1234ABCD 00000002 D");assert(strstr(output,"\"relative_counts\":28"));feed("STOP\n");}
    else if(scenario==22){regs[56]=240;regs[57]=15;packet("D 1234ABCD 00000002 D");assert(strstr(output,"UNEXPECTED_REVERSE_MOTION"));}
    else if(scenario==23){
        for(unsigned i=0;i<200;i++){
            unsigned p=(200+(i+1)*100)%4096;regs[56]=p&255;regs[57]=p>>8;
            char line[50];snprintf(line,sizeof(line),"D 1234ABCD %08X D",i+2);packet(line);
            assert(!strstr(output,"\"fault\":true"));
            if(i==199)assert(strstr(output,"\"relative_counts\":20000"));
            output[0]=0;
        }
        feed("STOP\n");
    }
    else if(scenario==25){regs[33]=0;unsigned before=writes;packet("D 1234ABCD 00000002 D");assert(writes==before&&strstr(output,"MODE_CHANGED"));return 0;}
    else if(scenario==27){HAL_UART_ErrorCallback(&host);SingleArmApp_Process();assert(strstr(output,"HOST_RX_ERROR"));}
    else if(scenario==28){char burst[1300];memset(burst,'X',sizeof(burst)-1);burst[sizeof(burst)-1]=0;feed(burst);assert(strstr(output,"HOST_RX_ERROR"));}
    else if(scenario==30||scenario==31||scenario==34){
        packet("D 1234ABCD 00000002 D");
        const char *why=scenario==31?"TORQUE_READ_FAILED":"MODE_READ_FAILED";
        assert(strstr(output,why)&&strstr(output,scenario==31?"\"failure_register\":40":"\"failure_register\":33"));
        assert(strstr(output,"\"failure_hal_status\":3")&&strstr(output,"\"bus_failure_reason\":2"));
        output[0]=0;feed("STOP\n");
        assert(strstr(output,why)&&strstr(output,"\"failure_hal_status\":3")&&strstr(output,"\"bus_failure_reason\":2"));
        stopped();
        assert(strstr(output,scenario==34?"\"stop_confirmed\":false":"\"stop_confirmed\":true"));
        puts(output);return 0;
    }
    else if(scenario==32||scenario==33){
        regs[scenario==32?33:40]=0;packet("D 1234ABCD 00000002 D");
        assert(strstr(output,scenario==32?"MODE_CHANGED":"TORQUE_CHANGED")&&strstr(output,"\"failure_register\":-1"));
        puts(output);return 0;
    }
    stopped();
    if(scenario==11)assert(strstr(output,"STOP_UNCONFIRMED"));
    else assert(strstr(output,"\"stop_confirmed\":true"));
    if(scenario!=0&&scenario!=8&&scenario!=12&&scenario!=19&&scenario!=21&&scenario!=23){
        assert(strstr(output,"\"fault\":true"));unsigned before=writes;packet("ARM 11111111");assert(writes==before);
    }
    puts(output);return 0;
}
