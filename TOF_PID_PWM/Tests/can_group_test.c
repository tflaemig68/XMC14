/** @file can_group_test.c
 * @brief Testet echte main-Helfer mit einem CAN-Sendemock ohne Hardware.
 */
#define main firmware_main
#include "../main.c"
#undef main
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
static unsigned sends;
static uint32_t sent_id;
static uint8_t sent_data[8];
static CANLIB_TxResult_t send_result=CANLIB_TX_ACCEPTED;
CANLIB_TxResult_t CANLIB_SendWithId(uint32_t id, const uint8_t *data, uint8_t length)
{
    ++sends; sent_id=id;
    if (length!=8) { return CANLIB_TX_ERROR; }
    for (unsigned i=0;i<8;++i) { sent_data[i]=data[i]; }
    return send_result;
}
int can_group_test(void)
{
    static CANLIB_Message_t m;
    static UARTPROTO_Command_t request={1,0,33,10,2};
    PIDinit(&height_pid,0.33f,0.1f,0.02f,0.05f);
    CHECK(UART_Group==0);
    CHECK(TOF_CAN_SendDistance(50,150)==CANLIB_TX_ERROR && sends==0);
    TOF_UARTApply(&request);
    CHECK(UART_Group==1 && UART_Setpoint_mm==0 && TOF_Setpoint_mm==0 && !can_setpoint_valid);
    m.id=CANLIB_RX_ID; m.id_bits=CANLIB_ID_BITS; m.dlc=3; m.length=3;
    m.data[0]=1; m.data[1]=150; m.data[2]=0;
    CHECK(TOF_CAN_ApplySetpoint(&m) && TOF_Setpoint_mm==150 && can_setpoint_valid);
    CHECK(can_setpoint_count==1);
    CHECK(TOF_CAN_SendDistance(50,150)==CANLIB_TX_ACCEPTED && sends==1 && sent_id==0x101);
    CHECK(sent_data[0]==0 && sent_data[1]==50 && sent_data[2]==150 && sent_data[3]==0);
    CHECK(sent_data[4]==150 && sent_data[5]==0 && sent_data[6]==2 && sent_data[7]==1);
    send_result=CANLIB_TX_BUSY;
    CHECK(TOF_CAN_SendDistance(50,150)==CANLIB_TX_BUSY);
    send_result=CANLIB_TX_ACCEPTED;
    CHECK(TOF_CAN_SendDistance(50,150)==CANLIB_TX_ACCEPTED && sent_data[0]==1);
    m.data[0]=2; CHECK(!TOF_CAN_ApplySetpoint(&m) && TOF_Setpoint_mm==150);
    m.data[0]=1; m.data[1]=0x2D; m.data[2]=1;
    CHECK(!TOF_CAN_ApplySetpoint(&m) && TOF_Setpoint_mm==150); /* 301 */
    m.data[1]=255; m.data[2]=255;
    CHECK(!TOF_CAN_ApplySetpoint(&m)); /* -1 */
    m.data[1]=0; m.data[2]=0;
    CHECK(TOF_CAN_ApplySetpoint(&m) && TOF_Setpoint_mm==0 && can_setpoint_valid);
    m.id=0x101; CHECK(!TOF_CAN_ApplySetpoint(&m)); m.id=CANLIB_RX_ID;
    m.id_bits=29; CHECK(!TOF_CAN_ApplySetpoint(&m)); m.id_bits=CANLIB_ID_BITS;
    for (unsigned n=0;n<=8;++n) {
        if (n==3) continue;
        m.length=n; m.dlc=n; CHECK(!TOF_CAN_ApplySetpoint(&m));
    }
    m.length=3; m.dlc=3;
    CHECK(!TOF_CAN_ApplySetpoint(NULL));
    request.group=32; TOF_UARTApply(&request);
    CHECK(!can_setpoint_valid && TOF_Setpoint_mm==0);
    CHECK(!TOF_CAN_ApplySetpoint(&m));
    m.data[0]=32; m.data[1]=0x2C; m.data[2]=1;
    CHECK(TOF_CAN_ApplySetpoint(&m) && TOF_Setpoint_mm==300);
    CHECK(TOF_CAN_SendDistance(0,300)==CANLIB_TX_ACCEPTED && sent_id==0x120);
    TOF_UARTApply(&request); CHECK(can_setpoint_valid && TOF_Setpoint_mm==300);
    request.setpoint=200; TOF_UARTApply(&request);
    CHECK(TOF_Setpoint_mm==200 && !TOF_CAN_ApplySetpoint(&m));
    request.setpoint=0; TOF_UARTApply(&request);
    CHECK(!can_setpoint_valid && TOF_Setpoint_mm==0);
    CHECK(TOF_CAN_ApplySetpoint(&m) && TOF_Setpoint_mm==300);
    /* Reproduce repeated UART profile updates: constant parameters, changing target. */
    request.group=31; request.setpoint=210; request.kp=10; request.ki=10; request.kd=0;
    TOF_UARTApply(&request);
    height_pid.TA=0.05f;
    float output=0.0f;
    for (unsigned i=0;i<100;++i) {
        request.setpoint=(int16_t)(210+i%2);
        TOF_UARTApply(&request);
        output=PIDrun(&height_pid,(float)TOF_Setpoint_mm-80.0f);
    }
    CHECK(output>78.0f && output<79.0f);
    CHECK(height_pid.ISUM>65.0f && height_pid.ISUM<66.0f);
    /* CAN target updates also retain I, while group changes still clear it. */
    request.setpoint=0; TOF_UARTApply(&request);
    height_pid.ISUM=123.0f;
    m.data[0]=31; m.data[1]=150; m.data[2]=0;
    CHECK(TOF_CAN_ApplySetpoint(&m) && height_pid.ISUM==123.0f);
    m.data[1]=160;
    CHECK(TOF_CAN_ApplySetpoint(&m) && height_pid.ISUM==123.0f);
    request.group=30; TOF_UARTApply(&request);
    CHECK(height_pid.ISUM==0.0f);
    return 0;
}
