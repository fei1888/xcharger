#include <iostream>
#include <time.h>
#include <unistd.h>

#define DEBUG_LOG printf
#define MSEC 1000

//CAN struct example
typedef struct {
    uint8_t Data[8];
    uint16_t Length;
    uint32_t ID;
} CAN_msg_typedef;



enum XNERGY_CAN_ID_e {
    XNERGY_CAN_HB ,       // hearbeat msg
    XNERGY_CAN_CTRL,      // charge control
    XNERGY_CAN_STAT       // charge status
};

enum XNERGY_CHARGER_STAT_e {
    XNERGY_CHARGER_IDLE_STAT,       // state changed when enable_command is true
    XNERGY_CHARGER_CONST_CURR_STAT, // state changed when vol_fb >= vref
    XNERGY_CHARGER_CONST_VOLT_STAT  // state changed when vol_fb >= verf and curr_fb < minimum_current 
};

typedef char state_name[30];

const state_name charging_state_str[] {
    "IDLE state",
    "Constant current state",
    "Constant voltage state",
    "unknown state"
};

enum XNERGY_NETWORK_STAT_e {
    XNERGY_NETWORK_INIT,
    XNERGY_NETWORK_PRE_OPERATE,
    XNERGY_NETWORK_OPERATE
};

enum XNERGY_ACTION_e {
    XNERGY_STOP_VOLTAGE_AND_CURRENT,
    XNERGY_SET_VOLTAGE,
    XNERGY_SET_CURRENT
};

typedef struct {
    XNERGY_ACTION_e action;
    uint16_t Iref;      // resolution 1 = 0.1V
    uint16_t Vref;      // resolution 1 = 0.1V
    uint16_t Imin;      // resolution 1 = 0.1A
    bool enable_command;
    uint16_t current_feedback;
    uint16_t voltage_feedback;
} XNERGY_PI_CONTROL_t;

/**** function prototype and global var */
CAN_msg_typedef Can_tx;
CAN_msg_typedef Can_rx;

void CAN_write(CAN_msg_typedef *msg);
bool CAN_read(CAN_msg_typedef *msg);

XNERGY_CHARGER_STAT_e charger_operate_state;
XNERGY_NETWORK_STAT_e charger_network_state;
XNERGY_PI_CONTROL_t   charger_control;

const char* state_translate_str(XNERGY_CHARGER_STAT_e state) {
    switch (state) {
        case XNERGY_CHARGER_IDLE_STAT:
            return charging_state_str[0];
        case XNERGY_CHARGER_CONST_CURR_STAT:
            return charging_state_str[1];
        case XNERGY_CHARGER_CONST_VOLT_STAT:
            return charging_state_str[2];
        default:
            return charging_state_str[3];
    }
}

int regulate_current(uint16_t val) {
    DEBUG_LOG("XNERGY_INFO: Set current to %d.%d amp\n", val/10, val%10);
    return true;
}

int regulate_voltage(uint16_t val) {
    DEBUG_LOG("XNERGY_INFO: Set voltage to %d.%d volts\n", val/10, val%10);
    return true;
}

void control_current(void) {
    
    switch(charger_operate_state) {
        case XNERGY_CHARGER_IDLE_STAT:
            /* Stop charging */
            regulate_current(0);
            break;
        case XNERGY_CHARGER_CONST_CURR_STAT:
            if (charger_control.current_feedback != charger_control.Iref) {
                /* Regulate current = Iref */
                charger_control.action = XNERGY_SET_CURRENT;
                regulate_current(charger_control.Iref);
            }
            break;
        case XNERGY_CHARGER_CONST_VOLT_STAT:
            /* no regulate for voltage at this state */
            break;
        default:
            DEBUG_LOG("XNERGY_ERR: fault state machine at control_current()\n");
    }
}

void control_voltage(void) {
    
    switch(charger_operate_state) {
        case XNERGY_CHARGER_IDLE_STAT:
            /* Stop charging */
            regulate_voltage(0);
            break;
        case XNERGY_CHARGER_CONST_CURR_STAT:
            /* no regulate for voltage at this state */
            break;
        case XNERGY_CHARGER_CONST_VOLT_STAT:
            if (charger_control.voltage_feedback != charger_control.Vref) {
                // V=IR , decrease charging current to increase V
                charger_control.action = XNERGY_SET_VOLTAGE;
                regulate_voltage(charger_control.Vref);
            } 
            break;
        default:
            DEBUG_LOG("XNERGY_ERR: fault state machine at control_voltage()\n");
    }
}

void control_routine(void) {
    control_voltage();
    control_current();
}

void main_state_machine(void){
    XNERGY_CHARGER_STAT_e previous_state;
    previous_state = charger_operate_state;
    
    switch(charger_operate_state) {
        case XNERGY_CHARGER_IDLE_STAT:
            charger_network_state = XNERGY_NETWORK_PRE_OPERATE;
            if (charger_control.enable_command == true) {
                charger_operate_state = XNERGY_CHARGER_CONST_CURR_STAT;
            }
            break;
        case XNERGY_CHARGER_CONST_CURR_STAT:
            charger_network_state = XNERGY_NETWORK_OPERATE;
            if (charger_control.voltage_feedback >= charger_control.Vref) {
                charger_operate_state = XNERGY_CHARGER_CONST_VOLT_STAT;
            }
            break;
        case XNERGY_CHARGER_CONST_VOLT_STAT:
            charger_network_state = XNERGY_NETWORK_OPERATE;
            if (charger_control.current_feedback <= charger_control.Imin) {
                charger_control.enable_command = false;
                charger_operate_state = XNERGY_CHARGER_IDLE_STAT;
            }
            break;
        default:
            DEBUG_LOG("XNERGY_ERR: should not fall into this state\n");
    }

    DEBUG_LOG("XNERGY_INFO: Previous state %s (%d), Current state %s (%d)\n",
              state_translate_str(previous_state), previous_state, 
              state_translate_str(charger_operate_state), charger_operate_state);
    DEBUG_LOG("XNERGY_INFO: Charging enable = %s\n", charger_control.enable_command? "true":"false");
    
    /* control voltage and current */
    control_routine();

}

void initialization(void){
    DEBUG_LOG("XNERGY_INFO: initialization() start\n");
    charger_operate_state = XNERGY_CHARGER_IDLE_STAT;
    charger_control.action = XNERGY_STOP_VOLTAGE_AND_CURRENT;
    charger_network_state = XNERGY_NETWORK_INIT;
    DEBUG_LOG("XNERGY_INFO: initialization() completed\n");
    
#ifdef TESTING
    charger_control.enable_command = true;
    charger_control.Vref = 120;   //12 Volts
    charger_control.Iref = 50;    // 5 A
    charger_control.Imin = 10;    // 1 A
    charger_control.current_feedback = 0;
    charger_control.current_feedback = 0;
#else
    charger_control.enable_command = false;
    charger_control.Vref = 0;
    charger_control.Iref = 0;
    charger_control.Imin = 0;
    charger_control.current_feedback = 0;
    charger_control.current_feedback = 0;
#endif

}


void CAN_write(CAN_msg_typedef *msg) {
    return;
}
bool CAN_read(CAN_msg_typedef *msg) {
    return true;
}

void can_write_handler() {
    
    /* send HB out */
    Can_tx.ID = 0x701;
    Can_tx.Data[0] = (uint8_t)charger_network_state;
    CAN_write(&Can_tx);
    
    switch (charger_network_state) {
        case XNERGY_NETWORK_INIT:
            break;
        case XNERGY_NETWORK_PRE_OPERATE: 
            break;
        case XNERGY_NETWORK_OPERATE:
            Can_tx.ID = 0x181;
            Can_tx.Data[0] = (charger_control.voltage_feedback >>8) & 0xFF;
            Can_tx.Data[1] = (charger_control.voltage_feedback & 0xFF);
            Can_tx.Data[2] = (charger_control.current_feedback >>8) & 0xFF;
            Can_tx.Data[3] = (charger_control.current_feedback & 0xFF);
            CAN_write(&Can_tx);
            break;
        default:
            break;
    }
}

void can_read_handler(void) {

    if (CAN_read(&Can_rx)==true) {
        switch (Can_rx.ID) {
            case XNERGY_CAN_CTRL:
                // vref update
                charger_control.Vref = ((Can_rx.Data[0] & 0xFF)<<8) | Can_rx.Data[1];
                charger_control.Iref = ((Can_rx.Data[2] & 0xFF)<<8) | Can_rx.Data[3];
                // enable cmd update
                if (Can_rx.Data[4] == 0) {
                    charger_control.enable_command = false;
                } else if (Can_rx.Data[4] == 1) {
                    charger_control.enable_command = true;
                } else {
                    DEBUG_LOG("XNERGY_ERR: can_read_handler() unexpected enable_command value %d\n", Can_rx.Data[4]);
                }
                break;
            case XNERGY_CAN_HB: 
            case XNERGY_CAN_STAT:
                // not suppose receive by the charger`
            default:
                break;
        }
    }
}

void network_management(void)
{
    can_write_handler();
    can_read_handler();
}

int main(void)
{
    initialization();

    printf("==== Xnergy charger start operating ====\n");
    
    while (true) {
        main_state_machine();
        network_management();
        usleep(MSEC);
    }
    
	return 0;
}
