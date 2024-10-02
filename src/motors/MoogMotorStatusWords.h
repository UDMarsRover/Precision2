

#ifndef MoogMotorStatusWords_h
#define MoogMotorStatusWords_h


#define okSTAT    0b0 // Need To Set
#define warnSTAT  0b0 // Need To Set
#define errorSTAT 0b0 // Need To Set

struct errorCode {
  String name;
  int code;
};
/*
//=================================================//
//          Define Status Word 0 Values            //
//=================================================//
errorCode leftPosLimit  = {"Left Position Limit",     0b1000000000000000};  //0x8000
errorCode rightPosLimit = {"Right Position Limit",    0b0100000000000000};  //0x4000
errorCode histLeftLimit = {"Histogram Left Limit",    0b0010000000000000};  //0x2000
errorCode histRightLimit= {"Histogram Right Limit",   0b0001000000000000};  //0x1000
errorCode negHWLimit    = {"Negative Hardware Limit", 0b0000100000000000};  //0x0800
errorCode posHWLimit    = {"Positive Hardware Limit", 0b0000010000000000};  //0x0400
errorCode dedtError     = {"Derivitive Error",        0b0000001000000000};  //0x0200
errorCode tempWarning   = {"Over Temperature Warning",0b0000000100000000};  //0x0100
errorCode velocityLimit = {"Velocity Limit",          0b0000000010000000};  //0x0080
errorCode positionLimit = {"Position Limit Error",    0b0000000001000000};  //0x0040
errorCode tempError     = {"Over Temperature Error",  0b0000000000100000};  //0x0020
errorCode peakOverCur   = {"Peak Over Current Error", 0b0000000000010000};  //0x0010
errorCode servoBusVolt  = {"Servo Bus Voltage Error", 0b0000000000001000};  //0x0008
errorCode motorInMotion = {"Rotor In Motion Status",  0b0000000000000100};  //0x0004
errorCode motorOff      = {"Motor Coils Turned Off",  0b0000000000000010};  //0x0002
errorCode driveReady    = {"Motor Is Ready To Use",   0b0000000000000001};  //0x0001



//=================================================//
//          Define Status Word 1 Values            //
// Index Registration And Software Travel Limits   //
//=================================================//
errorCode rtNegSoftLim        = {"Real Time Negative Soft Limit",                                           0b1000000000000000}; //0x8000
errorCode rtPosSoftLim        = {"Real Time Positive Soft Limit",                                           0b0100000000000000}; //0x4000
errorCode hisNegSoftOvTravLim = {"Historical Negative Software Over Travel Limit",                          0b0010000000000000}; //0x2000
errorCode hisPosSoftOvTravLim = {"Historical Positive Software Over Travel Limit",                          0b0001000000000000}; //0x1000
errorCode softLimMode         = {"Soft Limit Mode (Indicator): 0-Don't Stop. 1-Cause Fault. Default Is 1",  0b0000100000000000}; //0x0800
errorCode softTravLimEn       = {"Software Travel Limits Enabled",                                          0b0000010000000000}; //0x0400
errorCode capInState1         = {"Capture Input State 1 (Indicator)",                                       0b0000001000000000}; //0x0200
errorCode capInState0         = {"Capture Input State 0 (Indicator)",                                       0b0000000100000000}; //0x0100
errorCode falEdgCapENC1       = {"Falling Edge Captured ENC(1) (Historical Bit)",                           0b0000000010000000}; //0x0080
errorCode risEdgCapENC1       = {"Rising Edge Captured ENC(1) (Historical Bit)",                            0b0000000001000000}; //0x0040
errorCode falCapEnc1          = {"Fall Capture Encoder(1) Armed",                                           0b0000000000100000}; //0x0020
errorCode risCapEnc1          = {"Rise Capture Encoder(1) Armed",                                           0b0000000000010000}; //0x0010
errorCode falEdgCapENC0       = {"Falling Edge Captured ENC(0) (Historical Bit)",                           0b0000000000001000}; //0x0008
errorCode risEdgCapENC0       = {"Rising Edge Captured ENC(0) (Historical Bit)",                            0b0000000000000100}; //0x0004
errorCode falCapEnc0          = {"Fall Capture Encoder(0) Armed",                                           0b0000000000000010}; //0x0002
errorCode risCapEnc0          = {"Rise Capture Encoder(0) Armed",                                           0b0000000000000001}; //0x0001

//=================================================//
//          Define Status Word 2 Values            //
//        Communications, Program, and Memory      //
//=================================================//
errorCode prgmCheckSum  = {"Program Checksum Error",           0b1000000000000000};  //0x8000
errorCode commandError  = {"Command Syntax Error",             0b0100000000000000};  //0x4000
errorCode eeBusy        = {"EE Busy",                          0b0010000000000000};  //0x2000
errorCode eeWriteBuff   = {"EE Write Buffer Overflow",         0b0001000000000000};  //0x1000
errorCode traceInprog   = {"Trace In Progress",                0b0000100000000000};  //0x0800
errorCode progRunning   = {"Program Running",                  0b0000010000000000};  //0x0400
errorCode adbBad        = {"ADB Bad Checksum",                 0b0000001000000000};  //0x0200
errorCode watchdogEvent = {"Watchdog Event",                   0b0000000100000000};  //0x0100
errorCode i2cRunning    = {"I2C Running Error",                0b0000000010000000};  //0x0080
errorCode ethernetError = {"Ethernet Error",                   0b0000000001000000};  //0x0040
errorCode reserved5     = {"Reserved 5",                       0b0000000000100000};  //0x0020
errorCode canPortErr    = {"CAN Port Error",                   0b0000000000010000};  //0x0010
errorCode reserved3     = {"Reserved 3",                       0b0000000000001000};  //0x0008
errorCode USBerr        = {"USB Error ",                       0b0000000000000100};  //0x0004
errorCode ComChnl1Err   = {"Error on Communications Channel 1",0b0000000000000010};  //0x0002
errorCode ComChnl0Err   = {"Error on Communicatoins Channel 0",0b0000000000000001};  //0x0001


//=================================================//
//          Define Status Word 3 Values            //
// PID State, Brake, Move Generation Indicators    //
//=================================================//
errorCode velTarRea   = {"Velocity Target Reached", 0b1000000000000000}; //0x8000
errorCode exGoEn      = {"External Go Enabled",     0b0100000000000000}; //0x4000
errorCode braOK       = {"Brake OK",                0b0010000000000000}; //0x2000
errorCode braAss      = {"Brake Asserted",          0b0001000000000000}; //0x1000
errorCode modRoll     = {"Modulo Rollover",         0b0000100000000000}; //0x0800
errorCode peaCurSat   = {"Peak Current Saturation", 0b0000010000000000}; //0x0400
errorCode res9        = {"Reserved 9",              0b0000001000000000}; //0x0200
errorCode relPosMode  = {"Relative Position Mode",  0b0000000100000000}; //0x0100
errorCode ioFauLat    = {"I/O Fault Latch",         0b0000000010000000}; //0x0080
errorCode torDir      = {"Torque Direction",        0b0000000001000000}; //0x0040
errorCode velDir      = {"Velocity Direction",      0b0000000000100000}; //0x0020
errorCode kgEn        = {"KG Enabled",              0b0000000000010000}; //0x0010
errorCode wrapOcc     = {"Wraparound Occured",      0b0000000000001000}; //0x0008
errorCode volSat      = {"Voltage Saturation",      0b0000000000000100}; //0x0004
errorCode torSat      = {"Torque Saturation",       0b0000000000000010}; //0x0002
errorCode res0        = {"Reserved 0",              0b0000000000000001}; //0x0001



//=================================================//
//          Define Status Word 4 Values            //
//               Interrupt Timers                  //
//=================================================//
errorCode timer3   = {"Timer 3",   0b0000000000001000};//0x0008
errorCode timer2   = {"Timer 2",   0b0000000000000100}; //0x0004
errorCode timer1   = {"Timer 1",   0b0000000000000010}; //0x0002
errorCode timer0   = {"Timer 0",   0b0000000000000001}; //0x0001



//=================================================//
//          Define Status Word 5 Values            //
//          Interrupt Status Indicators            //
//=================================================//
errorCode enIntrpts   = {"Enable Interrupts",       0b1000000000000000}; //0x8000
errorCode enIntrpt7   = {"Interrupt 7 Enabled",     0b0000000010000000}; //0x0080
errorCode enIntrpt6   = {"Interrupt 6 Enabled",     0b0000000001000000}; //0x0040
errorCode enIntrpt5   = {"Interrupt 5 Enabled",     0b0000000000100000}; //0x0020
errorCode enIntrpt4   = {"Interrupt 4 Enabled",     0b0000000000010000}; //0x0010
errorCode enIntrpt3   = {"Interrupt 3 Enabled",     0b0000000000001000}; //0x0008
errorCode enIntrpt2   = {"Interrupt 2 Enabled",     0b0000000000000100}; //0x0004
errorCode enIntrpt1   = {"Interrupt 1 Enabled",     0b0000000000000010}; //0x0002
errorCode enIntrpt0   = {"Interrupt 0 Enabled",     0b0000000000000001}; //0x0001



//=================================================//
//          Define Status Word 6 Values            //
//                  Drive Modes                    //
//=================================================//
errorCode rgnAct      = {"Regen Active",            0b1000000000000000}; //0x8000
errorCode hghBusVolt  = {"High Bus Voltage",        0b0100000000000000}; //0x4000
errorCode lowBusVolt  = {"Low Bus Voltage",         0b0010000000000000}; //0x2000
errorCode ABSbatFlt   = {"ABS Battery Fault",       0b0001000000000000}; //0x1000
errorCode MTBact      = {"MTB Active",              0b0000100000000000}; //0x0800
errorCode invd        = {"Inverted",                0b0000010000000000}; //0x0400
errorCode enTOB       = {"TOB enabled",             0b0000001000000000}; //0x0200
errorCode angleMatch  = {"Angle Match",             0b0000000100000000}; //0x0100
errorCode driveEnFault= {"Drive Enable Fault",      0b0000000010000000}; //0x0080
errorCode MDHmodeAct  = {"MDH Mode Active",         0b0000000001000000}; //0x0040
errorCode fdbckFault  = {"Velocity Direction",      0b0000000000100000}; //0x0020
errorCode reserved4   = {"Reserved 4",              0b0000000000010000}; //0x0010
errorCode vectorCntrl = {"Vector Control",          0b0000000000001000}; //0x0008
errorCode sineMode    = {"sineMode",                0b0000000000000100}; //0x0004
errorCode enhcdTrapMd = {"Enhanced Trap Mode",      0b0000000000000010}; //0x0002
errorCode trapMd      = {"Trap Mode",               0b0000000000000001}; //0x0001


//=================================================//
//          Define Status Word 7 Values            //
//                  Drive Modes                    //
//=================================================//
errorCode TSwait            = {"TS WAIT",                                0b1000000000000000}; //0x8000
errorCode tvrsLwrDwellInPrgs= {"Traverse Lower Dwell in Progress",       0b0100000000000000}; //0x4000
errorCode tvrsDirdction     = {"Traverse Direction, 0:FWD, 1:Rev",       0b0010000000000000}; //0x2000
errorCode TG2dwell          = {"TG2 Dwell (or higher Dwell in progress)",0b0001000000000000}; //0x1000
errorCode TG2Decel          = {"TG2 Decel/Decend",                       0b0000100000000000}; //0x0800
errorCode TG2slewing        = {"TG2 Slewing",                            0b0000010000000000}; //0x0400
errorCode TG2Accel          = {"TG2 Accel/Ascend",                       0b0000001000000000}; //0x0200
errorCode TG2inProg         = {"TG2 in Progress",                        0b0000000100000000}; //0x0100
errorCode TG1dwell          = {"TG1 Reserved or Dwell",                  0b0000000000010000}; //0x0010
errorCode TG1decel          = {"TG1 Decel/Descend",                      0b0000000000001000}; //0x0008
errorCode TG1slewing        = {"TG1 Slewing",                            0b0000000000000100}; //0x0004
errorCode TG1accel          = {"TG1 Accel",                              0b0000000000000010}; //0x0002
errorCode TG1inPrgs         = {"Tg1 in Progress",                        0b0000000000000001}; //0x0001



//=================================================//
//          Define Status Word 8 Values            //
//                 Cam Support                     //
//=================================================//
errorCode ipMode1     = {"IP Mode 1",               0b1000000000000000}; //0x8000
errorCode ipMode0     = {"IP Mode 0",               0b0100000000000000}; //0x4000
errorCode ipUserBit5  = {"IP User Bit 5",           0b0010000000000000}; //0x2000
errorCode ipUserBit4  = {"IP User Bit 4",           0b0001000000000000}; //0x1000
errorCode ipUserBit3  = {"IP User Bit 3",           0b0000100000000000}; //0x0800
errorCode ipUserBit2  = {"IP User Bit 2",           0b0000010000000000}; //0x0400
errorCode ipUserBit1  = {"IP User Bit 1",           0b0000001000000000}; //0x0200
errorCode ipUserBit0  = {"IP User Bit 0",           0b0000000100000000}; //0x0100
errorCode camMode1    = {"Cam Mode 1",              0b0000000010000000}; //0x0080
errorCode camMode0    = {"Cam Mode 0",              0b0000000001000000}; //0x0040
errorCode camUserBit5 = {"Cam User Bit 5",          0b0000000000100000}; //0x0020
errorCode camUserBit4 = {"Cam User Bit 4",          0b0000000000010000}; //0x0010
errorCode camUserBit3 = {"Cam User Bit 3",          0b0000000000001000}; //0x0008
errorCode camUserBit2 = {"Cam User Bit 2",          0b0000000000000100}; //0x0004
errorCode camUserBit1 = {"Cam User Bit 1",          0b0000000000000010}; //0x0002
errorCode camUserBit0 = {"Cam User Bit 0",          0b0000000000000001}; //0x0001



//=================================================//
//            Define Status Word 10 Values         //
//             RxPDO Arrival Notification          //
//=================================================//
errorCode RxPDO5ari   = {"RxPDO 5 arrived",          0b0000000000100000}; //0x0020
errorCode RxPDO4ari   = {"RxPDO 4 arrived",          0b0000000000010000}; //0x0010
errorCode RxPDO3ari   = {"RxPDO 3 arrived",          0b0000000000001000}; //0x0008
errorCode RxPDO2ari   = {"RxPDO 2 arrived",          0b0000000000000100}; //0x0004
errorCode RxPDO1ari   = {"RxPDO 1 arrived",          0b0000000000000010}; //0x0002
errorCode MasterEnb   = {"Master Enabled",           0b0000000000000001}; //0x0001
*/
#endif