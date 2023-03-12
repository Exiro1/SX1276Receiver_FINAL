#include "mbed.h"
#include "main.h"
#include "sx1276-hal.h"
#include "modulation.h"
#include "time.h"

/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE   0

/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA

/* Set default receiver parameters          */

#define RF_FREQUENCY                                    434400000 // Hz
#define TX_OUTPUT_POWER                                 7        // 14 dBm


#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR                       9         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_FHSS_ENABLED                           false
#define LORA_NB_SYMB_HOP                            4
#define LORA_IQ_INVERSION_ON                        false
#define LORA_CRC_ENABLED                            true


#define FSK_FDEV                                    25000     // Hz
#define FSK_DATARATE                                19200     // bps
#define FSK_BANDWIDTH                               50000     // Hz
#define FSK_AFC_BANDWIDTH                           83333     // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false
#define FSK_CRC_ENABLED                             true

#define RX_TIMEOUT_VALUE                                3500      // in ms
#define BUFFER_SIZE                                     40        // Define the payload size here


#define ERROR_MAX                           1

#define CONFIGNBR 4



#if( defined ( TARGET_KL25Z ) || defined ( TARGET_LPC11U6X ) )
DigitalOut led( LED2 );
#else
DigitalOut led( LED1 );
#endif

/*
 *  Global variables declarations
 */
typedef enum {
    LOWPOWER = 0,
    IDLE,

    RX,
    RX_TIMEOUT,
    RX_ERROR,

    TX,
    TX_TIMEOUT,

    CAD,
    CAD_DONE
} AppStates_t;

typedef enum {
    HS,
    TIME,
    CONFIGTYPE,
    CONFIGBAND,
    CONFIGFDEV,
    CONFIGDR,
    PATTERN,
    END
} UpdateState_t;


volatile AppStates_t State = LOWPOWER;
UpdateState_t updState = HS;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*
 *  Global variables declarations
 */
SX1276MB1xAS Radio( NULL );


uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

int16_t RssiValue = 0.0;
int8_t SnrValue = 0.0;

uint8_t SFvalue[6] = {7,8,9,10,11,12};

const uint8_t handshake[7] = "HSINIT";

bool configurated = false;

uint8_t sfind = 1;
RadioConfig conf;


Serial pc(USBTX,USBRX);



//liste des config a tester
/*
uint32_t freqTL[CONFIGNBR] = {434400000,434400000,434400000,434400000,434400000,434400000};
uint32_t fDevTL[CONFIGNBR] = {0,0,0,0,0,0};
uint32_t bandTL[CONFIGNBR] = {0,0,0,0,0,0};
uint32_t drTL[CONFIGNBR] = {7,8,9,10,11,12};
uint32_t afcTL[CONFIGNBR] = {0,0,0,0,0,0};
ModemType typeTL[CONFIGNBR] = {MODEM_LORA,MODEM_LORA,MODEM_LORA,MODEM_LORA,MODEM_LORA,MODEM_LORA};
uint8_t crTL[CONFIGNBR] = {1,1,1,1,1,1};
uint8_t ptxTL[CONFIGNBR] = {7,7,7,7,7,7}; //10
*/
uint32_t freqTL[4] = {434400000,434400000,434400000,434400000};
uint32_t fDevTL[4] = {0,0,0,0,};
uint32_t bandTL[4] = {0,0,0,0};
uint32_t drTL[4] = {12,11,10,7};
ModemType typeTL[4] = {MODEM_LORA,MODEM_LORA,MODEM_LORA,MODEM_LORA};
uint8_t crTL[4] = {1,1,1,1};
uint8_t ptxTL[4] = {7,7,7,7}; //10


ModemType typeT;
uint8_t bandT;
uint8_t sfT;
uint8_t crT;
uint8_t ptxT;

uint16_t cycleCounter = 0;
uint8_t lastMinute = 0;

struct tm * timeInfosP;
time_t timestampP;
uint8_t ID_config=0;


int main( void )
{
    pc.baud (115200);
    debug( "------Reicever Started------\n\r" );
    set_time(1617729516);
    
    
    
    
    
    
    // Initialize Radio driver
    initRadioEvent();

    // verify the connection with the board
    while( Radio.Read( REG_VERSION ) == 0x00  ) {
        debug( "Radio could not be detected!\n\r");
        wait( 1 );
    }

    //debug_if( ( DEBUG_MESSAGE & ( Radio.DetectBoardType( ) == SX1276MB1LAS ) ), "\n\r > Board Type: SX1276MB1LAS < \n\r" );
    //debug_if( ( DEBUG_MESSAGE & ( Radio.DetectBoardType( ) == SX1276MB1MAS ) ), "\n\r > Board Type: SX1276MB1MAS < \n\r" );

    conf = (RadioConfig) {
        MODEM_LORA, RF_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,LORA_CODINGRATE, 0,
                    LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT,LORA_FIX_LENGTH_PAYLOAD_ON,0,
                    LORA_CRC_ENABLED,LORA_FHSS_ENABLED,LORA_NB_SYMB_HOP,LORA_IQ_INVERSION_ON,
                    true,TX_OUTPUT_POWER,FSK_FDEV
    };



    debug( "-----HandShake Process Started-----\n\r" );



    led = 0;

    //updateProcess();

    //on forcela config 0 car pour les test, on utilise pas celle envoyé par l'ordi
    changeConfig(0);
    time_t timestamp = time( NULL );
    struct tm * timeInfos = localtime( & timestamp );
    lastMinute = timeInfos->tm_min;
    startProcessing();



}

uint8_t freqTemp[16];
uint8_t dataRateTemp[16];
uint8_t bandTemp[16];
uint8_t freqDevTemp[16];

uint8_t pattern[16];
char bufferT[32];
uint8_t patternSize = 0;

void getInfo()
{

    pc.printf("set time\n\r");
    pc.gets(bufferT, 12);
    set_time(atoi(bufferT));

    //wait_ms(500);
    //type
    pc.printf("set mode\n\r");
    memset(bufferT, 0, 32);
    pc.gets(bufferT, 16);
    typeT = MODEM_LORA;
    if(atoi(bufferT) == 0)
        typeT = MODEM_FSK;
    wait_ms(500);
    // freq
    pc.printf("set freq\n\r");
    memset(bufferT, 0, 32);
    pc.gets(bufferT, 16);
    strcpy((char*)freqTemp, bufferT);
    wait_ms(500);
    //band
    pc.printf("set band\n\r");
    memset(bufferT, 0, 32);
    pc.gets(bufferT, 16);
    if(typeT == MODEM_LORA) {
        bandT = atoi(bufferT);
    }
    if(typeT == MODEM_FSK) {
        strcpy((char*)bandTemp, bufferT);
    }


    wait_ms(500);
    //SF / datarate
    pc.printf("set rate\n\r");
    memset(bufferT, 0, 32);
    pc.gets(bufferT, 16);

    if(typeT == MODEM_LORA) {
        sfT = atoi(bufferT)+7;
    }
    if(typeT == MODEM_FSK) {
        strcpy((char*)dataRateTemp, bufferT);
    }
    wait_ms(500);
    //CR
    pc.printf("set cr\n\r");
    memset(bufferT, 0, 32);
    pc.gets(bufferT, 16);
    if(typeT == MODEM_LORA) {
        crT = atoi(bufferT)+1;
    }
    if(typeT == MODEM_FSK) {
        strcpy((char*)freqDevTemp, bufferT);
    }
    wait_ms(500);
    //PTX
    pc.printf("set ptx\n\r");
    memset(bufferT, 0, 32);
    pc.gets(bufferT, 10);
    ptxT = atoi(bufferT);

    wait_ms(500);
    //
    pc.printf("set pattern\n\r");
    memset(bufferT, 0, 32);
    pc.gets(bufferT, 16);
    strcpy((char*)pattern, bufferT);


    pc.printf("SETUP COMPLETE\n\r");

}

void changeConfig(uint8_t id)
{
    wait_ms(500);
    conf.freq = freqTL[id];
    conf.power = ptxTL[id];
    pc.printf("INFO : %u %u\n\n",typeTL[id],id);
    conf.type= typeTL[id];
    if(conf.type == MODEM_LORA) {
        conf.datarate = drTL[id];
        conf.coderate = crTL[id];
        conf.bandwidth = bandTL[id];
        pc.printf("CONF;1;%u;%u;%u;%u;%u;%u\n\r",conf.freq,conf.datarate,conf.coderate,conf.power,conf.bandwidth, conf.dev);
    } else {
        conf.datarate = drTL[id];
        conf.dev = fDevTL[id];
        conf.bandwidth = bandTL[id];
        pc.printf("CONF;0;%u;%u;%u;%u;%u;%u\n\r",conf.freq,conf.datarate,conf.coderate,conf.power,conf.bandwidth, conf.dev);
    }
    setRF(conf,&Radio, true);
    wait_ms(500);
    
    
}


//peut etre reduire la puissance pour pas dépasser la limite autorisé (a cause du gain de l'antenne yagi)
void updateProcess()
{

    getInfo();
    wait_ms(500);
    pc.printf("CONF;%u;%u;%u;%u;%u;%u;%u\n\r",conf.type,conf.freq,conf.datarate,conf.coderate,conf.power,conf.bandwidth, conf.dev);
    setRF(conf,&Radio, false);
    //get info from computer

    wait_ms(100);


    //uint8_t* p_time2 = getTime();
    //pc.printf("%u %u %u %u %u %u\n",p_time2[0],p_time2[1],p_time2[2],p_time2[3],p_time2[4],p_time2[5]);
    //delete p_time2;



    sendFrame(handshake,7);
    bool end = false;

    while( !end ) {
        switch( State ) {

            case RX: {
                State = LOWPOWER;
                if( BufferSize > 0 ) {
                    wait_ms(500);
                    if(strcmp((char*)Buffer,"ACK") == 0) {
                        updState = TIME;
                        pc.printf("HandShake accpeted \n\r");
                        uint8_t* p_time = getTime();
                        pc.printf("%u %u %u %u %u %u \n\r",p_time[0],p_time[1],p_time[2],p_time[3],p_time[4],p_time[5]);
                        sendFrame(p_time,6);
                        delete p_time;
                    } else if(strcmp((char*)Buffer,"TIMESET") == 0) { //send type + Power
                        updState = CONFIGTYPE;
                        pc.printf("Time set \n\r");
                        uint8_t configt[2]; //implement config manager
                        configt[0] = 0;
                        if(typeT == MODEM_LORA)
                            configt[0] = 1;
                        configt[1] = ptxT;
                        sendFrame(configt,2);
                    }

                    else if(strcmp((char*)Buffer,"TYPESET") == 0) { //send band
                        updState = CONFIGBAND;
                        pc.printf("Type set \n\r");
                        if(typeT == MODEM_FSK)
                            sendFrame(bandTemp,16);

                        if(typeT == MODEM_LORA) {
                            uint8_t bandc[1];
                            bandc[0] = bandT;
                            sendFrame(bandc,1);
                        }
                    }

                    else if(strcmp((char*)Buffer,"BANDSET") == 0) { //send freqdev
                        updState = CONFIGFDEV;
                        pc.printf("Bandwidth set \n\r");
                        if(typeT == MODEM_FSK)
                            sendFrame(freqDevTemp,16);
                        if(typeT == MODEM_LORA) {
                            uint8_t crc[1];
                            crc[0] = crT;
                            sendFrame(crc,1);
                        }
                    } else if(strcmp((char*)Buffer,"FDEVSET") == 0) { //send datarate
                        updState = CONFIGDR;
                        pc.printf("FreqDev set \n\r");
                        if(typeT == MODEM_FSK)
                            sendFrame(dataRateTemp,16);
                        if(typeT == MODEM_LORA) {
                            uint8_t sfc[1];
                            sfc[0] = sfT;
                            sendFrame(sfc,1);
                        }
                    } else if(strcmp((char*)Buffer,"DATASET") == 0) {
                        updState = PATTERN;
                        pc.printf("Config set \n\r");
                        sendFrame(pattern,16);
                    }

                    else if(strcmp((char*)Buffer,"PATSET") == 0) {
                        updState = END;
                        pc.printf("Pattern set \n\r");
                        sendFrame(freqTemp,16);
                    } else {
                        //error, restarting process
                        updState = HS;
                        sendFrame(handshake,7);
                    }

                }

                break;
            }

            case TX: {
                if(updState == END) {
                    end = true;
                    State = LOWPOWER;
                    break;
                }
                Radio.Rx( 3000 );
                State = LOWPOWER;
                break;
            }

            //si rien reçu, on renvoi le handshake
            case RX_TIMEOUT: {
                State = LOWPOWER;
                pc.printf( "TIMEOUT\n\r" );
                sendFrame(handshake,7);
                updState = HS;
                break;
            }
            case RX_ERROR: {
                //restart process
                pc.printf( "ERROR\n\r" );
                updState = HS;
                State = LOWPOWER;
                wait_ms(500);
                sendFrame(handshake,7);
                break;
            }
            case LOWPOWER:
                break;
            default:
                State = LOWPOWER;
                break;

        }
    }


    conf.freq = atoi((char*)freqTemp);
    if(typeT == MODEM_FSK) {
        conf.datarate = atoi((char*)dataRateTemp);
        conf.dev= atoi((char*)freqDevTemp);
        conf.bandwidth = atoi((char*)bandTemp);
    } else {
        conf.datarate = sfT;
        conf.coderate = crT;
        conf.bandwidth = bandT;
    }
    conf.power = ptxT;
    conf.type = typeT;

    conf.bandwidthAfc = 83333;

    pc.printf("END %u %u %u %u %u %u\n\r",conf.freq,conf.datarate,conf.coderate,conf.dev,conf.power,conf.bandwidth);
    configurated = true;

}



//pas oublier de free !!!
uint8_t* getTime()
{
    uint8_t * timel = new uint8_t[6];
    time_t timestamp = time( NULL );
    struct tm * timeInfos = localtime( & timestamp );
    timel[0] = timeInfos->tm_mday;
    timel[1] = timeInfos->tm_mon;
    timel[2] = timeInfos->tm_year;
    timel[3] = timeInfos->tm_hour;
    timel[4] = timeInfos->tm_min;
    timel[5] = timeInfos->tm_sec;
    //lastMinute = timeInfos->tm_min;
    return timel;
}



void startProcessing()
{


//reset receiver parameters

    pc.printf("CONF;%u;%u;%u;%u;%u;%u;%u\n\r",conf.type,conf.freq,conf.datarate,conf.coderate,conf.power,conf.bandwidth, conf.dev);
    //setRF(conf,&Radio);
    Radio.Rx( RX_TIMEOUT_VALUE );

    debug_if( DEBUG_MESSAGE, "Starting reception loop ...\n\r" );

    while( 1 ) {
                
                timestampP = time( NULL );
                timeInfosP = localtime( & timestampP );
                /*
                if(timeInfosP->tm_min != lastMinute && cycleCounter < 60) {
                    cycleCounter++;
                    pc.printf("TIME : %u %u %u\n",timeInfosP->tm_hour,timeInfosP->tm_min,timeInfosP->tm_sec);
                    lastMinute = timeInfosP->tm_min;
                    ID_config++;
                    if(ID_config >= 4)
                        ID_config = 0;
                    if(cycleCounter>=60)
                        ID_config = 0;    
                        
                    changeConfig(ID_config);
                    Radio.Rx( RX_TIMEOUT_VALUE );
                    State = LOWPOWER;
                    pc.printf("config : %u\n",ID_config);
                }
                */
        switch( State ) {
            case RX:

                if( BufferSize > 0 ) {
                    //pc.printf("recu");

                    pc.printf(" rssi: %d", RssiValue);
                    char buf[16];
                    sprintf(buf, "SUCCESS;%d;", RssiValue);
                    pc.printf(buf);

                    for(uint8_t j=0; j<BufferSize; j++) {
                        pc.printf("%02X:", (Buffer[j]/*-1*/));
                        //pc.putc(Buffer[j]);
                    }
                    //pc.printf("%c",(Buffer[j]-1));

                    //strcpy(text, (char*)Buffer);
                    //pc.printf("%s",text);

                    pc.printf( "\n\r" );


                    led = !led;
                    //wait_ms( 10 );


                    //if(strcmp((char*)Buffer,"next") == 0){
                    // switchRF();
                    //}
                    memset(Buffer, 0, sizeof(Buffer)); //fill Buffer with 0



                }
                
                Radio.Rx( RX_TIMEOUT_VALUE );
                State = LOWPOWER;
                break;

            case RX_TIMEOUT:
                Radio.Rx( RX_TIMEOUT_VALUE );
                State = LOWPOWER;
                pc.printf( "TIMEOUT\n\r" );
                break;
            case RX_ERROR:
                // We have received a Packet with a CRC error, send reply as if packet was correct
                pc.printf( "ERROR\n\r" );
                Radio.Rx(RX_TIMEOUT_VALUE);
                State = LOWPOWER;
                break;
            case LOWPOWER:
                break;
            default:
                State = LOWPOWER;
                break;
        }
    }
}

void convertData(){





}


void sendFrame(uint8_t* bytes,uint8_t len)
{
    char text[64];
    strcpy(text, (char*)bytes);
    debug( "sending %s\n\r",text);
    Radio.Send(bytes, len);
    //wait_ms(10);
}
void sendFrame(const uint8_t* bytes,uint8_t len)
{
    char text[64];
    strcpy(text, (char*)bytes);
    debug( "sending %s\n\r",text);
    Radio.Send((uint8_t*)text, len);
    //wait_ms(10);
}



void initRadioEvent()
{
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxError = OnRxError;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init(&RadioEvents);
}

void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
    debug_if( DEBUG_MESSAGE, "> OnTxDone\n\r" );
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
    debug_if( DEBUG_MESSAGE, "> OnRxDone\n\r" );
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
    debug_if( DEBUG_MESSAGE, "> OnTxTimeout\n\r" );
}
volatile uint8_t errorCounter = 0;

void OnRxTimeout( void )
{
    Radio.Sleep( );
    Buffer[BufferSize] = 0;
    State = RX_TIMEOUT;
    //pc.printf( "TIMEOUT\n\r" );
    debug_if( DEBUG_MESSAGE, "> OnRxTimeout\n\r" );
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
    //pc.printf( "ERROR\n\r" );
    debug_if( DEBUG_MESSAGE, "> OnRxError\n\r" );
}

