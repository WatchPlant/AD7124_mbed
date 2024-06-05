
#include "kvstore_global_api.h"
#include <AD7124.h>
#include <cstdint>
#include <cstdio>
#include <queue>
#include <iostream>
using namespace std;


static BufferedSerial FT232(PA_2,PA_3);     // tx, rx of UART
static BufferedSerial pc(USBTX,USBRX);
uint8_t bufbuf[9];
bool flag0, flag1;

void AD7124::init(bool f0, bool f1, uint8_t channel){
    flag0 = f0;
    flag1 = f1;
    FT232.set_baud(9600);
    
    pc.set_baud(19200);
    pc.set_format(
            /* bits */ 8,
            /* parity */ BufferedSerial::Odd,
            /* stop bit */ 1
    );

    AD7124::read_ID();
    AD7124::status();
    AD7124::reset();
    AD7124::status();
    //AD7124::set_data_status_bit();
    AD7124::channel_reg(channel, write); //activate 2 channels
    AD7124::ctrl_reg(read);     // same with control register
    AD7124::ctrl_reg(write);
    AD7124::ctrl_reg(read);

//flags for if you want to have channel 0, or 1, or both active
    if(flag0 == true){
        //config reg 0
        AD7124::config_reg(AD7124_CFG0_REG, read);   // read  configuration register
        AD7124::config_reg(AD7124_CFG0_REG, write);  // write configuration register
        AD7124::config_reg(AD7124_CFG0_REG, read);   // proof writing by reading again
        //filter reg 0
        AD7124::filter_reg(AD7124_FILT0_REG, read);  // same with filter register
        AD7124::filter_reg(AD7124_FILT0_REG, write);
        AD7124::filter_reg(AD7124_FILT0_REG, read);
    }

    if(flag1 == true){
        //config reg 1
        AD7124::config_reg(AD7124_CFG1_REG, read);
        AD7124::config_reg(AD7124_CFG1_REG, write);
        AD7124::config_reg(AD7124_CFG1_REG, read);
        //filter reg 1
        AD7124::filter_reg(AD7124_FILT1_REG, read);
        AD7124::filter_reg(AD7124_FILT1_REG, write);
        AD7124::filter_reg(AD7124_FILT1_REG, read);
    }
    
    //AD7124::calibrate(1,0,0,0);
}

double AD7124::get_analog_value(long measurement){
    /* calculate the analog value from the measurement */
    double voltage = (double)measurement/databits -1;
    voltage = voltage * Vref/ Gain;
    voltage *= 1000;
    //INFO("analog %.3F\n", voltage);
    return voltage;
}

void AD7124::reset(){
    /* reset the ADC */
    cs = 0;
    INFO("Reset ADC\n");
    for (int i = 0; i<=8; i++){
        spi.write(0xFF);
    }
    cs = 1 ;

}

char AD7124::status(){
    /* read the status register */
    cs = 0;
    spi.write(AD7124_R | AD7124_STATUS_REG);
    char status = spi.write(0x00); 
    //TRACE("ADC status = 0x%X, "BYTE_TO_BINARY_PATTERN"\n", status,BYTE_TO_BINARY(status));
    cs = 1;
    return status;
}

int AD7124::error_reg(){
    /* read the error register */
    cs = 0;
    int sum = 0;
    spi.write(AD7124_R | AD7124_ERR_REG); // read channel 1
    char error_reg[3] = {0};
    TRACE("Error register =");
    for (int i = 0; i<=2; i++){
        error_reg[i] = spi.write(0x00);
        TRACE(" "BYTE_TO_BINARY_PATTERN,BYTE_TO_BINARY(error_reg[i]));
        sum += error_reg[i];
    }
    TRACE("\n");
    cs =1;  
    return sum;
}


void AD7124::read_ID(){
    cs = 0;
    // acces ID
    spi.write(AD7124_R | AD7124_ID_REG);
    //read ID by sending dumm byte 
    int ID = spi.write(0x00);
    printf("ADC ID = 0x%X, "BYTE_TO_BINARY_PATTERN"\n", ID,BYTE_TO_BINARY(ID));
    // Deselect the device
    cs = 1;
}


void AD7124::calibrate(bool internal_zero_scale, bool internal_full_scale, bool system_zero_scale, bool system_full_scale){
    // calibrate the ADC, Full power mode, ref enable
    cs = 0;
    TRACE("Calibration\n");
    if(system_zero_scale){
        spi.write(AD7124_ADC_CTRL_REG);
        char control_reg[2] = {AD7124_ADC_CTRL_REG_REF_EN>>8,
        AD7124_ADC_CTRL_REG_POWER_MODE(3) | AD7124_ADC_CTRL_REG_MODE(0x07)};
        TRACE("SYSTEM ZERO SCALE\n");
        for (int i = 0; i<=1; i++){
            spi.write(control_reg[i]);
        }
        /* wait for the calibration to finish */
        char status = AD7124::status();
        while(status){
            status = AD7124::status();
        }
        TRACE("SYSTEM ZERO SCALE done\n");
    }

    if(system_full_scale){
        spi.write(AD7124_ADC_CTRL_REG); // 
        char control_reg[2] = {AD7124_ADC_CTRL_REG_REF_EN>>8,
        AD7124_ADC_CTRL_REG_POWER_MODE(3) | AD7124_ADC_CTRL_REG_MODE(0x08)}; 
        TRACE("SYSTEM FULL SCALE\n");
        for (int i = 0; i<=1; i++){
            spi.write(control_reg[i]);
        }
        /* wait for the calibration to finish */
        char status = AD7124::status();
        while(status){
            status = AD7124::status();
        }
        TRACE("SYSTEM FULL SCALE done\n");
    }

    if (internal_full_scale){
        spi.write(AD7124_ADC_CTRL_REG); // 
        char control_reg[2] = {AD7124_ADC_CTRL_REG_REF_EN>>8,
        AD7124_ADC_CTRL_REG_POWER_MODE(3) | AD7124_ADC_CTRL_REG_MODE(0x06)}; 
        TRACE("INTERNAL FULL SCALE\n");
        for (int i = 0; i<=1; i++){
            spi.write(control_reg[i]);
        }
        /* wait for the calibration to finish */
        char status = AD7124::status();
        while(status){
        status = AD7124::status();
        }
        TRACE("INTERNAL FULL SCALE done\n");
    }

    if (internal_zero_scale){
        spi.write(AD7124_ADC_CTRL_REG); 

        char control_reg[2] = {0x0D, 0xC0};//AD7124_ADC_CTRL_REG_REF_EN>>8,
        //AD7124_ADC_CTRL_REG_POWER_MODE(3) | AD7124_ADC_CTRL_REG_MODE(0x05)}; 
        TRACE("INTERNAL ZERO SCALE\n");
        for (int i = 0; i<=1; i++){
            spi.write(control_reg[i]);
        }
        /* wait for the calibration to finish */
        char status = AD7124::status();
        while(status != 0){
            INFO("status: %d", status);
            status = AD7124::status();
        }
        TRACE("INTERNAL ZERO SCALE done\n");
    }
    AD7124::ctrl_reg(write);
    cs =1;
}

void AD7124::config_reg(uint8_t address ,char RW){
    /* read/ write the configuration register */
    cs = 0;
    char config_reg[2] = {0};
    if(RW == AD7124::read){
        spi.write(AD7124_R | address);
        TRACE("ADC conf = ");
        for (int i = 0; i<=1; i++){
            config_reg[i] = spi.write(0x00);
            TRACE(" "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(config_reg[i]));
        }
        TRACE("\n");
    }
    // else {
    //     if(address == AD7124_CFG0_REG){
    //         spi.write(address);
    //         char my_config[]={(AD7124_CFG_REG_BIPOLAR) >>8 ,
    //                         0x70};//AD7124_CFG_REG_AIN_BUFP | AD7124_CFG_REG_AINN_BUFM | AD7124_CFG_REG_REF_SEL(2) |AD7124_CFG_REG_PGA(1)};
    //         //original 0x08, 0x71
    //         for (int i = 0; i<=1; i++){
    //             spi.write(my_config[i]);
    //         }
    //     }
    // ^ this was for testing different configs
    else{
        spi.write(address);
        char my_config[]={(AD7124_CFG_REG_BIPOLAR) >>8 ,
                        AD7124_CFG_REG_AIN_BUFP | AD7124_CFG_REG_AINN_BUFM | AD7124_CFG_REG_REF_SEL(2) |AD7124_CFG_REG_PGA(1)};
        //original 0x08, 0x71
        for (int i = 0; i<=1; i++){
            spi.write(my_config[i]);
        } 
        //}
    }
    cs = 1;        

}

void AD7124::ctrl_reg(char RW){
    /* read/write the control register */
    char contr_reg[2] = {0};
    cs = 0;
    if(RW == AD7124::read){
        spi.write(AD7124_R | AD7124_ADC_CTRL_REG);
        TRACE("ADC contr_reg =");

        for (int i = 0; i<=1; i++){
            contr_reg[i] = spi.write(0x00);
            TRACE(" "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(contr_reg[i]));
        }
        TRACE("\n");
    } else {
        spi.write(AD7124_ADC_CTRL_REG);
        char contr_reg_set[]={0x05, 0xC0};//(AD7124_ADC_CTRL_REG_REF_EN)>>8, 
        //AD7124_ADC_CTRL_REG_POWER_MODE(3)};

        for (int i = 0; i<=1; i++){
            spi.write(contr_reg_set[i]);
        }   
    }
    cs = 1;        
}

/* channel_reg
 * Sets up the channel registers.
 * Can setup channel 0, or 1, or both depending on flags
 */
void AD7124::channel_reg(uint8_t channel, char RW){
    //RW=1 -> read else write
    char channel_reg[2] = {0};
    cs = 0;
    if(RW == AD7124::read){
        spi.write(AD7124_R | channel);
        TRACE("Channel register =");
        for (int i = 0; i<=1; i++){
            channel_reg[i] = spi.write(0x00);
            TRACE(" "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(channel_reg[i]));
        }
        TRACE("\n");

    } else {
        //char contr_reg_set[]={0x00,0x08};
        //char channel_reg_set[]={0x82,0x53}; ref and dgnd
        //char channel_reg_set[]={0x82,0x95}; //AVDD - AVss
        // SET CHANNEL 0
        if(flag0 == true){
            spi.write(AD7124_CH0_MAP_REG);
            char channel_reg_set_ch0[]={0x80, 0x01}; //channel 0 and 1 (0x80, 0x01)
                //0x80 for setup 0
            for (int i = 0; i<=1; i++){
                spi.write(channel_reg_set_ch0[i]);
            }
        }
        // SET CHANNEL 1
        if(flag1 == true){
            spi.write(AD7124_CH1_MAP_REG);
            //register bytes set like this -> 10 00 00 (00 - 01 0)(0 00 11)
            //channel 1 - pins (2) and (3)
            //x80 is 1st byte, x43 is 2nd byte (for setting the AIN)
            char channel_reg_set_ch1[]={0x90, 0x43}; //channel 2 and 3 (0x90, 0x43)
                //0x90 for setup 1
            for (int i = 0; i<=1; i++){
                spi.write(channel_reg_set_ch1[i]);
            }
        }
    }
    cs = 1;        
}

/* set_data_status_bit
 * This function is not used, instead the DATA_STATUS bit is hard coded in the ctrl_reg function.
 */
void AD7124::set_data_status_bit() {
    cs = 0;
    uint16_t control_reg_value = spi.write(AD7124_ADC_CTRL_REG);
    INFO("%u", control_reg_value);
    uint8_t first_byte = control_reg_value & 0xFF;
    first_byte |= 0x04;
    control_reg_value = (control_reg_value & 0xFF00) | first_byte;
    spi.write(control_reg_value);
    INFO("%s", "bit set")
    cs = 1;
}

void AD7124::filter_reg(uint8_t filt, char RW){
    //default FS: 384, Post filter = 011
    char filter_reg[3] = {0};
    cs = 0;
    if(RW == AD7124::read){
        spi.write(AD7124_R | filt);
        TRACE("Filter register =");
        for (int i = 0; i<=2; i++){
            filter_reg[i] = spi.write(0x00);
            TRACE(" "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(filter_reg[i]));
        }
        TRACE("\n");
    }
    // else {
    //     if(filt == AD7124_FILT0_REG){
    //         spi.write(filt);
    //         //char contr_reg_set[]={0x00,0x08};
    //         //char filter_reg_set[]={AD7124_FILT_REG_FILTER(4)>>16,0x00,0x40};
    //         char filter_reg_set[]={0x20,0x00,0x40}; //0x00,0x12,0xC0 for testing
    //         for (int i = 0; i<=2; i++){
    //             spi.write(filter_reg_set[i]);
    //         }
    //         printf("reach 0");
    //     }
    // ^ this was for testing different filters
    else{
        spi.write(filt);
        //char contr_reg_set[]={0x00,0x08};
        //char filter_reg_set[]={AD7124_FILT_REG_FILTER(4)>>16,0x00,0x40};
        char filter_reg_set[]={0x00, 0x00, 0x40}; //0x00,0x12,0xC0 for testing
        for (int i = 0; i<=2; i++){
            spi.write(filter_reg_set[i]);
        }
    }
           
    //}
    cs = 1;        
}


/* read_data
 *  DATA_STATUS bit is set, therefore status is appended to end of data.
 *  In the data array, entires 0,1,2 is the data, entry 3 is the status.
 */
uint8_t* AD7124::read_data(void){
    cs = 0;
    spi.write(AD7124_R | AD7124_DATA_REG);
    static uint8_t data [4] = {0};
    for (int i = 0; i<=3; i++){
        data[i] = spi.write(0x00);
        //printf(" "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(data[i]));
    }
    //long measurement =  (((long)data[0] << 24)|((long)data[1] << 16)|((long)data[2] << 8)|((long)data[3]));
    cs = 1;
    return data;
}

uint8_t * AD7124::read_data_byte(){
    //LD =!LD;
    cs = 0;
    spi.write(AD7124_R | AD7124_DATA_REG);
    static uint8_t data [3] = {0};
    char data1 [4] = {0};
    for (int i = 0; i<=2; i++){
        data[i] = spi.write(0x00);
        data1[i] = data[i];
       // printf(" "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(data[i]));
    }
    data1[3] = 0x0A;
    //printf("\n");
    //char bufbuf[] = "In read data byte\n\r";
    //FT232.write(bufbuf, sizeof(bufbuf));

    FT232.write(data1, sizeof(data1));
    long measurement =  (((long)data[0] << 16)|((long)data[1] << 8)|((long)data[2] << 0));
    //printf("measurement %ld 0x%lX, %d, %d, %d\n", measurement, measurement, data[0], data[1], data[2]);

    //AD7124::get_analog_value(true, measurement);
    //char bufbuf1[] = "\n\r";
    //FT232.write(bufbuf1, sizeof(bufbuf1));

    //long measurement =  (((long)data[0] << 16)|((long)data[1] << 8)|((long)data[2] << 0));
    //printf("measurement %ld 0x%lX, %d, %d, %d\n", measurement, measurement, data[0], data[1], data[2]);

    AD7124::get_analog_value(measurement);

    cs = 1;
    return data;
}


uint8_t AD7124::status_ch(uint8_t channel){
    /* read the status register using channel*/
    cs = 0;
    spi.write(AD7124_R | channel);
    uint8_t status = spi.write(0x00);
    cs = 1;
    return status;
}

/* read_thread
 * Continuously reads from the DATA register and sends the data out in a buffer.
 */
void AD7124::read_thread(){

    //std::queue<uint8_t> q_ch0;
    //std::queue<uint8_t> q_ch1;

    uint8_t q_ch0[15] = {0};
    uint8_t q_ch1[15] = {0};

    int countCh1 = 0;
    int countCh0 = 0;

    //continuously get data
    while (1){
        static uint8_t data [3] = {0};
        static uint8_t data1[3] = {0};
        double volt = 0;
        int avg_num = 1;
        long measurement = 0;
        double volt1 = 0;
        long measurement1 = 0;

        uint8_t *cur_data;
        long actual_data;
        bool f0 = !flag0; //these flags are because want entries for both channel 0 and 1
        bool f1 = !flag1; // so only fill the buffer once there is data for both

        // avg counter and for loop stuff will not work right now
        //for ( int avg_counter = 0; avg_counter<avg_num; avg_counter++){
            //if (cur_status >> 7 == 0) {
            while((f0 == false) || (f1 == false)){
                //get the data and status from data array
                cur_data = AD7124::read_data();
                actual_data =  (((long)cur_data[0] << 16)|((long)data[1] << 8)|((long)data[2] << 0));
                //entry 3 in data is the status, read status to find active channel
                //     (1 0 0 0  0 0 0 1)
                //ready ^        ^ ^ ^ ^ active channel num
                //when READY bit is 1, do NOT read, (only read when READY == 0)
                if((cur_data[3] == 0) && (f0 == false)){
                    measurement += actual_data;//AD7124::read_data(); // read data register
                    volt += AD7124::get_analog_value(measurement/1);//(avg_counter+1));

                    f0 = true;
                }

                //add to channel 1 data
                if((cur_data[3] == 1) && (f1 == false)){
                    measurement1 += actual_data;//AD7124::read_data();
                    volt1 += AD7124::get_analog_value(measurement1/1);//(avg_counter+1));

                    f1 = true;
                }
            }
            //}
        //     else{
        //        avg_counter--;
        //     }
        // }

        volt = volt/avg_num;
        INFO("volt: %f", volt)
        measurement = measurement/avg_num;
        INFO("measure: %ld\n", measurement);
        data[2] = measurement         & 0xFF;
        data[1] = (measurement >>  8) & 0xFF;
        data[0] = (measurement >> 16) & 0xFF; 

        volt1 = volt1/avg_num;
        INFO("volt1: %f", volt1)
        measurement1 = measurement1/avg_num;
        INFO("measure1: %ld\n", measurement1);
        data1[2] = measurement1         & 0xFF;
        data1[1] = (measurement1 >>  8) & 0xFF;
        data1[0] = (measurement1 >> 16) & 0xFF;


        bufbuf[0] = 80;      // need start values
        bufbuf[1] = 0x1;     //board number (#1)
        bufbuf[2] = data[0]; //channel 0 data
        bufbuf[3] = data[1];
        bufbuf[4] = data[2];
        //bufbuf[5] = 0x0A;
        bufbuf[5] = data1[0]; //channel 1 data
        bufbuf[6] = data1[1];
        bufbuf[7] = data1[2];
        bufbuf[8] = 0x0A;     //end character

        pc.write(bufbuf, sizeof(bufbuf));

        /*
        mail_t *mail = mail_box.try_alloc();
        mail->voltage = volt;
        mail->ready = true;
        mail->raw_measurement = measurement;
        mail_box.put(mail); // must be freed after in ad7124.h set mailing box length
         */
        //mail_box.free(mail);
    }
}


/*
void AD7124::read_data_byte1(uint8_t *data){
    //LD =!LD;
    cs = 0;
    spi.write(AD7124_R | AD7124_DATA_REG);
    //static uint8_t data [3] = {0};
    for (int i = 0; i<=2; i++){
        data[i] = spi.write(0x00);
       // printf(" "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(data[i]));
    }
    //printf("\n");

    //long measurement =  (((long)data[0] << 16)|((long)data[1] << 8)|((long)data[2] << 0));
    //printf("measurement %ld 0x%lX, %d, %d, %d\n", measurement, measurement, data[0], data[1], data[2]);

    //AD7124::get_analog_value(true, measurement);

    cs = 1;
}*/