
#ifndef ADC_PROCESS_H_
#define ADC_PROCESS_H_

#include "DigitalOut.h"
#include "PinNames.h"
#include "mbed.h"
#include "hal/include/hal/spi_api.h"
#include "mstd_iterator"
#include <cstdint>
#include <cstdio>
#include <ad7124-defs.h>
#include <utils.h>
#include <logger.h>

inline SPI spi(PA_7, PA_6, PA_5); // mosi, miso, sclk
inline DigitalOut cs(PA_4); //(PA_4) - nucleo ;//(PB_2) - dongle;

class AD7124: private mbed::NonCopyable<AD7124>{
    public:
        //AD7124(events::EventQueue &event_queue, SendRandomNumberService &SRNS):

        // DigitalOut LD =LED2;
        bool trace = true;
        char read = 1;
        char write = 0;
        int readADC_ID; //used for callback
        //bool isconnected;
        char status1;
        int error = 0;
        void init(bool f0, bool f1, uint8_t channel);
        double get_analog_value(long measurement);
        //void reset();
        char status();
        int error_reg();
        void read_ID();
        void calibrate(bool internal_zero_scale, bool internal_full_scale, bool system_zero_scale, bool system_full_scale);
        void channel_reg(uint8_t channel, char RW);
        //void channel_reg1(char RW);
        uint8_t* read_data(void); //used to return long
        void filter_reg(uint8_t filt, char RW);
        uint8_t *read_data_byte(void);
        void read_thread();
        //void read_data_byte1(uint8_t *data[3]);
        typedef struct {
            double    voltage; /* AD result of measured voltage */
            char     ready; /* AD result of measured current */
            long  raw_measurement; /* A counter value               */
        } mail_t;
        Mail<mail_t, 2> mail_box;
        void reset();
        void config_reg (uint8_t adress, char RW);
        void ctrl_reg(char RW);
        uint8_t read_channel_reg(uint8_t channel);
        uint8_t status_ch(uint8_t channel);
        void set_data_status_bit();

        //void read_data_byte1(uint8_t *data[3]);

    private:
        const double databits 				    = 8388608;  //2^23
        const double Vref 						= 2.5;		//
        const double Gain 						= 2;

};
#endif /* GATT_SERVER_EXAMPLE_BLE_PROCESS_H_ */