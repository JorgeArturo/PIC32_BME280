/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdio.h>

#include "bme280/bme280.h"
#include "driver/i2c/drv_i2c.h"
#include "system/time/sys_time.h"
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
// *****************************************************************************
/* Structure that contains identifier details used in example */
struct identifier
{
    /* Variable to hold device address */
    uint8_t dev_addr;

    /* Variable that contains file descriptor */
    int8_t fd;
};

struct identifier id;
struct bme280_dev dev;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr){
    
    struct identifier id;

    uint8_t wbuff[2] = {reg_addr,0};
    
    id = *((struct identifier *)intf_ptr);
     
    if(DRV_I2C_WriteReadTransfer(appData.i2c_handler,id.dev_addr,&wbuff,1,&data,len)){
    
        return BME280_OK;
    
    }
    else
    {
    
        return BME280_E_COMM_FAIL;
    
    }

}
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr){

    struct identifier id;

    id = *((struct identifier *)intf_ptr);
    
    uint8_t wbuff[2] = {reg_addr};
    
    if(DRV_I2C_WriteTransfer(appData.i2c_handler,id.dev_addr,&wbuff,(size_t)1))
    {
    
        if(DRV_I2C_WriteTransfer(appData.i2c_handler,id.dev_addr,&data,(size_t)len))
        {

            return BME280_OK;

        }else
        {

            return BME280_E_COMM_FAIL;

        }
    
    }else
    {
    
        return BME280_E_COMM_FAIL;
    
    }

}
void user_delay_us(uint32_t period, void *intf_ptr)
{
    SYS_TIME_HANDLE timer = SYS_TIME_HANDLE_INVALID;
    
    if (SYS_TIME_DelayUS(period, &timer) != SYS_TIME_SUCCESS)
    {
        printf("Time Error\r\n");
    }
    else if (SYS_TIME_DelayIsComplete(timer) != true)
    {
        // Wait till the delay has not expired
        while (SYS_TIME_DelayIsComplete(timer) == false);
    }


}

/*!
 * @brief This API reads the sensor temperature, pressure and humidity data in forced mode.
 */
int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t settings_sel;
    struct bme280_data comp_data;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, dev);
    if (rslt != BME280_OK)
    {
        printf("Failed to set sensor settings (code %+d).\n",rslt);
        return rslt;
    }

    printf("Temperature, Pressure, Humidity\n");

    /* Continuously stream sensor data */
    while (1)
    {
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
        if (rslt != BME280_OK)
        {
            printf("Failed to set sensor mode (code %+d).\n",rslt);
            break;
        }

        /* Wait for the measurement to complete and print data @25Hz */
        dev->delay_us(40000, dev->intf_ptr);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
        if (rslt != BME280_OK)
        {
            printf("Failed to get sensor data (code %+d).\n",rslt);

            break;
        }
    
        printf("%0.1f %0.1f %0.1f \r\n",comp_data.pressure,comp_data.temperature,comp_data.humidity);
        dev->delay_us(1000000, dev->intf_ptr);
    }

    return rslt;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;



    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            
            id.dev_addr = BME280_I2C_ADDR_PRIM;
            dev.intf = BME280_I2C_INTF;
            dev.read = user_i2c_read;
            dev.write = user_i2c_write;
            dev.delay_us = user_delay_us;
            dev.intf_ptr = &id;
            
            printf("Initializing I2C...");
    
            appData.i2c_handler = DRV_I2C_Open(DRV_I2C_INDEX_0,DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_BLOCKING | DRV_IO_INTENT_EXCLUSIVE); 
                    
            if (appData.i2c_handler != DRV_HANDLE_INVALID)
            {
                printf("OK\r\n");
                printf("Initializing BME280...");
                
                if (bme280_init(&dev) == BME280_OK)
                {
                    printf("OK\r\n");
                    appData.state = APP_STATE_SERVICE_TASKS;
                }else
                {
                    printf("NOK\r\n");
                    appData.state = APP_STATE_ERROR;

                }   
            }else
            {
                printf("NOK\r\n");
                appData.state = APP_STATE_ERROR;
                
            }
            

            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            if(stream_sensor_data_forced_mode(&dev) != BME280_OK){
            
                appData.state = APP_STATE_ERROR;
            
            }

            break;
        }

        /* TODO: implement your application state machine.*/


        case APP_STATE_ERROR:
        {    
            DRV_I2C_Close(appData.i2c_handler);
            
            printf("Error to init BME280 \r\n");
            appData.state = APP_STATE_IDLE;
            break;
        
        }
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}


/*******************************************************************************
 End of File
 */
