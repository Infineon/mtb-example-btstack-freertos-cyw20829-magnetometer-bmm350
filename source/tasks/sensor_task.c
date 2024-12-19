/*******************************************************************************
 Copyright 2020-2024, Cypress Semiconductor Corporation (an Infineon company) or
 an affiliate of Cypress Semiconductor Corporation.  All rights reserved.

 This software, including source code, documentation and related
 materials ("Software") is owned by Cypress Semiconductor Corporation
 or one of its affiliates ("Cypress") and is protected by and subject to
 worldwide patent protection (United States and foreign),
 United States copyright laws and international treaty provisions.
 Therefore, you may use this Software only as provided in the license
 agreement accompanying the software package from which you
 obtained this Software ("EULA").
 If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 non-transferable license to copy, modify, and compile the Software
 source code solely for use in connection with Cypress's
 integrated circuit products.  Any reproduction, modification, translation,
 compilation, or representation of this Software except as specified
 above is prohibited without the express written permission of Cypress.

 Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 reserves the right to make changes to the Software without notice. Cypress
 does not assume any liability arising out of the application or use of the
 Software or any product or circuit described in the Software. Cypress does
 not authorize its products for use in any products where a malfunction or
 failure of the Cypress product may reasonably be expected to result in
 significant property damage, injury or death ("High Risk Product"). By
 including Cypress's product in a High Risk Product, the manufacturer
 of such system or application assumes all risk of such use and in doing
 so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/*******************************************************************************
 *        Header Files
 *******************************************************************************/
#include "cyhal.h"
#include "sensor_task.h"
#include "cy_retarget_io.h"
#include "timers.h"

#include "app_bt_gatt_handler.h"

#include "mtb_bmm350.h"

/*******************************************************************************
 *        Macro Definitions
 *******************************************************************************/
#define POLL_TIMER_IN_MSEC                 (1000u)
#define POLL_TIMER_FREQ                    (10000)

/* I2C Clock frequency in Hz */
#define I2C_CLK_FREQ_HZ                    (400000U)

/* BMM350 I2C Address */
#define BMM350_I2C_ADDRESS                 (BMM350_I2C_ADSEL_SET_LOW)

/* Check if notification is enabled for a valid connection ID */
#define IS_NOTIFIABLE(conn_id, cccd)       (((conn_id)!= 0)? (cccd) & GATT_CLIENT_CONFIG_NOTIFICATION: 0)

#define BMM350_MAG_TEMP_X_INDEX            (0)
#define BMM350_MAG_TEMP_Y_INDEX            (1)
#define BMM350_MAG_TEMP_Z_INDEX            (2)
#define BMM350_MAG_TEMP_TEMP_INDEX         (3)

/*******************************************************************************
 *        Variable Definitions
 *******************************************************************************/
/* Structure holding i2c obj */
cyhal_i2c_t i2c_obj;

/* Structure holding bmm350 variables */
mtb_bmm350_t bmm350_obj;

/* Handle to timer object */
TimerHandle_t timer_handle;

/******************************************************************************
 *                          Function Prototypes
 ******************************************************************************/
void timer_callback(TimerHandle_t xTimer);

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
int32_t sensor_task_init()
{
    cy_rslt_t result;

    /* Create timer */
    timer_handle = xTimerCreate("timer",
                                pdMS_TO_TICKS(POLL_TIMER_IN_MSEC),
                                pdTRUE,
                                NULL,
                                timer_callback);
    if (NULL == timer_handle)
    {
        return -1;
    }

    /* Start Timer */
    xTimerStart(timer_handle, 0);

    /* Initialize sensor driver here */

    /* I2C Configuration Structure */
    cyhal_i2c_cfg_t i2c_config = { false, 0, I2C_CLK_FREQ_HZ };

    /* Initialize I2C for BMI270 */
    result = cyhal_i2c_init(&i2c_obj, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error initializing i2c\n");
        return -2;
    }

    /* Configure the I2C Interface with the desired clock frequency */
    result = cyhal_i2c_configure(&i2c_obj, &i2c_config);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error configuring i2c\n");
        return -3;
    }

    /* Initialize BMM350 Driver */
    result = mtb_bmm350_init_i2c(&bmm350_obj, &i2c_obj, MTB_BMM350_ADDRESS_DEFAULT);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error initializing BMM350 driver\n");
        return -4;
    }

    return 0;
}

void sensor_task(void *pvParameters)
{
    cy_rslt_t result;

    /* BMM350 Structures */
    mtb_bmm350_data_t data;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        result = mtb_bmm350_read(&bmm350_obj, &data);
        if (CY_RSLT_SUCCESS == result)
        {
            int abs_x, abs_y, abs_z, abs_temp;

            abs_x = abs((int) data.sensor_data.x);
            abs_y = abs((int) data.sensor_data.y);
            abs_z = abs((int) data.sensor_data.z);
            abs_temp = abs((int) data.sensor_data.temperature);

            printf("x: %d, y: %d, z: %d, temperature: %d degC\r\n\n", abs_x,
                                                                      abs_y,
                                                                      abs_z,
                                                                      abs_temp);

            /* Copy data to BLE read buffer */
            app_xensiv_sensor_shield_bmm350[BMM350_MAG_TEMP_X_INDEX] = (uint8_t) abs_x;
            app_xensiv_sensor_shield_bmm350[BMM350_MAG_TEMP_Y_INDEX] = (uint8_t) abs_y;
            app_xensiv_sensor_shield_bmm350[BMM350_MAG_TEMP_Z_INDEX] = (uint8_t) abs_z;
            app_xensiv_sensor_shield_bmm350[BMM350_MAG_TEMP_TEMP_INDEX] = (uint8_t) abs_temp;
        }
        else
        {
            /* Don't send anything up to app if sensor data not ready */
            continue;
        }

        if (IS_NOTIFIABLE (app_bt_conn_id, app_xensiv_sensor_shield_bmm350_client_char_config[0]) == 0)
        {
            if (!app_bt_conn_id)
            {
                printf("This device is not connected to a central device\n");
            }
            else
            {
                printf("This device is connected to a central device but\n"
                        "GATT client notifications are not enabled\n");
            }
        }
        else
        {
            wiced_bt_gatt_status_t gatt_status;

            /*
            * Sending notification, set the pv_app_context to NULL, since the
            * data 'app_xensiv_sensor_shield_bmm350' is not to be freed
            */
            gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                                 HDLC_XENSIV_SENSOR_SHIELD_BMM350_VALUE,
                                                                 app_xensiv_sensor_shield_bmm350_len,
                                                                 (uint8_t *) app_xensiv_sensor_shield_bmm350,
                                                                 NULL);

            printf("Sent notification status 0x%x\n", gatt_status);
        }
    }
}

void timer_callback(TimerHandle_t xTimer)
{
    (void) xTimer;

    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(sensor_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
