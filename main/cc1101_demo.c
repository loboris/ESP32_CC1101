/* PPPoS Client Example with GSM
 *  (tested with SIM800)
 *  Author: LoBo (loboris@gmail.com, loboris.github)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */


#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#include "driver/gpio.h"

#include "libcc1100.h"

#define TX_INTERVAL 10000			// Transmit interval in milliseconds
#define ESP_INTR_FLAG_DEFAULT 0
#define TASK_SEMAPHORE_WAIT 5000	// time to wait for mutex in milliseconds

static QueueHandle_t rxtx_task_mutex = NULL;
static QueueHandle_t gdo2_evt_queue = NULL;
static uint8_t ignore_GDO2_int = 1;
static uint32_t next_tx_time = 0;

static const char tag[] = "[CC1101 Demo]";
static const char txtag[] = "[CC1101 TXtask]";
static const char rxtag[] = "[CC1101 RXtask]";

//Global CC1100 address
static uint8_t My_addr;


//-----------------------------------------------
 static void IRAM_ATTR GDO2_isr_handler(void* arg)
 {
     uint32_t gpio_num = (uint32_t) arg;
     /*if (!ignore_GDO2_int)*/ xQueueSendFromISR(gdo2_evt_queue, &gpio_num, NULL);
 }


//===============================
static void CC_RX_task(void* arg)
{
	ESP_LOGI(rxtag, "=== CC1101 RX TASK STARTED on core %d ===", xPortGetCoreID());

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for GDO2 pin
    gpio_isr_handler_add(GDO2, GDO2_isr_handler, (void*) GDO2);

    gpio_set_intr_type(GDO2, GPIO_INTR_POSEDGE);
	gpio_intr_enable(GDO2);	// enable pin change interrupt
	ignore_GDO2_int = 0;

    uint8_t Rx_fifo[FIFOBUFFER];
	uint32_t io_num, n_rx = 0, nrx_ok = 0, nrx_err = 0;
	uint32_t rf_timecode = 0;
	uint8_t rx_addr, sender, pktlen;
	uint32_t time_stamp;

	while(1) {
        if (xQueueReceive(gdo2_evt_queue, &io_num, portMAX_DELAY)) {
        	if (io_num == GDO2) {
				// ==== GDO2 Low->High change detected ====
				if (!(xSemaphoreTake(rxtx_task_mutex, TASK_SEMAPHORE_WAIT))) {
					ESP_LOGE(rxtag, "*** ERROR: CANNOT GET MUTEX ***");
					vTaskDelay(5000 / portTICK_PERIOD_MS);
					continue;
				}

				n_rx++;
				if (packet_available() == 1) {
					// === valid package is received ===
					printf("\r\n");
					ESP_LOGW(rxtag, "---------------------------RX---------------------------");
					time_stamp = clock();	// generate new time stamp
					uint8_t res = get_payload(Rx_fifo, &pktlen, &rx_addr, &sender);
					time_stamp = clock()-time_stamp;
					if (res == 1) {
						nrx_ok++;
						rf_timecode = ((uint32_t)Rx_fifo[3] << 24) +
									  ((uint32_t)Rx_fifo[4] << 16) +
									  ((uint16_t)Rx_fifo[5] <<  8) +
												 Rx_fifo[6];
						float tmp;
						memcpy(&tmp, Rx_fifo+7, sizeof(float));

						ESP_LOGI(rxtag, "[%u/%u] Rx_Time: %u ms  TX_Timecode: %d ms Temperature: %3.1f  RSSI: %d  LQI: %d  CRC: %s",
								nrx_ok, n_rx, time_stamp, rf_timecode, tmp, last_rssi_dbm, last_lqi, ((last_crc) ? "OK" : "BAD"));

					}
					else {
						nrx_err++;
						ESP_LOGE(rxtag, "[%u/%u] Error %d", nrx_ok, n_rx, res);
					}
					ESP_LOGW(rxtag, "^^^^^^^^^^^^^^^^^^^^^^^^^^^RX^^^^^^^^^^^^^^^^^^^^^^^^^^^");

					// Adjust next tx time
					int next_tx = next_tx_time - clock();
					if ((next_tx < (TX_INTERVAL/2-1000)) || (next_tx > (TX_INTERVAL/2+1000))) {
						next_tx_time = clock() + (TX_INTERVAL/2);
						ESP_LOGW(rxtag, "NEXT TX time adjusted");
					}
				}

				xSemaphoreGive(rxtx_task_mutex);
        	}
        }
    }

	ESP_LOGW(rxtag, "=== CC1101 RX TASK ENDED ===");
	vTaskDelete(NULL);
}

//===============================
static void CC_TX_task(void* arg)
{
	ESP_LOGI(txtag, "=== CC1101 TX TASK STARTED on core %d ===", xPortGetCoreID());

	uint32_t time_stamp;
	uint32_t n_tx = 0, n_tx_ok = 0, n_tx_err = 0;
	uint8_t Tx_fifo[FIFOBUFFER];
	uint8_t Rx_addr, Pktlen;

	while (1) {
		if (clock() < next_tx_time) {
			vTaskDelay(100 / portTICK_RATE_MS);
			continue;
		}

		if (!(xSemaphoreTake(rxtx_task_mutex, TASK_SEMAPHORE_WAIT))) {
			ESP_LOGE(txtag, "*** ERROR: CANNOT GET MUTEX ***");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
		}
		next_tx_time = clock() + TX_INTERVAL;
		ignore_GDO2_int = 1;

		printf("\r\n");
		ESP_LOGW(txtag, "---------------------------TX---------------------------");

		n_tx++;
		Rx_addr = 0x03;										// receiver address
		float tmp = get_temp(0); // Get temperature

		time_stamp = clock();								// generate new time stamp
		Tx_fifo[3] = (uint8_t)(time_stamp >> 24);			// split 32-Bit time stamp to 4 byte array
		Tx_fifo[4] = (uint8_t)(time_stamp >> 16);
		Tx_fifo[5] = (uint8_t)(time_stamp >> 8);
		Tx_fifo[6] = (uint8_t)(time_stamp);
		memcpy(Tx_fifo+7, &tmp, sizeof(float));

		Pktlen = 0x07+sizeof(float);						// set packet len to 0x13

		uint8_t res = send_packet(My_addr, Rx_addr, Tx_fifo, Pktlen, 0);	// send package over air. ACK is received via GPIO polling

		time_stamp = clock()-time_stamp;
		if (res) {
			n_tx_ok++;
			ESP_LOGI(txtag, "[%u/%u] OK; Tx_Time: %u ms [RSSI: %d  LQI: %d  CRC: %s]", n_tx_ok, n_tx, time_stamp, last_rssi_dbm, last_lqi, ((last_crc) ? "OK" : "BAD"));
		}
		else {
			n_tx_err++;
			ESP_LOGE(txtag, "[%u/%u] Error", n_tx_err, n_tx);
		}
		ESP_LOGW(txtag, "^^^^^^^^^^^^^^^^^^^^^^^^^^^TX^^^^^^^^^^^^^^^^^^^^^^^^^^^");

		ignore_GDO2_int = 0;
		xSemaphoreGive(rxtx_task_mutex);
	}

	ESP_LOGW(rxtag, "=== CC1101 TX TASK ENDED ===");
	vTaskDelete(NULL);
}


//=============
void app_main()
{
	rxtx_task_mutex = xSemaphoreCreateMutex();

	//create a queue to handle GDO2 event from isr
    gdo2_evt_queue = xQueueCreate(10, sizeof(uint32_t));

	vTaskDelay(3000 / portTICK_RATE_MS);

	uint8_t res = cc_setup(&My_addr, 1);
	if (res > 0) {

		float tmp = get_temp(2); // Get temperature
        if (tmp > -999) printf("CC1101 Temperature: %3.1f\r\n", tmp);

        set_output_power_level(-10);

        // === Start tasks ===
		xTaskCreatePinnedToCore(CC_RX_task, "CC_RX_task", 2048, NULL, 9, NULL, 1);
		vTaskDelay(1000 / portTICK_RATE_MS);
		xTaskCreatePinnedToCore(CC_TX_task, "CC_TX_task", 2048, NULL, 8, NULL, 1);
	}
	else {
		ESP_LOGE(tag, "ERROR initializing CC1101.");
	}

	while(1)
	{
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}
