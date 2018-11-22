### ESP32 driver library for TI CC1100 Low-Power Sub-1 GHz RF Transceiver

---

Based on [CC1101 library by Christian Weithe](https://github.com/SpaceTeddy/CC1101)

**Included example demo application**

---

#### How to build

Configure your esp32 build environment as for other **esp-idf examples**

Clone the repository

`git clone https://github.com/loboris/ESP32_CC1101.git`

Execute menuconfig and configure your Serial flash config and other settings. Included *sdkconfig.defaults* sets some defaults to be used.

Navigate to **CC1101 Example configuration** and set CC1101 and example parameters:

`make menuconfig`

Make and flash the example.

`make all && make flash`


**Example output:**

```
I (0) cpu_start: App cpu up.
I (1732) heap_alloc_caps: Initializing. RAM available for dynamic allocation:
I (1802) heap_alloc_caps: At 3FFAE2A0 len 00001D60 (7 KiB): DRAM
I (1864) heap_alloc_caps: At 3FFB2548 len 0002DAB8 (182 KiB): DRAM
I (1927) heap_alloc_caps: At 3FFE0440 len 00003BC0 (14 KiB): D/IRAM
I (1991) heap_alloc_caps: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (2056) heap_alloc_caps: At 40088304 len 00017CFC (95 KiB): IRAM
I (2119) cpu_start: Pro cpu start user code
I (2273) cpu_start: Starting scheduler on PRO CPU.
I (578) cpu_start: Starting scheduler on APP CPU.
I (3634) [CC1101]: NVS Initialized
I (3634) gpio: GPIO[25]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0 
I (3638) [CC1101]: ADC channel: 0 [GPIO: 36]
I (3642) [CC1101]: SPI: speed=5000000  native pins: true
I (3647) [CC1101]: SPI initialized
I (3652) [CC1101]: CC1100 Reset
I (3656) [CC1101]: Partnumber: 00 Version: 14
I (3661) [CC1101]: Initialization done.
     Mode: 2
Frequency: 3
  Channel: 1
    Power: 5
  My_Addr: 3
Cfg_reg:
06 2e 80 07 57 43 3e 0e 45 03 
01 06 00 21 65 6a ca 83 13 a0 
f8 34 07 0c 19 16 6c 43 40 91 
02 26 09 56 05 a9 0a 00 11 49 
00 59 7f 3f 81 3f 0b 
PaTable:
03 17 1d 26 50 86 cd c0 
I (3689) [CC1101]: Started.
ADC: 3074  Voltage: 816.7 mV
Temperature: 29.6
CC1101 Temperature: 29.6
I (5752) [CC1101 RXtask]: === CC1101 RX TASK STARTED on core 1 ===
I (6751) [CC1101 TXtask]: === CC1101 TX TASK STARTED on core 1 ===

W (6751) [CC1101 TXtask]: ---------------------------TX---------------------------
ADC: 3084  Voltage: 819.4 mV
Temperature: 30.7
TX_FIFO: 0a 03 03 00 00 18 5c 29 8e f5 41 
E (6928) [CC1101]: SEND FAILED, too many retries (0)
E (6928) [CC1101 TXtask]: [1/1] Error
W (6928) [CC1101 TXtask]: ^^^^^^^^^^^^^^^^^^^^^^^^^^^TX^^^^^^^^^^^^^^^^^^^^^^^^^^^
I (16149) [CC1101]: Packet available

W (16150) [CC1101 RXtask]: ---------------------------RX---------------------------
RX_FIFO: 0a 03 03 00 00 18 39 0b 0a 96 41 2c ab 
I (16157) [CC1101]: RSSI: -56  LQI: 2b  CRC: OK
TX_FIFO: 05 03 03 41 63 6b 
I (16170) [CC1101]: ACK sent.
I (16170) [CC1101 RXtask]: [1/2] Rx_Time: 19 ms  TX_Timecode: 6201 ms Temperature: 18.8  RSSI: -56  LQI: 43  CRC: OK
W (16178) [CC1101 RXtask]: ^^^^^^^^^^^^^^^^^^^^^^^^^^^RX^^^^^^^^^^^^^^^^^^^^^^^^^^^
W (16187) [CC1101 RXtask]: NEXT TX time adjusted

W (21235) [CC1101 TXtask]: ---------------------------TX---------------------------
ADC: 3115  Voltage: 827.6 mV
Temperature: 34.0
TX_FIFO: 0a 03 03 00 00 50 ec 91 1d 08 42 
RX_FIFO: 05 03 03 41 63 6b 2c a7 
I (21324) [CC1101]: [ACK!] RSSI: -56  LQI: 27  CRC: 80
I (21324) [CC1101]: SEND OK after 0 retries.
I (21326) [CC1101 TXtask]: [1/2] OK; Tx_Time: 32 ms [RSSI: -56  LQI: 39  CRC: OK]
W (21334) [CC1101 TXtask]: ^^^^^^^^^^^^^^^^^^^^^^^^^^^TX^^^^^^^^^^^^^^^^^^^^^^^^^^^
I (26156) [CC1101]: Packet available

W (26156) [CC1101 RXtask]: ---------------------------RX---------------------------
RX_FIFO: 0a 03 03 00 00 3f 50 ac ce a3 41 2c aa 
I (26163) [CC1101]: RSSI: -56  LQI: 2a  CRC: OK
TX_FIFO: 05 03 03 41 63 6b 
I (26177) [CC1101]: ACK sent.
I (26177) [CC1101 RXtask]: [2/6] Rx_Time: 19 ms  TX_Timecode: 16208 ms Temperature: 20.5  RSSI: -56  LQI: 42  CRC: OK
W (26185) [CC1101 RXtask]: ^^^^^^^^^^^^^^^^^^^^^^^^^^^RX^^^^^^^^^^^^^^^^^^^^^^^^^^^

W (31242) [CC1101 TXtask]: ---------------------------TX---------------------------
ADC: 3117  Voltage: 828.1 mV
Temperature: 34.2
TX_FIFO: 0a 03 03 00 00 78 03 db f9 08 42 
RX_FIFO: 05 03 03 41 63 6b 2c a5 
I (31331) [CC1101]: [ACK!] RSSI: -56  LQI: 25  CRC: 80
I (31331) [CC1101]: SEND OK after 0 retries.
I (31333) [CC1101 TXtask]: [2/3] OK; Tx_Time: 32 ms [RSSI: -56  LQI: 37  CRC: OK]
W (31341) [CC1101 TXtask]: ^^^^^^^^^^^^^^^^^^^^^^^^^^^TX^^^^^^^^^^^^^^^^^^^^^^^^^^^
I (36163) [CC1101]: Packet available

W (36163) [CC1101 RXtask]: ---------------------------RX---------------------------
RX_FIFO: 0a 03 03 00 00 66 67 b3 d4 a9 41 2c aa 
I (36170) [CC1101]: RSSI: -56  LQI: 2a  CRC: OK
TX_FIFO: 05 03 03 41 63 6b 
I (36183) [CC1101]: ACK sent.
I (36184) [CC1101 RXtask]: [3/10] Rx_Time: 20 ms  TX_Timecode: 26215 ms Temperature: 21.2  RSSI: -56  LQI: 42  CRC: OK
W (36192) [CC1101 RXtask]: ^^^^^^^^^^^^^^^^^^^^^^^^^^^RX^^^^^^^^^^^^^^^^^^^^^^^^^^^

```
