# Recognition with Command Line in Single Chip

This example demonstrates **Human Face Recognition** with a single ESP32 chip (without using any LCD module). ESP32 firstly gets images that are captured by the camera module, then determines if there are any recognized human faces as well as displays its **Recognition Results** in the **Serial Terminal**.

# Preparation

To run this example, you need the following components:

* An ESP32 Module: **ESP32-WROVER**, which we highly recommend for beginners, is used in this example.
* A Camera Module: the **OV2640** image sensor, which we highly recommend for beginners, is used in this example.
* SDKs:
	* [ESP-IDF](https://github.com/espressif/esp-idf)
	* [ESP-WHO](https://github.com/espressif/esp-who)

For the detailed introduction about preparation, please see [here](https://github.com/espressif/esp-who).


# Quick Start


After you've completed the hardware settings, please follow the steps below:

1. **Connect** the camera to ESP32 module;
2. **Flash Application** to ESP32;
3. **Start Human Face Recognition** and **Check Detection Results**.

## Connect

The table below lists the specific pins used in this example for connecting the ESP32 module and the camera module.

| Interface | Camera Pin | Pin Mapping for ESP32-WROVER |
| :--- | :---: | :---: |
| SCCB Clock | SIOC | IO27 |
| SCCB Data | SIOD | IO26 |
| System Clock | XCLK | IO21 |
| Vertical Sync | VSYNC | IO25 |
| Horizontal Reference | HREF | IO23 |
| Pixel Clock | PCLK | IO22 |
| Pixel Data Bit 0 | D2 | IO4 |
| Pixel Data Bit 1 | D3 | IO5 |
| Pixel Data Bit 2 | D4 | IO18 |
| Pixel Data Bit 3 | D5 | IO19 |
| Pixel Data Bit 4 | D6 | IO36 |
| Pixel Data Bit 5 | D7 | IO39 |
| Pixel Data Bit 6 | D8 | IO34 |
| Pixel Data Bit 7 | D9 | IO35 |
| Camera Reset | RESET | IO2 |
| Camera Power Down | PWDN | IO0 |
| Power Supply 3.3V | 3V3 | 3V3 |
| Ground | GND | GND |

> The pin mapping will be slightly different if you use other ESP32 modules.

In particular, if you are using a **ESP-WROVER-KIT** for your development, whose camera connector is already broken out (the one labeled Camera / JP4), please follow the steps below:

1. Plug your camera module, i.e. the OV2640 module in this example, on the board;
2. Connect the 3V3 and GND pins on the camera module to those counterparts on the board.

The image below shows a **ESP-WROVER-KIT** development board with a **OV2640** camera module installed on it.

![esp_wrover_kit_with_ov2640](../../../img/esp_wrover_kit_with_ov2640.png)  

## Flashing to ESP32

Please see [here](https://github.com/espressif/esp-who).

## Checking Results

1. Put your camera module away from a human face for at least 0.3 m;
2. Open a Serial Terminal by using the command line `make monitor`;
3. Check result at your Serial Terminal, and you will be able to see information as displayed in the screenshot below, which indicates the **Face Enrollment** will start soon:

	![login_delay2](../../../img/enroll_start_count_down.png)

### Enrolling a Face ID

To successfully enroll a **Face ID**, ESP32 will collect a certain number of samples of a user's face, which is configurable and 3 by default. To be more specific, by default, ESP32 will collect three samples of a user's face to enroll a new **Face ID**.

![start_enrollment_1](../../../img/enrollment_take_1st_sample.png)
![start_enrollment_2](../../../img/enrollment_take_2nd_sample.png)
![start_enrollment_3](../../../img/enrollment_take_3rd_sample.png)
![errolled_face_id](../../../img/errolled_face_id.png)

### Recognizing a Face ID

After the **Face ID Enrollment**, ESP32 starts the **Face Recognition**:

![start_recognition](../../../img/start_recognition.png)

ESP32 checks if the newly detected face matches any existing **Face ID**, whenever it detects a face:

* If Yes, the Serial Terminal displays the corresponding **Face ID**:

	![recognition_matched](../../../img/matched.png)

* If No, the Serial Terminal displays `No Matched ID`:

	![recognition_no_matched](../../../img/no_matched.png)


## Advance Configuration

Users can change the configuration by adjusting some macro definitions specified in the `app_facenet.h` file:

- `ENROLL_CONFIRM_TIMES`: the number of face samples required to enroll one new **Face ID**. By default, this parameter is set to 3, indicating three face samples are required to enroll a new **Face ID**.
- `FACE_ID_SAVE_NUMBER`: the number of **Face IDs** that are allowed to be enrolled. By default, this parameter is set to 1, indicating only one **Face ID** can be stored in the RAM when the system boots up. Users can configure this parameter to a bigger value if they want to enroll more than one **Face ID**. 

Users can also store the enrolled **Face IDs** in the flash of the board, so the existing **Face IDs** won't be lost when the board powers off. To achieve this, please use the following functions, provided in `esp-face`:

- `enroll_to_flash()`: Stores the enrolled **Face IDs** in the flash
- `read_id_from_flash()`: Reads all the enrolled **Face IDs** stored in the flash
- `delete_id_in_flash()`: Deletes the earliest enrolled **Face IDs** stored in the flash

For the detailed description of more parameters for face recognition, please see [Here](https://github.com/espressif/esp-face/tree/master/face_recognition).
