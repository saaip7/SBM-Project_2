/**
 * Author: Syaifullah Hilmi M | 22/497775/TK/54567
 * Universitas Gadjah Mada
 * Github: https://github.com/saaip7/SBM-Project_2
 * Description: This project integrates multiple sensors (DHT22, HC-SR04) with an ESP32 and displays
 *              the sensor data on a TFT screen using LVGL. The touch functionality is handled using 
 *              an XPT2046 touchscreen controller.
 * 
 * Sensors Used:
 * - DHT22 (Temperature and Humidity Sensor)
 * - HC-SR04 (Ultrasonic Distance Sensor)
 * 
 * Device Used:
 * - ESP32 WROOM-32
 * - TFT ILI9341 2.4" 240x320
 * 
 * LVGL Layout:
 * The LVGL layout was created using SquareLine Studio, which simplifies the process of designing
 * and managing LVGL-based GUIs.
 */

#include <lvgl.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <ui.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// DHT Sensor
#define DHTPIN 19     // Pin connected to the DHT sensor
#define DHTTYPE DHT22 // DHT 22 (AM2302)
DHT dht(DHTPIN, DHTTYPE);

// HC-SR04 Sensor
#define TRIG_PIN 5 // Pin connected to the Trig pin of HC-SR04
#define ECHO_PIN 18 // Pin connected to the Echo pin of HC-SR04

// Touchscreen pins
#define XPT2046_IRQ 36   // T_IRQ
#define XPT2046_MOSI 32  // T_DIN
#define XPT2046_MISO 39  // T_OUT
#define XPT2046_CLK 25   // T_CLK
#define XPT2046_CS 33    // T_CS

// Change to your screen resolution
static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 240;

// Setting up display buffer for LVGL
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10];

//Declare All screen needed
SPIClass touchscreenSPI = SPIClass(VSPI);
TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); // TFT instance
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

#if LV_USE_LOG != 0
// Serial debugging
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

// Display flushing
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    // Calculate the width and height of the area to be flushed
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    // Set the address window on the TFT display to the area
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);

    // Push the color data to the display
    tft.pushColors((uint16_t *)&color_p->full, w * h, true);

    // Signal to LVGL that the flushing is complete
    tft.endWrite();
    lv_disp_flush_ready(disp);
}

// Read the touchpad
void my_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data)
{
    uint16_t touchX = 0, touchY = 0;

    // Check if the touchpad is not touched
    if (!touchscreen.tirqTouched() && !touchscreen.touched())
    {
        // Set the state to release
        data->state = LV_INDEV_STATE_REL;
        
    }
    else{
        // Set the state to press
        data->state = LV_INDEV_STATE_PR;
        // Get the touch coordinates
        TS_Point p = touchscreen.getPoint();

        // Map the touch coordinates to the screen size
        // The touchpad coordinates go from 200 to 3700 in x axis and 240 to 3800 in y axis
        // We need to map these to the screen size of screenWidth and screenHeight
        touchX = map(p.x, 200, 3700, 1, screenWidth); 
        touchY = map(p.y, 240, 3800, 1, screenHeight);

        // Set the coordinates
        data->point.x = touchX;
        data->point.y = touchY;

        // Print the touchpad coordinates and pressure
        Serial.print("Data x: ");
        Serial.println(touchX);
        Serial.print("Data y: ");
        Serial.println(touchY);
        Serial.print("Pressure: ");
        Serial.println(p.z);
    }
}

//Declare All Variables needed in Global
float temperature = 0.0;
float hum = 0.0;
float distance = 0.0;

// Create Task to Read DHT Sensor
void readDHTTask(void *pvParameters) {
    for (;;) {
        // Read temperature and humidity from DHT22
        float temperature = dht.readTemperature();
        float hum = dht.readHumidity();

        // Check if any reads failed and handle them accordingly
        if (isnan(temperature) || isnan(hum)) {
            Serial.println("Failed to read from DHT sensor!");
        } else {
            // Round the temperature and humidity values
            int roundedTemperature = round(temperature);
            int roundedHum = round(hum);

            // Convert the rounded values to strings
            String temperatureStr = String(roundedTemperature) + "°C";
            String humStr = String(roundedHum) + "%";

            // Update the bars with sensor values (LVGL)
            lv_bar_set_value(ui_Bar5, roundedTemperature, LV_ANIM_OFF);
            lv_bar_set_value(ui_Bar3, roundedHum, LV_ANIM_OFF);

            // Update the labels with the sensor strings (LVGL)
            lv_label_set_text(ui_Label14, temperatureStr.c_str());
            lv_label_set_text(ui_Label4, humStr.c_str());
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

// Create Task to Read Ultrasonic Sensor
void readUltrasonicTask(void *pvParameters) {
    for (;;) {
        // Clear the trigger pin
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);

        // Set the trigger pin HIGH for 10 microseconds
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);

        // Read the echo pin, measure how long it stays HIGH
        unsigned long duration = pulseIn(ECHO_PIN, HIGH);

        // Calculate the distance (speed of sound is 34300 cm/s)
        float distance = duration * 0.034 / 2;

        // Round the distance value
        int roundedDistance = round(distance);

        // Convert the rounded value to a string
        String distanceStr = String(roundedDistance);

        // Update the label with the sensor string (LVGL)
        lv_label_set_text(ui_Label9, distanceStr.c_str());

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

// Create Task for GUI (Basically it's part of the LVGL library created by Squareline Studio)
void guiTask(void *pvParameters) {
    for (;;) {
        lv_timer_handler(); // Let the GUI do its work
        vTaskDelay(5 / portTICK_PERIOD_MS); // Short delay to avoid a tight loop
    }
}

void setup()
{
    Serial.begin(115200);

    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println(LVGL_Arduino);
    Serial.println("I am LVGL_Arduino");

    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb(my_print); // Register print function for debugging
#endif

    // Initialize the DHT sensor
    dht.begin();

    // Initialize the HC-SR04 sensor
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Initialize the touchscreen
    touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
    touchscreen.begin(touchscreenSPI);
    touchscreen.setRotation(0); // Adjust this based on your touchscreen orientation

    tft.init();          // TFT init
    tft.setRotation(3);   // Landscape orientation, flipped

    lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 10);

    // Initialize the display
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    // Initialize the input device driver
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    ui_init();

    Serial.println("Setup done");

    // Create FreeRTOS tasks
    xTaskCreate(readDHTTask, "Read DHT Task", 2048, NULL, 1, NULL);
    xTaskCreate(readUltrasonicTask, "Read Ultrasonic Task", 2048, NULL, 1, NULL);
    xTaskCreate(guiTask, "GUI Task", 4096, NULL, 1, NULL);
}

void loop() {
    // Do nothing, everything is handled by FreeRTOS tasks
}
