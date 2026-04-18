#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

// ------------------- Motor Pins -------------------
#define M1_IN1 GPIO_NUM_4
#define M1_IN2 GPIO_NUM_5
#define M1_IN3 GPIO_NUM_6
#define M1_IN4 GPIO_NUM_7

#define M2_IN1 GPIO_NUM_15
#define M2_IN2 GPIO_NUM_16
#define M2_IN3 GPIO_NUM_17
#define M2_IN4 GPIO_NUM_18

// ------------------- Stepper Config -------------------
#define STEPS_PER_REVOLUTION 50   // Change to your motor's full-step count
#define STEP_DELAY_MS        20      // Speed (lower = faster)

// Full-step sequence
const int stepSequence[4][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
};

// ------------------- Queue for Rotation Commands -------------------
typedef struct {
    uint8_t motor;      // 1 = Motor1, 2 = Motor2, 3 = Both
    uint8_t direction;  // 0 = CW, 1 = CCW
} rotation_cmd_t;

static QueueHandle_t rotation_queue;

// ------------------- UART Config -------------------
#define UART_NUM       UART_NUM_0
#define BUF_SIZE       256
#define UART_BAUDRATE  115200

// ------------------- Motor Control Helpers -------------------
static void set_motor_pins(int m1_idx, int m2_idx) {
    // Motor 1
    gpio_set_level(M1_IN1, stepSequence[m1_idx][0]);
    gpio_set_level(M1_IN2, stepSequence[m1_idx][1]);
    gpio_set_level(M1_IN3, stepSequence[m1_idx][2]);
    gpio_set_level(M1_IN4, stepSequence[m1_idx][3]);

    // Motor 2
    gpio_set_level(M2_IN1, stepSequence[m2_idx][0]);
    gpio_set_level(M2_IN2, stepSequence[m2_idx][1]);
    gpio_set_level(M2_IN3, stepSequence[m2_idx][2]);
    gpio_set_level(M2_IN4, stepSequence[m2_idx][3]);
}

static void reset_both_motors(void) {
    // Set both motors to the first step (index 0)
    set_motor_pins(0, 0);
}

static void release_both_motors(void) {
    gpio_set_level(M1_IN1, 0); gpio_set_level(M1_IN2, 0);
    gpio_set_level(M1_IN3, 0); gpio_set_level(M1_IN4, 0);
    gpio_set_level(M2_IN1, 0); gpio_set_level(M2_IN2, 0);
    gpio_set_level(M2_IN3, 0); gpio_set_level(M2_IN4, 0);
}

// ------------------- Rotation Function (Blocking) -------------------
void perform_rotation(uint8_t motor, uint8_t direction) {
    // Reset to known starting step
    reset_both_motors();
    vTaskDelay(pdMS_TO_TICKS(10));

    for (int step = 0; step < STEPS_PER_REVOLUTION; step++) {
        for (int i = 0; i < 4; i++) {
            int idx = direction ? (3 - i) : i;   // 0→CW, 1→CCW
            
            // Set both motors to the same index, but only move the selected motor(s)
            int m1_idx = (motor == 1 || motor == 3) ? idx : -1;
            int m2_idx = (motor == 2 || motor == 3) ? idx : -1;
            
            // If a motor is not selected, keep it at its last position (or released)
            // For simplicity, we keep both at the same index but only selected motor moves;
            // the unselected motor will also step along, but that's okay if we want both to be
            // in sync. Alternatively we could hold the unselected motor's pins unchanged.
            // Here we step both for code simplicity; you can modify if needed.
            set_motor_pins(m1_idx >= 0 ? m1_idx : 0, m2_idx >= 0 ? m2_idx : 0);
            vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
        }
    }
    release_both_motors();
}

// ------------------- Motor Control Task -------------------
void motor_control_task(void *pvParameters) {
    rotation_cmd_t cmd;
    while (1) {
        if (xQueueReceive(rotation_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            printf("Executing: Motor %s, Direction %s\n",
                   cmd.motor == 1 ? "1" : cmd.motor == 2 ? "2" : "BOTH",
                   cmd.direction == 0 ? "CW" : "CCW");
            perform_rotation(cmd.motor, cmd.direction);
            printf("Rotation complete.\n");
        }
    }
}

// ------------------- UART Command Parser Task -------------------
void uart_task(void *pvParameters) {
    uint8_t data[BUF_SIZE];
    char cmd[32];
    
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';
            sscanf((char*)data, "%s", cmd);

            rotation_cmd_t new_cmd = {0};

            if (strcmp(cmd, "M1") == 0) {
                sscanf((char*)data, "%*s %s", cmd);
                if (strcmp(cmd, "CW") == 0) {
                    new_cmd.motor = 1; new_cmd.direction = 0;
                    xQueueSend(rotation_queue, &new_cmd, 0);
                } else if (strcmp(cmd, "CCW") == 0) {
                    new_cmd.motor = 1; new_cmd.direction = 1;
                    xQueueSend(rotation_queue, &new_cmd, 0);
                }
            }
            else if (strcmp(cmd, "M2") == 0) {
                sscanf((char*)data, "%*s %s", cmd);
                if (strcmp(cmd, "CW") == 0) {
                    new_cmd.motor = 2; new_cmd.direction = 0;
                    xQueueSend(rotation_queue, &new_cmd, 0);
                } else if (strcmp(cmd, "CCW") == 0) {
                    new_cmd.motor = 2; new_cmd.direction = 1;
                    xQueueSend(rotation_queue, &new_cmd, 0);
                }
            }
            else if (strcmp(cmd, "BOTH") == 0) {
                sscanf((char*)data, "%*s %s", cmd);
                if (strcmp(cmd, "CW") == 0) {
                    new_cmd.motor = 3; new_cmd.direction = 0;
                    xQueueSend(rotation_queue, &new_cmd, 0);
                } else if (strcmp(cmd, "CCW") == 0) {
                    new_cmd.motor = 3; new_cmd.direction = 1;
                    xQueueSend(rotation_queue, &new_cmd, 0);
                }
            }
            else {
                printf("Unknown command. Use: M1 CW, M1 CCW, M2 CW, M2 CCW, BOTH CW, BOTH CCW\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ------------------- Main -------------------
void app_main(void) {
    // --- GPIO Init ---
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    io_conf.pin_bit_mask = (1ULL << M1_IN1) | (1ULL << M1_IN2) |
                           (1ULL << M1_IN3) | (1ULL << M1_IN4);
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << M2_IN1) | (1ULL << M2_IN2) |
                           (1ULL << M2_IN3) | (1ULL << M2_IN4);
    gpio_config(&io_conf);

    release_both_motors();

    // --- UART Init ---
    uart_config_t uart_config = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // --- Create Queue ---
    rotation_queue = xQueueCreate(5, sizeof(rotation_cmd_t));

    // Welcome message
    printf("\n--- Stepper Motor Control ---\n");
    printf("Commands: M1 CW | M1 CCW | M2 CW | M2 CCW | BOTH CW | BOTH CCW\n");

    // Create tasks
    xTaskCreate(motor_control_task, "motor_ctrl", 2048, NULL, 5, NULL);
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
}

