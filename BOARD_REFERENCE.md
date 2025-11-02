# STM32 DISCO_L475_IOT1 Board Reference Guide

Complete reference for using the STM32 B-L475E-IOT01A Discovery Kit with Zephyr RTOS.

## Table of Contents

- [Board Overview](#board-overview)
- [Pin Configuration](#pin-configuration)
- [GPIO Operations](#gpio-operations)
- [Analog Operations (ADC)](#analog-operations-adc)
- [PWM (Pulse Width Modulation)](#pwm-pulse-width-modulation)
- [RTOS Features](#rtos-features)
  - [Tasks (Threads)](#tasks-threads)
  - [Task Delays](#task-delays)
  - [Semaphores](#semaphores)
  - [Mutexes](#mutexes)
  - [Message Queues](#message-queues)
  - [Timers](#timers)
- [Communication Protocols](#communication-protocols)
  - [UART](#uart)
  - [I2C](#i2c)
  - [SPI](#spi)
- [Interrupts (IRQ)](#interrupts-irq)
- [Timers and Counters](#timers-and-counters)
- [Real-Time Clock (RTC)](#real-time-clock-rtc)
- [Onboard Sensors](#onboard-sensors)
- [Complete Examples](#complete-examples)

---

## Board Overview

**Board**: STM32 B-L475E-IOT01A Discovery Kit for IoT Node
**MCU**: STM32L475VGT6 (Arm Cortex-M4, 80 MHz, 1MB Flash, 128KB RAM)
**Zephyr Board Name**: `disco_l475_iot1`

### Key Features
- Ultra-low-power STM32L4 Series
- Rich peripheral set
- Multiple onboard sensors
- Bluetooth Low Energy
- Wi-Fi module
- NFC tag
- USB OTG

---

## Pin Configuration

### Arduino-Compatible Headers

Based on your [custom_pins.overlay](custom_pins.overlay):

| Pin | GPIO Port/Pin | Functions | Description |
|-----|---------------|-----------|-------------|
| **A0** | PC5 | ADC | Analog input |
| **A1** | PC4 | ADC | Analog input |
| **A2** | PC3 | ADC | Analog input |
| **A3** | PC2 | ADC | Analog input |
| **A4** | PC1 | ADC / I2C3_SDA | Analog or I2C data |
| **A5** | PC0 | ADC / I2C3_SCL | Analog or I2C clock |
| **D0** | PA1 | UART4_RX | Serial receive |
| **D1** | PA0 | UART4_TX | Serial transmit |
| **D2** | PD14 | EXTI14 | External interrupt |
| **D3** | PB0 | TIM3_CH3 / EXTI0 | PWM or interrupt |
| **D4** | PA3 | GPIO | General purpose |
| **D5** | PB4 | TIM3_CH1 | PWM capable |
| **D6** | PB1 | TIM3_CH4 | PWM capable |
| **D7** | PA4 | GPIO | General purpose |
| **D8** | PB2 | GPIO | General purpose |
| **D9** | PA15 | TIM2_CH1 | PWM capable |
| **D10** | PA2 | TIM2_CH3 | PWM capable |
| **D11** | PA7 | SPI1_MOSI | SPI data out |
| **D12** | PA6 | SPI1_MISO | SPI data in |
| **D13** | PA5 | SPI1_SCK / LED1 | SPI clock / LED |
| **D14** | PB9 | I2C1_SDA | I2C data |
| **D15** | PB8 | I2C1_SCL | I2C clock |

### Onboard Components

| Component | GPIO | Description |
|-----------|------|-------------|
| LED1 (Green) | PA5 | User LED |
| LED2 (Green) | PB14 | User LED |
| Button (Blue) | PC13 | User button |

---

## GPIO Operations

### Configuration Macros

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
```

### Defining a GPIO Pin

#### Method 1: Using Device Tree Aliases

In your overlay file:
```dts
/ {
    aliases {
        myled = &d13;
    };
};
```

In code:
```c
#define LED_NODE DT_ALIAS(myled)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
```

#### Method 2: Direct Definition

```c
// Using existing pin definitions
#define D4_NODE DT_ALIAS(d4)
static const struct gpio_dt_spec my_pin = GPIO_DT_SPEC_GET(D4_NODE, gpios);
```

### Digital Write (Output)

```c
// Configure as output
if (!gpio_is_ready_dt(&led)) {
    printk("Error: LED device not ready\n");
    return -1;
}

int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
if (ret < 0) {
    printk("Error: failed to configure LED pin\n");
    return ret;
}

// Set pin HIGH
gpio_pin_set_dt(&led, 1);

// Set pin LOW
gpio_pin_set_dt(&led, 0);

// Toggle pin
gpio_pin_toggle_dt(&led);
```

### Digital Read (Input)

```c
// Configure as input with pull-up
const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

int ret = gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
if (ret < 0) {
    return ret;
}

// Read pin state
int value = gpio_pin_get_dt(&button);
if (value < 0) {
    printk("Error reading pin\n");
} else {
    printk("Pin state: %d\n", value);
}
```

### Complete GPIO Example

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define LED_NODE DT_ALIAS(d13)
#define BUTTON_NODE DT_ALIAS(sw0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);

int main(void)
{
    // Configure LED
    if (!gpio_is_ready_dt(&led)) {
        return -1;
    }
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    // Configure button
    if (!gpio_is_ready_dt(&button)) {
        return -1;
    }
    gpio_pin_configure_dt(&button, GPIO_INPUT);

    while (1) {
        // Read button and control LED
        int button_state = gpio_pin_get_dt(&button);
        gpio_pin_set_dt(&led, button_state);
        k_msleep(10);
    }

    return 0;
}
```

---

## Analog Operations (ADC)

### Configuration

Add to [prj.conf](prj.conf):
```conf
CONFIG_ADC=y
```

### ADC Code Example

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>

#define ADC_NODE DT_NODELABEL(adc1)
#define ADC_RESOLUTION 12
#define ADC_GAIN ADC_GAIN_1
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT
#define ADC_CHANNEL_ID 0  // For A0 (PC5) - Channel IN14

static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

static const struct adc_channel_cfg channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_ID,
    .differential = 0
};

static int16_t sample_buffer[1];

static const struct adc_sequence sequence = {
    .channels = BIT(ADC_CHANNEL_ID),
    .buffer = sample_buffer,
    .buffer_size = sizeof(sample_buffer),
    .resolution = ADC_RESOLUTION,
};

int adc_init(void)
{
    if (!device_is_ready(adc_dev)) {
        printk("ADC device not ready\n");
        return -1;
    }

    int err = adc_channel_setup(adc_dev, &channel_cfg);
    if (err < 0) {
        printk("ADC channel setup failed: %d\n", err);
        return err;
    }

    return 0;
}

int adc_read(void)
{
    int err = adc_read(adc_dev, &sequence);
    if (err < 0) {
        printk("ADC read failed: %d\n", err);
        return err;
    }

    // Convert to millivolts
    int32_t val_mv = sample_buffer[0];
    adc_raw_to_millivolts(adc_vref_internal, ADC_GAIN, ADC_RESOLUTION, &val_mv);

    printk("ADC reading: %d mV\n", val_mv);
    return val_mv;
}

int main(void)
{
    if (adc_init() < 0) {
        return -1;
    }

    while (1) {
        adc_read();
        k_msleep(1000);
    }

    return 0;
}
```

### ADC Channel Mapping (STM32L475)

| Pin | ADC Channel | Note |
|-----|-------------|------|
| A0 (PC5) | IN14 | ADC1_IN14 |
| A1 (PC4) | IN13 | ADC1_IN13 |
| A2 (PC3) | IN4 | ADC1_IN4 |
| A3 (PC2) | IN3 | ADC1_IN3 |
| A4 (PC1) | IN2 | ADC1_IN2 |
| A5 (PC0) | IN1 | ADC1_IN1 |

---

## PWM (Pulse Width Modulation)

### Configuration

Add to [prj.conf](prj.conf):
```conf
CONFIG_PWM=y
```

### Device Tree Overlay

Add PWM configuration to your overlay:
```dts
&timers3 {
    status = "okay";

    pwm3: pwm {
        status = "okay";
        pinctrl-0 = <&tim3_ch1_pb4>;
        pinctrl-names = "default";
    };
};
```

### PWM Example

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>

#define PWM_NODE DT_NODELABEL(pwm3)
#define PWM_CHANNEL 1  // TIM3_CH1

static const struct device *pwm_dev = DEVICE_DT_GET(PWM_NODE);

int pwm_init(void)
{
    if (!device_is_ready(pwm_dev)) {
        printk("PWM device not ready\n");
        return -1;
    }
    return 0;
}

void set_pwm_duty_cycle(uint32_t period_us, uint32_t pulse_us)
{
    pwm_set(pwm_dev, PWM_CHANNEL, PWM_USEC(period_us), PWM_USEC(pulse_us), 0);
}

int main(void)
{
    if (pwm_init() < 0) {
        return -1;
    }

    uint32_t period = 20000;  // 20ms period (50Hz)
    uint32_t duty = 0;

    while (1) {
        // Fade in
        for (duty = 0; duty <= period; duty += 100) {
            set_pwm_duty_cycle(period, duty);
            k_msleep(10);
        }

        // Fade out
        for (duty = period; duty > 0; duty -= 100) {
            set_pwm_duty_cycle(period, duty);
            k_msleep(10);
        }
    }

    return 0;
}
```

### Available PWM Pins

| Pin | Timer | Channel |
|-----|-------|---------|
| D3 (PB0) | TIM3 | CH3 |
| D5 (PB4) | TIM3 | CH1 |
| D6 (PB1) | TIM3 | CH4 |
| D9 (PA15) | TIM2 | CH1 |
| D10 (PA2) | TIM2 | CH3 |

---

## RTOS Features

### Tasks (Threads)

#### Creating a Thread

```c
#include <zephyr/kernel.h>

#define STACK_SIZE 1024
#define THREAD_PRIORITY 7

// Define thread stack
K_THREAD_STACK_DEFINE(my_thread_stack, STACK_SIZE);

// Thread data structure
static struct k_thread my_thread_data;

// Thread entry function
void my_thread_entry(void *arg1, void *arg2, void *arg3)
{
    int count = 0;

    while (1) {
        printk("Thread running: %d\n", count++);
        k_msleep(1000);
    }
}

int main(void)
{
    // Create thread
    k_tid_t my_tid = k_thread_create(
        &my_thread_data,              // Thread data structure
        my_thread_stack,              // Stack
        K_THREAD_STACK_SIZEOF(my_thread_stack),  // Stack size
        my_thread_entry,              // Entry function
        NULL, NULL, NULL,             // Arguments
        THREAD_PRIORITY,              // Priority (lower = higher priority)
        0,                            // Options
        K_NO_WAIT                     // Start immediately
    );

    // Optional: name the thread
    k_thread_name_set(my_tid, "my_thread");

    return 0;
}
```

#### Thread Control

```c
// Suspend thread
k_thread_suspend(my_tid);

// Resume thread
k_thread_resume(my_tid);

// Abort thread
k_thread_abort(my_tid);

// Join thread (wait for completion)
k_thread_join(my_tid, K_FOREVER);

// Set thread priority
k_thread_priority_set(my_tid, NEW_PRIORITY);
```

### Task Delays

```c
// Delay in milliseconds
k_msleep(1000);  // Sleep for 1 second

// Delay in microseconds
k_usleep(500);   // Sleep for 500 microseconds

// Delay in system ticks
k_sleep(K_MSEC(100));  // Sleep for 100ms

// Delay with timeout
k_sleep(K_SECONDS(1));  // Sleep for 1 second

// Busy wait (blocks CPU)
k_busy_wait(1000);  // Wait 1000 microseconds
```

### Semaphores

#### Binary Semaphore

```c
#include <zephyr/kernel.h>

// Define semaphore
K_SEM_DEFINE(my_sem, 0, 1);  // Initial count=0, max=1

// Producer thread
void producer_thread(void *p1, void *p2, void *p3)
{
    while (1) {
        printk("Producing...\n");
        k_msleep(1000);
        k_sem_give(&my_sem);  // Signal semaphore
    }
}

// Consumer thread
void consumer_thread(void *p1, void *p2, void *p3)
{
    while (1) {
        // Wait for semaphore (block until available)
        if (k_sem_take(&my_sem, K_FOREVER) == 0) {
            printk("Consuming...\n");
        }
    }
}
```

#### Counting Semaphore

```c
// Define counting semaphore (max count = 10)
K_SEM_DEFINE(count_sem, 0, 10);

void task1(void *p1, void *p2, void *p3)
{
    while (1) {
        k_sem_give(&count_sem);  // Increment
        k_msleep(100);
    }
}

void task2(void *p1, void *p2, void *p3)
{
    while (1) {
        // Wait with timeout
        if (k_sem_take(&count_sem, K_MSEC(500)) == 0) {
            printk("Got semaphore\n");
        } else {
            printk("Timeout\n");
        }
    }
}
```

### Mutexes

```c
#include <zephyr/kernel.h>

// Define mutex
K_MUTEX_DEFINE(my_mutex);

// Shared resource
static int shared_counter = 0;

void thread_a(void *p1, void *p2, void *p3)
{
    while (1) {
        // Lock mutex
        k_mutex_lock(&my_mutex, K_FOREVER);

        // Critical section
        shared_counter++;
        printk("Thread A: counter = %d\n", shared_counter);

        // Unlock mutex
        k_mutex_unlock(&my_mutex);

        k_msleep(100);
    }
}

void thread_b(void *p1, void *p2, void *p3)
{
    while (1) {
        k_mutex_lock(&my_mutex, K_FOREVER);

        shared_counter--;
        printk("Thread B: counter = %d\n", shared_counter);

        k_mutex_unlock(&my_mutex);

        k_msleep(150);
    }
}
```

### Message Queues

```c
#include <zephyr/kernel.h>

// Define message structure
typedef struct {
    int sensor_id;
    float value;
} sensor_data_t;

// Define message queue
#define QUEUE_SIZE 10
K_MSGQ_DEFINE(sensor_queue, sizeof(sensor_data_t), QUEUE_SIZE, 4);

// Producer thread
void sensor_reader(void *p1, void *p2, void *p3)
{
    sensor_data_t data;

    while (1) {
        data.sensor_id = 1;
        data.value = 25.5;  // Example temperature

        // Send to queue (non-blocking)
        if (k_msgq_put(&sensor_queue, &data, K_NO_WAIT) != 0) {
            printk("Queue full!\n");
        }

        k_msleep(500);
    }
}

// Consumer thread
void data_processor(void *p1, void *p2, void *p3)
{
    sensor_data_t data;

    while (1) {
        // Receive from queue (blocking)
        if (k_msgq_get(&sensor_queue, &data, K_FOREVER) == 0) {
            printk("Sensor %d: %.2f\n", data.sensor_id, data.value);
        }
    }
}

// Queue operations
void queue_operations(void)
{
    sensor_data_t data;

    // Peek without removing
    k_msgq_peek(&sensor_queue, &data);

    // Purge queue (clear all messages)
    k_msgq_purge(&sensor_queue);

    // Get number of used entries
    uint32_t used = k_msgq_num_used_get(&sensor_queue);

    // Get free space
    uint32_t free = k_msgq_num_free_get(&sensor_queue);
}
```

### Timers

#### Software Timers

```c
#include <zephyr/kernel.h>

// Timer expiry function
void timer_expired(struct k_timer *timer_id)
{
    printk("Timer expired!\n");
}

// Timer stop function (optional)
void timer_stopped(struct k_timer *timer_id)
{
    printk("Timer stopped\n");
}

// Define timer
K_TIMER_DEFINE(my_timer, timer_expired, timer_stopped);

int main(void)
{
    // One-shot timer (expires once after 1 second)
    k_timer_start(&my_timer, K_SECONDS(1), K_NO_WAIT);

    // Periodic timer (expires every 500ms)
    k_timer_start(&my_timer, K_MSEC(500), K_MSEC(500));

    // Stop timer
    k_timer_stop(&my_timer);

    // Get remaining time
    uint32_t remaining = k_timer_remaining_get(&my_timer);

    // Get status
    uint32_t status = k_timer_status_get(&my_timer);

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
```

#### Using Timers with User Data

```c
typedef struct {
    int counter;
    const char *name;
} timer_data_t;

void my_timer_handler(struct k_timer *timer)
{
    timer_data_t *data = k_timer_user_data_get(timer);
    data->counter++;
    printk("%s: %d\n", data->name, data->counter);
}

K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

int main(void)
{
    static timer_data_t my_data = {
        .counter = 0,
        .name = "MyTimer"
    };

    k_timer_user_data_set(&my_timer, &my_data);
    k_timer_start(&my_timer, K_SECONDS(1), K_SECONDS(1));

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
```

---

## Communication Protocols

### UART

#### Configuration

Add to [prj.conf](prj.conf):
```conf
CONFIG_SERIAL=y
CONFIG_UART_INTERRUPT_DRIVEN=y
```

#### UART Example

```c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#define UART_DEVICE_NODE DT_NODELABEL(usart1)

static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

// Callback for received data
static void uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_dev)) {
        return;
    }

    if (uart_irq_rx_ready(uart_dev)) {
        uart_fifo_read(uart_dev, &c, 1);
        printk("Received: %c\n", c);

        // Echo back
        uart_poll_out(uart_dev, c);
    }
}

int uart_init(void)
{
    if (!device_is_ready(uart_dev)) {
        printk("UART device not ready\n");
        return -1;
    }

    // Configure interrupt
    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    uart_irq_rx_enable(uart_dev);

    return 0;
}

void uart_send_string(const char *str)
{
    int len = strlen(str);
    for (int i = 0; i < len; i++) {
        uart_poll_out(uart_dev, str[i]);
    }
}

int main(void)
{
    if (uart_init() < 0) {
        return -1;
    }

    uart_send_string("Hello UART!\n");

    while (1) {
        k_msleep(1000);
    }

    return 0;
}
```

### I2C

#### Configuration

Add to [prj.conf](prj.conf):
```conf
CONFIG_I2C=y
```

#### I2C Example

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

#define I2C_NODE DT_NODELABEL(i2c1)
#define I2C_SLAVE_ADDR 0x3C  // Example address

static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

int i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_write(i2c_dev, buf, 2, addr);
}

int i2c_read_byte(uint8_t addr, uint8_t reg, uint8_t *value)
{
    return i2c_write_read(i2c_dev, addr, &reg, 1, value, 1);
}

int i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    return i2c_write_read(i2c_dev, addr, &reg, 1, buf, len);
}

int main(void)
{
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return -1;
    }

    // Write example
    i2c_write_byte(I2C_SLAVE_ADDR, 0x00, 0xAF);

    // Read example
    uint8_t data;
    i2c_read_byte(I2C_SLAVE_ADDR, 0x00, &data);
    printk("Read: 0x%02X\n", data);

    return 0;
}
```

### SPI

#### Configuration

Add to [prj.conf](prj.conf):
```conf
CONFIG_SPI=y
```

#### SPI Example

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#define SPI_NODE DT_NODELABEL(spi1)

static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

// SPI configuration
static struct spi_config spi_cfg = {
    .frequency = 1000000,  // 1 MHz
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
                 SPI_MODE_CPOL | SPI_MODE_CPHA,
    .slave = 0,
    .cs = NULL,
};

int spi_write_read(uint8_t *tx_buf, uint8_t *rx_buf, size_t len)
{
    struct spi_buf tx_spi_buf = {
        .buf = tx_buf,
        .len = len
    };
    struct spi_buf_set tx_spi_buf_set = {
        .buffers = &tx_spi_buf,
        .count = 1
    };

    struct spi_buf rx_spi_buf = {
        .buf = rx_buf,
        .len = len
    };
    struct spi_buf_set rx_spi_buf_set = {
        .buffers = &rx_spi_buf,
        .count = 1
    };

    return spi_transceive(spi_dev, &spi_cfg, &tx_spi_buf_set, &rx_spi_buf_set);
}

int main(void)
{
    if (!device_is_ready(spi_dev)) {
        printk("SPI device not ready\n");
        return -1;
    }

    uint8_t tx_data[4] = {0x01, 0x02, 0x03, 0x04};
    uint8_t rx_data[4] = {0};

    spi_write_read(tx_data, rx_data, 4);

    printk("Received: %02X %02X %02X %02X\n",
           rx_data[0], rx_data[1], rx_data[2], rx_data[3]);

    return 0;
}
```

---

## Interrupts (IRQ)

### GPIO Interrupt Example

See your working example in [src/main.c:8-27](src/main.c#L8-L27):

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define BUTTON_NODE DT_ALIAS(sw0)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static struct gpio_callback button_cb_data;

// Interrupt callback
void button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("Button pressed! Pin mask: 0x%x\n", pins);
    // Handle interrupt here
}

int main(void)
{
    // Configure pin as input
    gpio_pin_configure_dt(&button, GPIO_INPUT);

    // Configure interrupt (trigger on rising edge)
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_RISING);

    // Initialize and add callback
    gpio_init_callback(&button_cb_data, button_callback, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
```

### Interrupt Types

```c
// Edge triggered
GPIO_INT_EDGE_RISING      // Trigger on rising edge (LOW to HIGH)
GPIO_INT_EDGE_FALLING     // Trigger on falling edge (HIGH to LOW)
GPIO_INT_EDGE_BOTH        // Trigger on both edges

// Level triggered
GPIO_INT_LEVEL_LOW        // Trigger while LOW
GPIO_INT_LEVEL_HIGH       // Trigger while HIGH

// Disable interrupt
GPIO_INT_DISABLE
```

### Advanced Interrupt Handling

```c
// Disable interrupts temporarily
gpio_pin_interrupt_configure_dt(&button, GPIO_INT_DISABLE);

// Re-enable interrupts
gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_RISING);

// Remove callback
gpio_remove_callback(button.port, &button_cb_data);
```

---

## Timers and Counters

### Hardware Timer (Counter)

#### Configuration

Add to [prj.conf](prj.conf):
```conf
CONFIG_COUNTER=y
```

#### Counter Example

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>

#define TIMER_NODE DT_NODELABEL(timers2)

static const struct device *counter_dev = DEVICE_DT_GET(TIMER_NODE);

struct counter_alarm_cfg alarm_cfg;

void alarm_callback(const struct device *dev, uint8_t chan_id, uint32_t ticks, void *user_data)
{
    printk("Alarm triggered! Ticks: %u\n", ticks);
}

int counter_init(void)
{
    if (!device_is_ready(counter_dev)) {
        printk("Counter device not ready\n");
        return -1;
    }

    // Start counter
    counter_start(counter_dev);

    return 0;
}

void set_alarm(uint32_t delay_ms)
{
    uint32_t ticks;
    counter_get_value(counter_dev, &ticks);

    alarm_cfg.flags = 0;
    alarm_cfg.ticks = counter_us_to_ticks(counter_dev, delay_ms * 1000);
    alarm_cfg.callback = alarm_callback;
    alarm_cfg.user_data = NULL;

    counter_set_channel_alarm(counter_dev, 0, &alarm_cfg);
}

int main(void)
{
    counter_init();
    set_alarm(1000);  // Alarm in 1 second

    while (1) {
        k_msleep(100);
    }

    return 0;
}
```

### Measuring Time with Counters

```c
uint32_t start_ticks, end_ticks;
uint32_t elapsed_us;

// Get start time
counter_get_value(counter_dev, &start_ticks);

// Do something...
k_busy_wait(1000);

// Get end time
counter_get_value(counter_dev, &end_ticks);

// Calculate elapsed time
elapsed_us = counter_ticks_to_us(counter_dev, end_ticks - start_ticks);
printk("Elapsed: %u us\n", elapsed_us);
```

---

## Real-Time Clock (RTC)

### Configuration

Add to [prj.conf](prj.conf):
```conf
CONFIG_RTC=y
```

### RTC Example

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/rtc.h>
#include <time.h>

#define RTC_NODE DT_NODELABEL(rtc)

static const struct device *rtc_dev = DEVICE_DT_GET(RTC_NODE);

int rtc_init(void)
{
    if (!device_is_ready(rtc_dev)) {
        printk("RTC device not ready\n");
        return -1;
    }
    return 0;
}

void rtc_set_time(int year, int month, int day, int hour, int min, int sec)
{
    struct rtc_time time = {
        .tm_year = year - 1900,
        .tm_mon = month - 1,
        .tm_mday = day,
        .tm_hour = hour,
        .tm_min = min,
        .tm_sec = sec,
    };

    rtc_set_time(rtc_dev, &time);
}

void rtc_get_time(void)
{
    struct rtc_time time;

    rtc_get_time(rtc_dev, &time);

    printk("Time: %04d-%02d-%02d %02d:%02d:%02d\n",
           time.tm_year + 1900,
           time.tm_mon + 1,
           time.tm_mday,
           time.tm_hour,
           time.tm_min,
           time.tm_sec);
}

// Set alarm
void rtc_set_alarm(int hour, int min, int sec)
{
    struct rtc_time alarm_time = {
        .tm_hour = hour,
        .tm_min = min,
        .tm_sec = sec,
    };

    uint16_t mask = RTC_ALARM_TIME_MASK_HOUR |
                    RTC_ALARM_TIME_MASK_MINUTE |
                    RTC_ALARM_TIME_MASK_SECOND;

    rtc_alarm_set_time(rtc_dev, 0, mask, &alarm_time);
}

int main(void)
{
    rtc_init();

    // Set time: 2025-01-15 10:30:00
    rtc_set_time(2025, 1, 15, 10, 30, 0);

    while (1) {
        rtc_get_time();
        k_msleep(1000);
    }

    return 0;
}
```

---

## Onboard Sensors

The DISCO_L475_IOT1 board includes several onboard sensors:

### 1. Temperature and Humidity (HTS221)

#### Configuration

Add to [prj.conf](prj.conf):
```conf
CONFIG_I2C=y
CONFIG_SENSOR=y
CONFIG_HTS221=y
```

#### Code Example

```c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#define HTS221_NODE DT_NODELABEL(hts221)

static const struct device *hts221 = DEVICE_DT_GET(HTS221_NODE);

int main(void)
{
    struct sensor_value temp, hum;

    if (!device_is_ready(hts221)) {
        printk("HTS221 not ready\n");
        return -1;
    }

    while (1) {
        // Fetch sensor data
        sensor_sample_fetch(hts221);

        // Get temperature
        sensor_channel_get(hts221, SENSOR_CHAN_AMBIENT_TEMP, &temp);

        // Get humidity
        sensor_channel_get(hts221, SENSOR_CHAN_HUMIDITY, &hum);

        printk("Temperature: %d.%06d C\n", temp.val1, temp.val2);
        printk("Humidity: %d.%06d %%\n", hum.val1, hum.val2);

        k_msleep(2000);
    }

    return 0;
}
```

### 2. Pressure (LPS22HB)

#### Configuration

```conf
CONFIG_LPS22HB=y
```

#### Code Example

```c
#include <zephyr/drivers/sensor.h>

#define LPS22HB_NODE DT_NODELABEL(lps22hb_press)

static const struct device *lps22hb = DEVICE_DT_GET(LPS22HB_NODE);

void read_pressure(void)
{
    struct sensor_value pressure, temp;

    if (!device_is_ready(lps22hb)) {
        return;
    }

    sensor_sample_fetch(lps22hb);
    sensor_channel_get(lps22hb, SENSOR_CHAN_PRESS, &pressure);
    sensor_channel_get(lps22hb, SENSOR_CHAN_AMBIENT_TEMP, &temp);

    printk("Pressure: %d.%06d kPa\n", pressure.val1, pressure.val2);
    printk("Temperature: %d.%06d C\n", temp.val1, temp.val2);
}
```

### 3. Accelerometer and Gyroscope (LSM6DSL)

#### Configuration

```conf
CONFIG_LSM6DSL=y
```

#### Code Example

```c
#include <zephyr/drivers/sensor.h>

#define LSM6DSL_NODE DT_NODELABEL(lsm6dsl)

static const struct device *lsm6dsl = DEVICE_DT_GET(LSM6DSL_NODE);

void read_motion(void)
{
    struct sensor_value accel[3], gyro[3];

    if (!device_is_ready(lsm6dsl)) {
        return;
    }

    sensor_sample_fetch(lsm6dsl);

    // Read accelerometer (m/s^2)
    sensor_channel_get(lsm6dsl, SENSOR_CHAN_ACCEL_XYZ, accel);
    printk("Accel: X=%d.%06d Y=%d.%06d Z=%d.%06d m/s^2\n",
           accel[0].val1, accel[0].val2,
           accel[1].val1, accel[1].val2,
           accel[2].val1, accel[2].val2);

    // Read gyroscope (rad/s)
    sensor_channel_get(lsm6dsl, SENSOR_CHAN_GYRO_XYZ, gyro);
    printk("Gyro: X=%d.%06d Y=%d.%06d Z=%d.%06d rad/s\n",
           gyro[0].val1, gyro[0].val2,
           gyro[1].val1, gyro[1].val2,
           gyro[2].val1, gyro[2].val2);
}
```

### 4. Magnetometer (LIS3MDL)

#### Configuration

```conf
CONFIG_LIS3MDL=y
```

#### Code Example

```c
#include <zephyr/drivers/sensor.h>

#define LIS3MDL_NODE DT_NODELABEL(lis3mdl_magn)

static const struct device *lis3mdl = DEVICE_DT_GET(LIS3MDL_NODE);

void read_magnetometer(void)
{
    struct sensor_value magn[3];

    if (!device_is_ready(lis3mdl)) {
        return;
    }

    sensor_sample_fetch(lis3mdl);
    sensor_channel_get(lis3mdl, SENSOR_CHAN_MAGN_XYZ, magn);

    printk("Magnetometer: X=%d.%06d Y=%d.%06d Z=%d.%06d gauss\n",
           magn[0].val1, magn[0].val2,
           magn[1].val1, magn[1].val2,
           magn[2].val1, magn[2].val2);
}
```

### 5. Time-of-Flight Distance Sensor (VL53L0X)

#### Configuration

```conf
CONFIG_VL53L0X=y
```

#### Code Example

```c
#include <zephyr/drivers/sensor.h>

#define VL53L0X_NODE DT_NODELABEL(vl53l0x)

static const struct device *vl53l0x = DEVICE_DT_GET(VL53L0X_NODE);

void read_distance(void)
{
    struct sensor_value distance;

    if (!device_is_ready(vl53l0x)) {
        return;
    }

    sensor_sample_fetch(vl53l0x);
    sensor_channel_get(vl53l0x, SENSOR_CHAN_DISTANCE, &distance);

    printk("Distance: %d.%03d m\n", distance.val1, distance.val2 / 1000);
}
```

---

## Complete Examples

### Example 1: Multi-Sensor Data Logger

```c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>

#define HTS221_NODE DT_NODELABEL(hts221)
#define LPS22HB_NODE DT_NODELABEL(lps22hb_press)
#define LED_NODE DT_ALIAS(led0)

static const struct device *hts221 = DEVICE_DT_GET(HTS221_NODE);
static const struct device *lps22hb = DEVICE_DT_GET(LPS22HB_NODE);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

K_MSGQ_DEFINE(sensor_msgq, sizeof(struct sensor_value) * 3, 10, 4);

void sensor_thread(void *p1, void *p2, void *p3)
{
    struct sensor_value data[3];

    while (1) {
        // Read temperature
        sensor_sample_fetch(hts221);
        sensor_channel_get(hts221, SENSOR_CHAN_AMBIENT_TEMP, &data[0]);

        // Read humidity
        sensor_channel_get(hts221, SENSOR_CHAN_HUMIDITY, &data[1]);

        // Read pressure
        sensor_sample_fetch(lps22hb);
        sensor_channel_get(lps22hb, SENSOR_CHAN_PRESS, &data[2]);

        // Send to queue
        k_msgq_put(&sensor_msgq, &data, K_NO_WAIT);

        // Blink LED
        gpio_pin_toggle_dt(&led);

        k_msleep(1000);
    }
}

void logger_thread(void *p1, void *p2, void *p3)
{
    struct sensor_value data[3];

    while (1) {
        if (k_msgq_get(&sensor_msgq, &data, K_FOREVER) == 0) {
            printk("T: %d.%02d C | H: %d.%02d %% | P: %d.%02d kPa\n",
                   data[0].val1, data[0].val2 / 10000,
                   data[1].val1, data[1].val2 / 10000,
                   data[2].val1, data[2].val2 / 10000);
        }
    }
}

K_THREAD_DEFINE(sensor_tid, 1024, sensor_thread, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(logger_tid, 1024, logger_thread, NULL, NULL, NULL, 7, 0, 0);

int main(void)
{
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
```

### Example 2: Button-Controlled LED with Debouncing

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define LED_NODE DT_ALIAS(led0)
#define BUTTON_NODE DT_ALIAS(sw0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static struct gpio_callback button_cb_data;

K_SEM_DEFINE(button_sem, 0, 1);

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_sem_give(&button_sem);
}

void button_handler_thread(void *p1, void *p2, void *p3)
{
    while (1) {
        k_sem_take(&button_sem, K_FOREVER);

        // Debounce delay
        k_msleep(50);

        // Check if button still pressed
        if (gpio_pin_get_dt(&button) == 1) {
            gpio_pin_toggle_dt(&led);
            printk("LED toggled\n");
        }

        // Wait for button release
        while (gpio_pin_get_dt(&button) == 1) {
            k_msleep(10);
        }
    }
}

K_THREAD_DEFINE(button_tid, 512, button_handler_thread, NULL, NULL, NULL, 7, 0, 0);

int main(void)
{
    // Configure LED
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

    // Configure button with interrupt
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
```

### Example 3: PWM-Controlled RGB LED (External)

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>

// Assuming PWM channels configured in device tree
#define PWM_RED_NODE DT_ALIAS(pwm_red)
#define PWM_GREEN_NODE DT_ALIAS(pwm_green)
#define PWM_BLUE_NODE DT_ALIAS(pwm_blue)

static const struct pwm_dt_spec red = PWM_DT_SPEC_GET(PWM_RED_NODE);
static const struct pwm_dt_spec green = PWM_DT_SPEC_GET(PWM_GREEN_NODE);
static const struct pwm_dt_spec blue = PWM_DT_SPEC_GET(PWM_BLUE_NODE);

#define PWM_PERIOD PWM_MSEC(20)

void set_color(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t r_pulse = (PWM_PERIOD * r) / 255;
    uint32_t g_pulse = (PWM_PERIOD * g) / 255;
    uint32_t b_pulse = (PWM_PERIOD * b) / 255;

    pwm_set_dt(&red, PWM_PERIOD, r_pulse);
    pwm_set_dt(&green, PWM_PERIOD, g_pulse);
    pwm_set_dt(&blue, PWM_PERIOD, b_pulse);
}

int main(void)
{
    // Check devices
    if (!pwm_is_ready_dt(&red) || !pwm_is_ready_dt(&green) || !pwm_is_ready_dt(&blue)) {
        printk("PWM devices not ready\n");
        return -1;
    }

    while (1) {
        // Red
        set_color(255, 0, 0);
        k_msleep(1000);

        // Green
        set_color(0, 255, 0);
        k_msleep(1000);

        // Blue
        set_color(0, 0, 255);
        k_msleep(1000);

        // White
        set_color(255, 255, 255);
        k_msleep(1000);
    }

    return 0;
}
```

### Example 4: UART Echo with Command Processing

```c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

#define UART_DEVICE_NODE DT_NODELABEL(usart1)
#define RX_BUFFER_SIZE 128

static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static char rx_buffer[RX_BUFFER_SIZE];
static uint16_t rx_index = 0;

K_MSGQ_DEFINE(uart_msgq, RX_BUFFER_SIZE, 10, 4);

static void uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_dev)) {
        return;
    }

    if (uart_irq_rx_ready(uart_dev)) {
        while (uart_fifo_read(uart_dev, &c, 1) == 1) {
            if (c == '\n' || c == '\r') {
                if (rx_index > 0) {
                    rx_buffer[rx_index] = '\0';
                    k_msgq_put(&uart_msgq, rx_buffer, K_NO_WAIT);
                    rx_index = 0;
                }
            } else if (rx_index < RX_BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = c;
                uart_poll_out(uart_dev, c);  // Echo
            }
        }
    }
}

void command_processor(void *p1, void *p2, void *p3)
{
    char cmd_buffer[RX_BUFFER_SIZE];

    while (1) {
        if (k_msgq_get(&uart_msgq, cmd_buffer, K_FOREVER) == 0) {
            printk("Received command: %s\n", cmd_buffer);

            if (strcmp(cmd_buffer, "help") == 0) {
                printk("Available commands: help, status, reset\n");
            } else if (strcmp(cmd_buffer, "status") == 0) {
                printk("System OK\n");
            } else if (strcmp(cmd_buffer, "reset") == 0) {
                printk("Resetting...\n");
                sys_reboot(SYS_REBOOT_COLD);
            } else {
                printk("Unknown command\n");
            }
        }
    }
}

K_THREAD_DEFINE(cmd_tid, 1024, command_processor, NULL, NULL, NULL, 7, 0, 0);

int main(void)
{
    if (!device_is_ready(uart_dev)) {
        return -1;
    }

    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    uart_irq_rx_enable(uart_dev);

    printk("UART Command Processor Ready\n");
    printk("Type 'help' for available commands\n");

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
```

---

## Quick Reference Tables

### GPIO Configuration Flags

| Flag | Description |
|------|-------------|
| `GPIO_INPUT` | Configure as input |
| `GPIO_OUTPUT` | Configure as output |
| `GPIO_OUTPUT_INIT_LOW` | Output initialized to LOW |
| `GPIO_OUTPUT_INIT_HIGH` | Output initialized to HIGH |
| `GPIO_PULL_UP` | Enable pull-up resistor |
| `GPIO_PULL_DOWN` | Enable pull-down resistor |
| `GPIO_OPEN_DRAIN` | Open-drain output |
| `GPIO_ACTIVE_LOW` | Active low polarity |
| `GPIO_ACTIVE_HIGH` | Active high polarity |

### Thread Priority Levels

Lower numbers = higher priority:
- `0-7`: Cooperative threads (do not preempt)
- `8-15`: Preemptive threads

Common values:
- `0`: Highest priority (critical)
- `7`: High priority
- `10`: Medium priority (default)
- `15`: Low priority (background tasks)

### Common K_ Timeout Values

| Value | Description |
|-------|-------------|
| `K_NO_WAIT` | Return immediately |
| `K_FOREVER` | Wait indefinitely |
| `K_MSEC(ms)` | Wait for milliseconds |
| `K_SECONDS(s)` | Wait for seconds |
| `K_MINUTES(m)` | Wait for minutes |
| `K_HOURS(h)` | Wait for hours |

### Sensor Channel Types

| Channel | Description |
|---------|-------------|
| `SENSOR_CHAN_AMBIENT_TEMP` | Temperature |
| `SENSOR_CHAN_HUMIDITY` | Humidity |
| `SENSOR_CHAN_PRESS` | Pressure |
| `SENSOR_CHAN_ACCEL_XYZ` | Acceleration (all axes) |
| `SENSOR_CHAN_GYRO_XYZ` | Gyroscope (all axes) |
| `SENSOR_CHAN_MAGN_XYZ` | Magnetometer (all axes) |
| `SENSOR_CHAN_DISTANCE` | Distance/proximity |

---

## Additional Resources

- **Zephyr API Documentation**: https://docs.zephyrproject.org/latest/doxygen/html/index.html
- **STM32L4 Reference Manual**: https://www.st.com/resource/en/reference_manual/rm0351-stm32l47xxx-stm32l48xxx-stm32l49xxx-and-stm32l4axxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
- **Board Documentation**: https://docs.zephyrproject.org/latest/boards/st/disco_l475_iot1/doc/index.html
- **Zephyr Samples**: https://docs.zephyrproject.org/latest/samples/index.html

---

## Configuration Checklist

For each feature, ensure you have:

1. **In [prj.conf](prj.conf)**:
   - Enabled required `CONFIG_*` options

2. **In device tree overlay** (if needed):
   - Defined pin mappings
   - Configured peripherals

3. **In code**:
   - Included necessary headers
   - Checked device readiness
   - Error handling for all operations

---

## Common Pitfalls

1. **Forgetting to check device readiness**
   ```c
   if (!device_is_ready(dev)) {
       return -1;
   }
   ```

2. **Not enabling peripheral in prj.conf**
   - Always add `CONFIG_*` for the peripheral you're using

3. **Stack size too small**
   - Increase `STACKSIZE` if threads crash

4. **Priority inversion**
   - Ensure critical threads have higher priority (lower numbers)

5. **Interrupt context limitations**
   - Don't use blocking calls in ISRs
   - Use workqueues or semaphores instead

---

**Last Updated**: 2025-11-02
**Board**: STM32 DISCO_L475_IOT1
**Zephyr Version**: 4.3.0-rc2
