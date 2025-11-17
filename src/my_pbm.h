#ifndef MY_PBM_H
#define MY_PBM_H

#include <zephyr/types.h>


/** @brief Callback type for when an LED state change is received. */
typedef void (*led_cb_t)(const bool led_state);

/** @brief Callback type for when the button state is pulled. */
typedef bool (*button_cb_t)(void);

/** @brief Callback struct used by the PBM Service. */
struct my_pbm_cb {
	led_cb_t led_cb;
	button_cb_t button_cb;
};

enum CommandType {
	CMD_STOP_ALL = 0x00,
    CMD_BATTERY_CHECK = 0x01,
    CMD_READ_CONFIG = 0x04,
    CMD_SET_CONFIG = 0x05,
	CMD_START_SINGLE = 0x11,
    CMD_START_CONTINUOUS = 0x12,
	CMD_STOP_MEASUREMENT = 0x13,
};

extern uint8_t default_command[16];

int my_pbm_init(void);
int my_pbm_send_button_state_indicate(bool button_state);
int my_pbm_send_button_state_notify(bool button_state);
int my_pbm_send_sensor_notify(uint8_t *sensor_value);

#endif // MY_PBM_H
