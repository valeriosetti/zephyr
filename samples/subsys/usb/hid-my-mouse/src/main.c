
#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/qdec_stm32.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(main);

#define STACKSIZE		1024

#define GPIO_SPEC(node_id)	GPIO_DT_SPEC_GET_OR(node_id, gpios, {0})
static const uint8_t hid_report_desc[] = HID_MOUSE_REPORT_DESC(2);
static enum usb_dc_status_code usb_status;

static K_SEM_DEFINE(report_sem, 0, 1);

struct button_props {
	const struct gpio_dt_spec dt_spec;
	uint8_t idle_val;
	uint8_t status;
	uint8_t debounce_counter;
};

#define DEBOUNCE_CYCLES 	5

enum buttons_name{
	LEFT_BUTTON,
	RIGHT_BUTTON,
	MIDDLE_BUTTON
};

static struct button_props buttons_props[] = {
	[LEFT_BUTTON] = {
		.dt_spec = GPIO_SPEC(DT_ALIAS(sw0)),
	},
	[RIGHT_BUTTON] = {
		.dt_spec = GPIO_SPEC(DT_ALIAS(sw1)),
	},
	[MIDDLE_BUTTON] = {
		.dt_spec = GPIO_SPEC(DT_ALIAS(sw2)),
	},
};

static const struct gpio_dt_spec led0 = GPIO_SPEC(DT_ALIAS(led0));

struct hid_report {
	uint8_t buttons;
	int8_t x;
	int8_t y;
	int8_t wheel;
} __packed;

#define LEFT_BTN_BIT		BIT(0)
#define RIGHT_BTN_BIT		BIT(1)
#define MIDDLE_BTN_BIT		BIT(2)

static int8_t wheel_change;

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	usb_status = status;
}

void button_task_func(void)
{
	int ret, i;
	struct button_props* curr_btn;
	bool is_state_changed = false;
	
	while (true) {
		for (i=0; i<ARRAY_SIZE(buttons_props); i++) {
			curr_btn = &buttons_props[i];

			if (curr_btn->dt_spec.port == NULL) {
				continue;
			}
			
			ret = gpio_pin_get(curr_btn->dt_spec.port,
						curr_btn->dt_spec.pin);
			if (ret < 0) {
				LOG_ERR("Failed to get the state of port %s pin %u, error: %d",
					curr_btn->dt_spec.port->name,
					curr_btn->dt_spec.pin, ret);
				continue;
			}

			if (curr_btn->idle_val) {
				ret = !ret;
			}

			if (curr_btn->status != (uint8_t)ret) {
				curr_btn->debounce_counter++;
			} else {
				curr_btn->debounce_counter = 0;
			}

			if (curr_btn->debounce_counter >= DEBOUNCE_CYCLES) {
				curr_btn->debounce_counter = 0;
				curr_btn->status = ret;
				is_state_changed = true;
			}
		}

		if (is_state_changed) {
			k_sem_give(&report_sem);
			is_state_changed = false;
		}
		
		k_msleep(10);
	}
}
K_THREAD_DEFINE(button_task, STACKSIZE, button_task_func, NULL, NULL, NULL,
		K_PRIO_PREEMPT(0), 0, 0);

int gpio_configure(struct button_props* btn_props)
{
	const struct device *gpio = btn_props->dt_spec.port;
	gpio_pin_t pin = btn_props->dt_spec.pin;
	int ret;

	if (gpio == NULL) {
		/* Optional GPIO is missing. */
		return 0;
	}

	if (!device_is_ready(gpio)) {
		LOG_ERR("GPIO port %s is not ready", gpio->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&(btn_props->dt_spec), GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure port %s pin %u, error: %d",
			gpio->name, pin, ret);
		return ret;
	}

	ret = gpio_pin_get(gpio, pin);
	if (ret < 0) {
		LOG_ERR("Failed to get the state of port %s pin %u, error: %d",
			gpio->name, pin, ret);
		return ret;
	}

	btn_props->idle_val = ret;

	return 0;
}

void blink_task_func(void)
{
	int ret;

	while (1) {
		ret = gpio_pin_toggle(led0.port, led0.pin);
		if (ret < 0) {
			LOG_ERR("Failed to toggle the LED pin, error: %d", ret);
		}
		k_msleep(1000);
	}
}
K_THREAD_DEFINE(blink_task, STACKSIZE, blink_task_func, NULL, NULL, NULL,
		K_PRIO_PREEMPT(0), 0, 0);

void encoder_task_func(void)
{
	const struct device *dev = DEVICE_DT_GET_ONE(st_stm32_qdec);
	struct sensor_value sensor_val;
	uint32_t curr_val, prev_val, max_val;
	int32_t delta_val;
	int ret;

	prev_val = 0;

	ret = sensor_attr_get(dev, SENSOR_CHAN_RAW_COUNTER, SENSOR_ATTR_MAX_VALUE,
				&sensor_val);
	if (ret) {
		printk("Failed to parse counter attribute (%d)\n", ret);
		return;
	}
	max_val = sensor_val.val1;
	
	while (1) {
		ret = sensor_sample_fetch(dev);
		if (ret) {
			printk("Failed to fetch sample (%d)\n", ret);
			continue;
		}

		ret = sensor_channel_get(dev, SENSOR_CHAN_RAW_COUNTER, &sensor_val);
		if (ret) {
			printk("Failed to get data (%d)\n", ret);
			continue;
		}

		curr_val = sensor_val.val1;

		if (curr_val != prev_val) {
			if ((curr_val < UINT16_MAX/2) && (prev_val > UINT16_MAX/2)) {
				delta_val = curr_val + max_val - prev_val + 1;
			} else if ((curr_val > UINT16_MAX/2) && (prev_val < UINT16_MAX/2)) {
				delta_val = max_val - curr_val + prev_val + 1;
				delta_val *= -1;
			} else {
				delta_val = curr_val - prev_val;
			}
			
			wheel_change = delta_val;
			//LOG_INF("curr=%d - prev=%d - rep=%hhd",
			//			curr_val, prev_val,
			//			wheel_change);
			prev_val = curr_val;
			k_sem_give(&report_sem);
		}

		k_msleep(50);
	}
}
K_THREAD_DEFINE(encoder_task, STACKSIZE, encoder_task_func, NULL, NULL, NULL,
		K_PRIO_PREEMPT(0), 0, 0);

void main(void)
{
	int ret;
	const struct device *hid_dev;
	struct hid_report report;

	if (!device_is_ready(led0.port)) {
		LOG_ERR("LED device %s is not ready", led0.port->name);
		return;
	}

	hid_dev = device_get_binding("HID_0");
	if (hid_dev == NULL) {
		LOG_ERR("Cannot get USB HID Device");
		return;
	}

	ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure the LED pin, error: %d", ret);
		return;
	}

	if (gpio_configure(&buttons_props[LEFT_BUTTON])) {
		LOG_ERR("Failed configuring left button.");
		return;
	}

	if (gpio_configure(&buttons_props[RIGHT_BUTTON])) {
		LOG_ERR("Failed configuring right button.");
		return;
	}

	if (gpio_configure(&buttons_props[MIDDLE_BUTTON])) {
		LOG_ERR("Failed configuring middle button.");
		return;
	}

	usb_hid_register_device(hid_dev,
				hid_report_desc, sizeof(hid_report_desc),
				NULL);

	usb_hid_init(hid_dev);

	ret = usb_enable(status_cb);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}
	
	while (true) {
		k_sem_take(&report_sem, K_FOREVER);

		memset(&report, 0, sizeof(report));

		if (buttons_props[LEFT_BUTTON].status) {
			report.buttons |= LEFT_BTN_BIT;
		}
		if (buttons_props[RIGHT_BUTTON].status) {
			report.buttons |= RIGHT_BTN_BIT;
		}
		if (buttons_props[MIDDLE_BUTTON].status) {
			report.buttons |= MIDDLE_BTN_BIT;
		}
		
		report.wheel = wheel_change;
		wheel_change = 0;
		
		ret = hid_int_ep_write(hid_dev, (uint8_t*)&report, sizeof(report),
					NULL);
		if (ret) {
			LOG_ERR("HID write error, %d", ret);
		}
	}
}
