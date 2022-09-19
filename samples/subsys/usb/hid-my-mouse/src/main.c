
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

#define GPIO_SPEC(node_id) GPIO_DT_SPEC_GET_OR(node_id, gpios, {0})
static const struct gpio_dt_spec sw0 = GPIO_SPEC(DT_ALIAS(sw0)),
					sw1 = GPIO_SPEC(DT_ALIAS(sw1)),
					sw2 = GPIO_SPEC(DT_ALIAS(sw2)),
					led0 = GPIO_SPEC(DT_ALIAS(led0));

static const uint8_t hid_report_desc[] = HID_MOUSE_REPORT_DESC(2);

static uint8_t def_val[4];
static volatile uint8_t status[4];
static K_SEM_DEFINE(sem, 0, 1);	/* starts off "not available" */
static struct gpio_callback callback[3]; // 3 buttons
static enum usb_dc_status_code usb_status;

#define MOUSE_BTN_REPORT_POS	0
#define MOUSE_X_REPORT_POS	1
#define MOUSE_Y_REPORT_POS	2
#define WHEEL_REPORT_POS	3

#define MOUSE_BTN_LEFT		BIT(0)
#define MOUSE_BTN_RIGHT		BIT(1)
#define MOUSE_BTN_MIDDLE	BIT(2)

#define ZERO_CROSS_THRESHOLD		270

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	usb_status = status;
}

static void left_button(const struct device *gpio, struct gpio_callback *cb,
			uint32_t pins)
{
	int ret;
	uint8_t state = status[MOUSE_BTN_REPORT_POS];

	if (IS_ENABLED(CONFIG_USB_DEVICE_REMOTE_WAKEUP)) {
		if (usb_status == USB_DC_SUSPEND) {
			usb_wakeup_request();
			return;
		}
	}

	ret = gpio_pin_get(gpio, sw0.pin);
	if (ret < 0) {
		LOG_ERR("Failed to get the state of port %s pin %u, error: %d",
			gpio->name, sw0.pin, ret);
		return;
	}

	if (def_val[0] != (uint8_t)ret) {
		state |= MOUSE_BTN_LEFT;
	} else {
		state &= ~MOUSE_BTN_LEFT;
	}

	if (status[MOUSE_BTN_REPORT_POS] != state) {
		status[MOUSE_BTN_REPORT_POS] = state;
		k_sem_give(&sem);
	}
}

static void right_button(const struct device *gpio, struct gpio_callback *cb,
			 uint32_t pins)
{
	int ret;
	uint8_t state = status[MOUSE_BTN_REPORT_POS];

	if (IS_ENABLED(CONFIG_USB_DEVICE_REMOTE_WAKEUP)) {
		if (usb_status == USB_DC_SUSPEND) {
			usb_wakeup_request();
			return;
		}
	}

	ret = gpio_pin_get(gpio, sw1.pin);
	if (ret < 0) {
		LOG_ERR("Failed to get the state of port %s pin %u, error: %d",
			gpio->name, sw1.pin, ret);
		return;
	}

	if (def_val[1] != (uint8_t)ret) {
		state |= MOUSE_BTN_RIGHT;
	} else {
		state &= ~MOUSE_BTN_RIGHT;
	}

	if (status[MOUSE_BTN_REPORT_POS] != state) {
		status[MOUSE_BTN_REPORT_POS] = state;
		k_sem_give(&sem);
	}
}

static void middle_button(const struct device *gpio, struct gpio_callback *cb,
			 uint32_t pins)
{
	int ret;
	uint8_t state = status[MOUSE_BTN_REPORT_POS];

	if (IS_ENABLED(CONFIG_USB_DEVICE_REMOTE_WAKEUP)) {
		if (usb_status == USB_DC_SUSPEND) {
			usb_wakeup_request();
			return;
		}
	}

	ret = gpio_pin_get(gpio, sw2.pin);
	if (ret < 0) {
		LOG_ERR("Failed to get the state of port %s pin %u, error: %d",
			gpio->name, sw2.pin, ret);
		return;
	}

	if (def_val[2] != (uint8_t)ret) {
		state |= MOUSE_BTN_MIDDLE;
	} else {
		state &= ~MOUSE_BTN_MIDDLE;
	}

	if (status[MOUSE_BTN_REPORT_POS] != state) {
		status[MOUSE_BTN_REPORT_POS] = state;
		k_sem_give(&sem);
	}
}

int callbacks_configure(const struct gpio_dt_spec *spec,
			gpio_callback_handler_t handler,
			struct gpio_callback *callback, uint8_t *val)
{
	const struct device *gpio = spec->port;
	gpio_pin_t pin = spec->pin;
	int ret;

	if (gpio == NULL) {
		/* Optional GPIO is missing. */
		return 0;
	}

	if (!device_is_ready(gpio)) {
		LOG_ERR("GPIO port %s is not ready", gpio->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(spec, GPIO_INPUT);
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

	*val = (uint8_t)ret;

	gpio_init_callback(callback, handler, BIT(pin));
	ret = gpio_add_callback(gpio, callback);
	if (ret < 0) {
		LOG_ERR("Failed to add the callback for port %s pin %u, "
			"error: %d",
			gpio->name, pin, ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(spec, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("Failed to configure interrupt for port %s pin %u, "
			"error: %d",
			gpio->name, pin, ret);
		return ret;
	}

	return 0;
}

void main(void)
{
	int ret;
	uint8_t report[4] = { 0x00 };
	const struct device *hid_dev;

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

	if (callbacks_configure(&sw0, &left_button, &callback[0],
				&def_val[0])) {
		LOG_ERR("Failed configuring left button callback.");
		return;
	}

	if (callbacks_configure(&sw1, &right_button, &callback[1],
				&def_val[1])) {
		LOG_ERR("Failed configuring right button callback.");
		return;
	}

	if (callbacks_configure(&sw2, &middle_button, &callback[2],
				&def_val[2])) {
		LOG_ERR("Failed configuring right button callback.");
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
		k_sem_take(&sem, K_FOREVER);

		report[MOUSE_BTN_REPORT_POS] = status[MOUSE_BTN_REPORT_POS];
		report[WHEEL_REPORT_POS] = status[WHEEL_REPORT_POS];
		
		ret = hid_int_ep_write(hid_dev, report, sizeof(report), NULL);
		if (ret) {
			LOG_ERR("HID write error, %d", ret);
		}
	}
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

	ret = sensor_attr_get(dev, SENSOR_CHAN_RAW_COUNTER, SENSOR_ATTR_MAX_VALUE, &sensor_val);
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
			
			status[WHEEL_REPORT_POS] = delta_val;
			LOG_INF("curr=%d - prev=%d - rep=%hhd",
						curr_val, prev_val,
						status[WHEEL_REPORT_POS]);
			prev_val = curr_val;
			k_sem_give(&sem);
		}

		k_msleep(50);
	}
}

K_THREAD_DEFINE(encoder_task, STACKSIZE, encoder_task_func, NULL, NULL, NULL,
		K_PRIO_PREEMPT(0), 0, 0);
