/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
/**
 * @file
 * @brief Model handler for the light switch.
 *
 * Instantiates a Generic OnOff Client model for each button on the devkit, as
 * well as the standard Config and Health Server models. Handles all application
 * behavior related to the models.
 */
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <zephyr/types.h>
#include <zephyr.h>
#include <drivers/uart.h>
#include <usb/usb_device.h>
#include <device.h>
#include <soc.h>
#include <dk_buttons_and_leds.h>
#include "uart_async_adapter.h"
#include "model_handler.h"

#define STACKSIZE CONFIG_UART_THREAD_STACK_SIZE
#define PRIORITY 7

#define UART_BUF_SIZE 3
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_UART_RX_WAIT_TIME

#define UART_BUF_ELT	0
#define UART_BUF_LVL	1
#define UART_BUF_TAIL	2

static const struct device *uart;
static struct k_work_delayable uart_work;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

#if CONFIG_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
static const struct device *const async_adapter;
#en

/* Light switch behavior */

/** Context for a single light switch. */
struct uart_elt {
	/** Current light status of the corresponding server. */
	int16_t status;

	/** Generic level client instance for this switch. */
	struct bt_mesh_lvl_cli client;
};

static void status_handler(struct bt_mesh_lvl_cli *cli,
			   struct bt_mesh_msg_ctx *ctx,
			   const struct bt_mesh_lvl_status *status);

static struct uart_elt uart_elts[] = {

	{ .client = BT_MESH_LVL_CLI_INIT(&status_handler) },

	{ .client = BT_MESH_LVL_CLI_INIT(&status_handler) },

};

static void status_handler(struct bt_mesh_lvl_cli *cli,
			   struct bt_mesh_msg_ctx *ctx,
			   const struct bt_mesh_lvl_status *status)
{
	struct uart_elt *uart =
		CONTAINER_OF(cli, struct uart_elt, client);
	int index = uart - &uart_elts[0];

	uart->status = status->current;

	printk("Button %d: Received response: %d\n", index + 1,
	       status->current );
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static uint8_t *current_buf;
	static size_t aborted_len;
	static bool buf_release;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	
	
	if (!bt_mesh_is_provisioned()) {
		return;
	}
	
	switch (evt->type) {
	case UART_TX_DONE:
	
		break;
	
	case UART_RX_RDY:
	
		LOG_DBG("rx_rdy");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		buf->len += evt->data.rx.len;
		buf_release = false;

		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
			  (evt->data.rx.buf[buf->len - 1] == '\r')) {
			k_fifo_put(&fifo_uart_rx_data, buf);
			current_buf = evt->data.rx.buf;
			buf_release = true;
			uart_rx_disable(uart);
		}
	
	
		break;
	
	case UART_RX_DISABLED:
	
		break;
	
	case UART_RX_BUF_REQUEST:
	
		LOG_DBG("rx_buf_request");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;
	
	case UART_RX_BUF_RELEASED:
	
		LOG_DBG("rx_buf_released");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data);
		if (buf_release && (current_buf != evt->data.rx_buf.buf)) {
			
			k_free(buf);
			buf_release = false;
			current_buf = NULL;
	
			//Send rx_buf data thru BLE mesh
			buf = k_fifo_get(&fifo_uart_rx_data, K_FOREVER);
			
			int err;
			uint8_t i = buf.data[UART_BUF_ELT];

				
			struct bt_mesh_lvl_set set = {
				.lvl = buf.data[UART_BUF_LVL] ,
				.new_transaction = true,
			}
			
			
			if (bt_mesh_model_pub_is_unicast(uart_elts[i].client.model)) {
				err = bt_mesh_lvl_cli_set(&uart_elts[i].client, NULL,
								&set, NULL);
			} else {
				err = bt_mesh_lvl_cli_set_unack(&uart_elts[i].client,
								  NULL, &set);
				if (!err) {
					/* There'll be no response status for the
					 * unacked message. Set the state immediately.
					 */
					uart_elts[i].status = set.lvl;
				}
			}

			if (err) {
				printk("Level %d set failed: %d\n", i + 1, err);
			}
		
		}
		
	
		break;
	
	case UART_TX_ABORTED:
	
		break;
	
	default:
	
		break;
	

}

/* Set up a repeating delayed work to blink the DK's LEDs when attention is
 * requested.
 */
static struct k_work_delayable attention_blink_work;
static bool attention;

static void attention_blink(struct k_work *work)
{
	static int idx;
	const uint8_t pattern[] = {
#if DT_NODE_EXISTS(DT_ALIAS(sw0))
		BIT(0),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(sw1))
		BIT(1),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(sw2))
		BIT(2),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(sw3))
		BIT(3),
#endif
	};

	if (attention) {
		dk_set_leds(pattern[idx++ % ARRAY_SIZE(pattern)]);
		k_work_reschedule(&attention_blink_work, K_MSEC(30));
	} else {
		dk_set_leds(DK_NO_LEDS_MSK);
	}
}

static void attention_on(struct bt_mesh_model *mod)
{
	attention = true;
	k_work_reschedule(&attention_blink_work, K_NO_WAIT);
}

static void attention_off(struct bt_mesh_model *mod)
{
	/* Will stop rescheduling blink timer */
	attention = false;
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

static struct bt_mesh_elem elements[] = {

	BT_MESH_ELEM(1,
		     BT_MESH_MODEL_LIST(
			     BT_MESH_MODEL_CFG_SRV,
			     BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
			     BT_MESH_MODEL_LVL_CLI(&uart_elts[0].client)),
		     BT_MESH_MODEL_NONE),

	BT_MESH_ELEM(2,
		     BT_MESH_MODEL_LIST(
			     BT_MESH_MODEL_LVL_CLI(&uart_elts[1].client)),
		     BT_MESH_MODEL_NONE),

};

static const struct bt_mesh_comp comp = {
	.cid = CONFIG_BT_COMPANY_ID,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	uart = device_get_binding(UART_DEV);
	if (!uart) {
		return -ENXIO;
	}

	if (IS_ENABLED(CONFIG_USB)) {
		err = usb_enable(NULL);
		if (err) {
			LOG_ERR("Failed to enable USB");
			return err;
		}
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);


	if (IS_ENABLED(CONFIG_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		LOG_INF("Wait for DTR");
		while (true) {
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) {
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err) {
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err) {
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx) {
		pos = snprintf(tx->data, sizeof(tx->data),
			       "Starting Nordic UART service example\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data))) {
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	} else {
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	return uart_rx_enable(uart, rx->data, sizeof(rx->data), 50);
}



const struct bt_mesh_comp *model_handler_init(void)
{

	uart_init();

	k_work_init_delayable(&attention_blink_work, attention_blink);

	return &comp;
}

