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
#include <dk_buttons_and_leds.h>
#include "model_handler.h"

/* Light switch behavior */

/** Context for a single light switch. */
struct button {
	/** Current light status of the corresponding server. */
	int16_t status;

	/** Generic OnOff client instance for this switch. */
	struct bt_mesh_lvl_cli client;
};

static void status_handler(struct bt_mesh_lvl_cli *cli,
			   struct bt_mesh_msg_ctx *ctx,
			   const struct bt_mesh_lvl_status *status);

static struct button buttons[] = {

	{ .client = BT_MESH_LVL_CLI_INIT(&status_handler) },

	{ .client = BT_MESH_LVL_CLI_INIT(&status_handler) },

};

static void status_handler(struct bt_mesh_lvl_cli *cli,
			   struct bt_mesh_msg_ctx *ctx,
			   const struct bt_mesh_lvl_status *status)
{
	struct button *button =
		CONTAINER_OF(cli, struct button, client);
	int index = button - &buttons[0];

	button->status = status->current;

	printk("Button %d: Received response: %d\n", index + 1,
	       status->current );
}

static void button_handler_cb(uint32_t pressed, uint32_t changed)
{

	
	if (!bt_mesh_is_provisioned()) {
		return;
	}

	for (int i = 0; i < 4; ++i) {
		if (!(pressed & changed & BIT(i))) {
			continue;
		}

		int tmp = (i < 2)? i : (i - 2);
		
		
		if  ( (i < 2) && (buttons[tmp].status ==0x7000) )
		{		
			continue;
		}
		
		if  ( (i >= 2) && (buttons[tmp].status == BT_MESH_LVL_MIN) )
		{
			continue;
		}
		

		struct bt_mesh_lvl_set set = {
			.lvl =  ( i < 2 )? (buttons[tmp].status + 0x1000)  : (buttons[tmp].status - 0x1000),
			.new_transaction = true,
		};
		
		int err;

		/* As we can't know how many nodes are in a group, it doesn't
		 * make sense to send acknowledged messages to group addresses -
		 * we won't be able to make use of the responses anyway.
		 */
		if (bt_mesh_model_pub_is_unicast(buttons[tmp].client.model)) {
			err = bt_mesh_lvl_cli_set(&buttons[tmp].client, NULL,
						    &set, NULL);
		} else {
			err = bt_mesh_lvl_cli_set_unack(&buttons[tmp].client,
							  NULL, &set);
			if (!err) {
				/* There'll be no response status for the
				 * unacked message. Set the state immediately.
				 */
				buttons[tmp].status = set.lvl;
			}
		}

		if (err) {
			printk("Level %d set failed: %d\n", i + 1, err);
		}
	}
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
			     BT_MESH_MODEL_LVL_CLI(&buttons[0].client)),
		     BT_MESH_MODEL_NONE),

	BT_MESH_ELEM(2,
		     BT_MESH_MODEL_LIST(
			     BT_MESH_MODEL_LVL_CLI(&buttons[1].client)),
		     BT_MESH_MODEL_NONE),


};

static const struct bt_mesh_comp comp = {
	.cid = CONFIG_BT_COMPANY_ID,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

const struct bt_mesh_comp *model_handler_init(void)
{
	static struct button_handler button_handler = {
		.cb = button_handler_cb,
	};

	dk_button_handler_add(&button_handler);
	k_work_init_delayable(&attention_blink_work, attention_blink);

	return &comp;
}
