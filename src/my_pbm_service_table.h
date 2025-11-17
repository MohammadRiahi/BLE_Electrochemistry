// my_pbm_service_table.h
/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */



/**@file
 * @defgroup bt_lbs LED Button Service API
 * @{
 * @brief API for the LED Button Service (LBS).
 */



#ifndef MY_PBM_SERVICE_TABLE_H
#define MY_PBM_SERVICE_TABLE_H


#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include "my_pbm.h"

#include <zephyr/types.h>

/** @brief PBM Service and Advertising UUID. */
#define BT_UUID_PBM_VAL                BT_UUID_128_ENCODE(0x0000525F, 0x45D0, 0x4AA9, 0xDB0A, 0x93D9E09A7CFC)
#define BT_UUID_PBM_ADVERTISING_VAL    BT_UUID_128_ENCODE(0x000062C4, 0xB99E, 0x4141, 0x9439, 0xC4F9DB977899)
                                                          
/** @brief COMMAND Characteristic UUID. */
#define BT_UUID_PBM_COMMAND_VAL                                                       \
	BT_UUID_128_ENCODE(0x0100525F, 0x45D0, 0x4AA9, 0xDB0A, 0x93D9E09A7CFC)

/** @brief MESSAGE Characteristic UUID. */
#define BT_UUID_PBM_MESSAGE_VAL BT_UUID_128_ENCODE(0x0300525F, 0x45D0, 0x4AA9, 0xDB0A, 0x93D9E09A7CFC)

/** @brief DATA Characteristic UUID. */
#define BT_UUID_PBM_DATA_VAL                                                 \
	BT_UUID_128_ENCODE(0x0200525F, 0x45D0, 0x4AA9, 0xDB0A, 0x93D9E09A7CFC)

	/** @brief HeartBeat Characteristic UUID. */
#define BT_UUID_PBM_HEARTBEAT_VAL    BT_UUID_128_ENCODE(0x0400525F, 0x45D0, 0x4AA9, 0xDB0A, 0x93D9E09A7CFC)

#define BT_UUID_PBM_ADVERTISING  BT_UUID_DECLARE_128(BT_UUID_PBM_ADVERTISING_VAL)
#define BT_UUID_PBM              BT_UUID_DECLARE_128(BT_UUID_PBM_VAL)
#define BT_UUID_PBM_COMMAND      BT_UUID_DECLARE_128(BT_UUID_PBM_COMMAND_VAL)
#define BT_UUID_PBM_MESSAGE      BT_UUID_DECLARE_128(BT_UUID_PBM_MESSAGE_VAL)
#define BT_UUID_PBM_DATA         BT_UUID_DECLARE_128(BT_UUID_PBM_DATA_VAL)
#define BT_UUID_PBM_HEARTBEAT    BT_UUID_DECLARE_128(BT_UUID_PBM_HEARTBEAT_VAL)

// GATT Service Declaration (extern so it can be used in .c)
extern const struct bt_gatt_service_static my_pbm_svc;


#endif /* MY_PBM_SERVICE_TABLE_H_ */

/**
 * @}
 */
