/**
 * @file
 * @brief Bluetooth Hearing Access Service (HAS) shell.
 *
 * Copyright (c) 2022 Codecoup
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/audio/has.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <stdio.h>

#include "bt.h"

static struct bt_has *inst;

static void has_client_discover_cb(struct bt_conn *conn, int err, struct bt_has *has,
				   enum bt_has_hearing_aid_type type,
				   enum bt_has_capabilities caps)
{
	if (err) {
		shell_error(ctx_shell, "HAS discovery (err %d)", err);
		return;
	}

	shell_print(ctx_shell, "HAS discovered %p type 0x%02x caps 0x%02x for conn %p",
		    has, type, caps, conn);

	inst = has;
}

static void has_client_preset_switch_cb(struct bt_has *has, uint8_t index)
{
	shell_print(ctx_shell, "HAS %p preset switch index 0x%02x", has, index);
}

static const struct bt_has_client_cb has_client_cb = {
	.discover = has_client_discover_cb,
	.preset_switch = has_client_preset_switch_cb,
};

static int cmd_has_client_init(const struct shell *sh, size_t argc, char **argv)
{
	int err;

	if (!ctx_shell) {
		ctx_shell = sh;
	}

	err = bt_has_client_cb_register(&has_client_cb);
	if (err != 0) {
		shell_error(sh, "bt_has_client_cb_register (err %d)", err);
	}

	return err;
}

static int cmd_has_client_discover(const struct shell *sh, size_t argc, char **argv)
{
	int err;

	if (default_conn == NULL) {
		shell_error(sh, "Not connected");
		return -ENOEXEC;
	}

	if (!ctx_shell) {
		ctx_shell = sh;
	}

	err = bt_has_client_discover(default_conn);
	if (err != 0) {
		shell_error(sh, "bt_has_client_discover (err %d)", err);
	}

	return err;
}

static int cmd_has_client(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1) {
		shell_error(sh, "%s unknown parameter: %s", argv[0], argv[1]);
	} else {
		shell_error(sh, "%s missing subcomand", argv[0]);
	}

	return -ENOEXEC;
}

#define HELP_NONE "[none]"

SHELL_STATIC_SUBCMD_SET_CREATE(has_client_cmds,
	SHELL_CMD_ARG(init, NULL, HELP_NONE, cmd_has_client_init, 1, 0),
	SHELL_CMD_ARG(discover, NULL, HELP_NONE, cmd_has_client_discover, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_ARG_REGISTER(has_client, &has_client_cmds, "Bluetooth HAS client shell commands",
		       cmd_has_client, 1, 1);
