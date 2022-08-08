load("//:defs.bzl", "PORT_NAMES")

[config_setting(
    name = port_name,
    flag_values = {"//config:port": port_name},
) for port_name in PORT_NAMES]
