#!/usr/bin/env python3
#
# Copyright (c) 2017 Intel Corporation
# Copyright (c) 2020 Nordic Semiconductor NA
#
# SPDX-License-Identifier: Apache-2.0
"""Translate generic handles into ones optimized for the application.

Immutable device data includes information about dependencies,
e.g. that a particular sensor is controlled through a specific I2C bus
and that it signals event on a pin on a specific GPIO controller.
This information is encoded in the first-pass binary using identifiers
derived from the devicetree.  This script extracts those identifiers
and replaces them with ones optimized for use with the devices
actually present.

For example the sensor might have a first-pass handle defined by its
devicetree ordinal 52, with the I2C driver having ordinal 24 and the
GPIO controller ordinal 14.  The runtime ordinal is the index of the
corresponding device in the static devicetree array, which might be 6,
5, and 3, respectively.

The output is a C source file that provides alternative definitions
for the array contents referenced from the immutable device objects.
In the final link these definitions supersede the ones in the
driver-specific object file.
"""

import sys
import argparse
import os
import pickle

from elf_parser import ZephyrElf

# This is needed to load edt.pickle files.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'dts', 'python-devicetree', 'src'))
from devicetree import edtlib  # pylint: disable=unused-import

def parse_args():
    global args

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)

    parser.add_argument("-k", "--kernel", required=True,
                        help="Input zephyr ELF binary")
    parser.add_argument("-d", "--num-dynamic-devices", required=False, default=0,
                        type=int, help="Input number of dynamic devices allowed")
    parser.add_argument("-o", "--output-source", required=True,
                        help="Output source file")
    parser.add_argument("-z", "--zephyr-base",
                        help="Path to current Zephyr base. If this argument \
                        is not provided the environment will be checked for \
                        the ZEPHYR_BASE environment variable.")
    parser.add_argument("-s", "--start-symbol", required=True,
                        help="Symbol name of the section which contains the \
                        devices. The symbol name must point to the first \
                        device in that section.")

    args = parser.parse_args()

    ZEPHYR_BASE = args.zephyr_base or os.getenv("ZEPHYR_BASE")

    if ZEPHYR_BASE is None:
        sys.exit("-z / --zephyr-base not provided. Please provide "
                 "--zephyr-base or set ZEPHYR_BASE in environment")

    sys.path.insert(0, os.path.join(ZEPHYR_BASE, "scripts/dts"))

def c_handle_comment(dev):
    def dev_path_str(dev):
        return dev.edt_node and dev.edt_node.path or dev.sym.name
    lines = [
        '',
        '/* {:d} : {:s}:'.format(dev.handle, (dev_path_str(dev))),
    ]
    if len(dev.devs_depends_on) > 0:
        lines.append(' * Direct Dependencies:')
        for dep in dev.devs_depends_on:
            lines.append(' *    - {:s}'.format(dev_path_str(dep)))
    if len(dev.devs_depends_on_injected) > 0:
        lines.append(' * Injected Dependencies:')
        for dep in dev.devs_depends_on_injected:
            lines.append(' *    - {:s}'.format(dev_path_str(dep)))
    if len(dev.devs_supports) > 0:
        lines.append(' * Supported:')
        for sup in dev.devs_supports:
            lines.append(' *    - {:s}'.format(dev_path_str(sup)))
    lines.append(' */')
    return lines

def c_handle_array(dev, extra_support_handles=0):
    handles = [
        *[str(d.handle) for d in dev.devs_depends_on],
        'DEVICE_HANDLE_SEP',
        *[str(d.handle) for d in dev.devs_depends_on_injected],
        'DEVICE_HANDLE_SEP',
        *[str(d.handle) for d in dev.devs_supports],
        *(extra_support_handles * ['DEVICE_HANDLE_NULL']),
        'DEVICE_HANDLE_ENDS',
    ]
    return [
        'const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))',
        '{:s}[] = {{ {:s} }};'.format(dev.ordinals.sym.name, ', '.join(handles)),
    ]

def main():
    parse_args()

    edtser = os.path.join(os.path.split(args.kernel)[0], "edt.pickle")
    with open(edtser, 'rb') as f:
        edt = pickle.load(f)

    parsed_elf = ZephyrElf(args.kernel, edt, args.start_symbol)

    with open(args.output_source, "w") as fp:
        fp.write('#include <zephyr/device.h>\n')
        fp.write('#include <zephyr/toolchain.h>\n')
        for dev in parsed_elf.devices:
            extra_sups = args.num_dynamic_devices if dev.pm and dev.pm.is_power_domain else 0
            lines = c_handle_comment(dev)
            lines.extend(c_handle_array(dev, extra_sups))
            lines.extend([''])
            fp.write('\n'.join(lines))

if __name__ == "__main__":
    main()
