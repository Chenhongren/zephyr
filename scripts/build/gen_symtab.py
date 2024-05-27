#!/usr/bin/env python3
#
# Copyright (c) 2024 Meta Platforms
#
# SPDX-License-Identifier: Apache-2.0

import argparse
import sys
import os
import re

from elftools.elf.elffile import ELFFile
from elftools.elf.descriptions import (
    describe_symbol_type,
)


class gen_symtab_log:

    def __init__(self, debug=False):
        self.__debug = debug

    def debug(self, text):
        """Print debug message if debugging is enabled.

        Note - this function requires config global variable to be initialized.
        """
        if self.__debug:
            sys.stdout.write(os.path.basename(
                sys.argv[0]) + ": " + text + "\n")

    @staticmethod
    def error(text):
        sys.exit(os.path.basename(sys.argv[0]) + ": error: " + text + "\n")

    def set_debug(self, state):
        self.__debug = state


log = gen_symtab_log()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter, allow_abbrev=False)

    parser.add_argument("-k", "--kernel", required=True,
                        help="Zephyr kernel image")
    parser.add_argument("-o", "--output", required=True,
                        help="Output source file")
    parser.add_argument("-d", "--debug", action="store_true",
                        help="Print additional debugging information")

    return parser.parse_args()


class symtab_entry:
    def __init__(self, addr, offset, name):
        self.addr = addr
        self.offset = offset
        self.name = name


start_addr = 0
symtab_list = []


def sanitize_func_name(name):
    pattern = r'(^[a-zA-Z_][a-zA-Z0-9_]*)'
    match = re.match(pattern, name)
    if match:
        return match.group(0)
    else:
        log.error(f"Failed to sanitize function name: {name}")

    return name


def main():
    args = parse_args()
    log.set_debug(args.debug)

    with open(args.kernel, "rb") as rf:
        elf = ELFFile(rf)

        # Find the symbol table.
        symtab = elf.get_section_by_name('.symtab')

        i = 1
        for nsym, symbol in enumerate(symtab.iter_symbols()):  # pylint: disable=unused-variable
            symbol_type = describe_symbol_type(symbol['st_info']['type'])
            symbol_addr = symbol['st_value']

            if symbol_type == 'FUNC' and symbol_addr != 0:
                symbol_name = sanitize_func_name(symbol.name)
                symtab_list.append(symtab_entry(
                    symbol_addr, symbol_addr, symbol_name))
                log.debug('%6d: %s %.25s' % (
                    i,
                    hex(symbol_addr),
                    symbol_name))
                i = i + 1

        symtab_list.sort(key=lambda x: x.addr, reverse=False)

        # Get the address of the first symbol
        start_addr = symtab_list[0].addr
        # Use that to calculate the offset of other symbols relative to the first one
        for i, entry in enumerate(symtab_list):
            entry.offset = entry.addr - start_addr

    with open(args.output, 'w') as wf:
        print("/* AUTO-GENERATED by gen_symtab.py, do not edit! */", file=wf)
        print("", file=wf)
        print("#include <zephyr/debug/symtab.h>", file=wf)
        print("", file=wf)
        print(
            f"const struct z_symtab_entry z_symtab_entries[{len(symtab_list)}] = {{", file=wf)
        for i, entry in enumerate(symtab_list):
            print(
                f"\t[{i}] = {{.offset = {hex(entry.offset)}, .name = \"{entry.name}\"}}, /* {hex(entry.addr)} */", file=wf)
        print(f"}};\n", file=wf)

        print(f"const struct symtab_info z_symtab = {{", file=wf)
        print(f"\t.start_addr = {hex(start_addr)},", file=wf)
        print(f"\t.length = {len(symtab_list)},", file=wf)
        print(f"\t.entries = z_symtab_entries,", file=wf)
        print(f"}};\n", file=wf)


if __name__ == "__main__":
    main()
