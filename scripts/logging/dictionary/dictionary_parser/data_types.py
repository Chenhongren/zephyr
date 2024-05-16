#!/usr/bin/env python3
#
# Copyright (c) 2024 Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0

"""
Contains a class to describe data types used for
dictionary logging.
"""

import struct

class DataTypes():
    """Class regarding data types, their alignments and sizes"""
    INT = 0
    UINT = 1
    LONG = 2
    ULONG = 3
    LONG_LONG = 4
    ULONG_LONG = 5
    PTR = 6
    DOUBLE = 7
    LONG_DOUBLE = 8
    NUM_TYPES = 9

    def __init__(self, database):
        self.database = database
        self.data_types = {}

        if database.is_tgt_64bit():
            self.add_data_type(self.LONG, "q")
            self.add_data_type(self.LONG_LONG, "q")
            self.add_data_type(self.PTR, "Q")
        else:
            self.add_data_type(self.LONG, "i")
            self.add_data_type(self.LONG_LONG, "q")
            self.add_data_type(self.PTR, "I")

        self.add_data_type(self.INT, "i")
        self.add_data_type(self.DOUBLE, "d")
        self.add_data_type(self.LONG_DOUBLE, "d")


    def add_data_type(self, data_type, fmt):
        """Add one data type"""
        if self.database.is_tgt_little_endian():
            endianness = "<"
        else:
            endianness = ">"

        formatter = endianness + fmt

        self.data_types[data_type] = {}
        self.data_types[data_type]['fmt'] = formatter

        size = struct.calcsize(formatter)

        if data_type == self.LONG_DOUBLE:
            # Python doesn't have long double but we still
            # need to skip correct number of bytes
            size = 16

        self.data_types[data_type]['sizeof'] = size

        # Might need actual number for different architectures
        # but these seem to work fine for now.
        if self.database.is_tgt_64bit():
            self.data_types[data_type]['align'] = 8
        else:
            self.data_types[data_type]['align'] = 4


    def get_sizeof(self, data_type):
        """Get sizeof() of a data type"""
        return self.data_types[data_type]['sizeof']


    def get_alignment(self, data_type):
        """Get the alignment of a data type"""
        return self.data_types[data_type]['align']


    def get_formatter(self, data_type):
        """Get the formatter for a data type"""
        return self.data_types[data_type]['fmt']
