#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2022/7/15 18:19
# @Author  : 李洋
# @File    : GY302_BH1750.py

"""
光亮度数据参考
单位： lx
晚上： 0.001-0.02；
月夜： 0.02-0.3；
多云室内： 5-50；
多云室外： 50-500；
晴天室内： 100-1000；
夏天中午光照下： 大约10*6能量；
阅读书籍时的照明度：50-60；
家庭录像标准照明度：1400
"""

import utime
from machine import Pin
from machine import I2C, SoftI2C


OP_SINGLE_HRES1 = 0x20
OP_SINGLE_HRES2 = 0x21
OP_SINGLE_LRES = 0x23

DELAY_HMODE = 180  # 180ms in H-mode
DELAY_LMODE = 24  # 24ms in L-mode


class BH1750_Sensor:

    def __init__(self, sdapin, sclpin, mode=OP_SINGLE_HRES1):
        # self.i2c = I2C(sda=Pin(sdapin), scl=Pin(sclpin))
        self.i2c = SoftI2C(sda=Pin(sdapin), scl=Pin(sclpin))
        addrs = self.i2c.scan()
        if not addrs:
            raise Exception('no BH1750 found at bus on SDA pin %d SCL pin %d' % (sdapin, sclpin))
        self.i2c_addr = 0x23
        self.mode = mode

    def read_data(self):
        self.i2c.writeto(self.i2c_addr, b"\x00")  # make sure device is in a clean state
        self.i2c.writeto(self.i2c_addr, b"\x01")  # power up
        self.i2c.writeto(self.i2c_addr, bytes([self.mode]))  # set measurement mode

        utime.sleep_ms(DELAY_LMODE if self.mode == OP_SINGLE_LRES else DELAY_HMODE)

        raw = self.i2c.readfrom(self.i2c_addr, 2)
        self.i2c.writeto(self.i2c_addr, b"\x00")  # power down again

        # we must divide the end result by 1.2 to get the lux
        # (buffer[0] << 8 | buffer[1]) / (1.2)
        return ((raw[0] << 24) | (raw[1] << 16)) / 78642



