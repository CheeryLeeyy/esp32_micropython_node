from machine import RTC, reset, Timer, I2C, Pin, SoftI2C
import network
import time
import ssd1306
import ntptime
import json

from GY302_BH1750 import BH1750_Sensor
from bmp280 import BMP280
from sgp30 import SGP30
from sht3x import SHT3x_Sensor
from umqtt_simple import MQTTClient

node_name = "node1"

# MQTT 参数
mqtt_host = "192.168.1.20"
mqtt_port = 9883
mqtt_recv = "/mysmarthome/" + node_name + "/recv"
mqtt_send = "/mysmarthome/send"
mqtt_resp = "/mysmarthome/resp"


# /mysmarthome/node1/recv
# /mysmarthome/send
# /mysmarthome/resp


# wifi
def do_connect(ssid, password, led=None):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    connected = wlan.isconnected()
    if not connected:
        print('connecting to network...')
        wlan.connect(ssid, password)
        for retry in range(100):
            connected = wlan.isconnected()
            if connected:
                break
            time.sleep(0.1)
            print('.', end='')
            if led:
                led.value(not led.value())
    if connected:
        print('\nConnected : ', wlan.ifconfig())
        connected = wlan.ifconfig()
    else:
        print('\nFailed. Not Connected to: ' + ssid)
    return connected


# LED指示灯
led = Pin(23, Pin.OUT)
led.value(1)

wifi_info = do_connect(ssid="Yang_Home_2.4G", password="Liyang123", led=led)
if not wifi_info:
    reset()


def sync_ntp(timer=None):
    """通过网络校准时间"""
    import ntptime
    ntptime.NTP_DELTA = 3155644800  # 可选 UTC+8偏移时间（秒），不设置就是UTC0
    ntptime.host = 'ntp1.aliyun.com'  # 可选，ntp服务器，默认是"pool.ntp.org" 这里使用阿里服务器
    try:
        ntptime.settime()
    except:
        pass


# 获取NTP时间
# rtc = RTC()
sync_ntp()
time_tuple = time.localtime()
print(time_tuple)

# 传感器初始化
### 数据字典
data = {
    "node_name": node_name,
    "ip": wifi_info[0],
    "temperature": 0.0,
    "relative_humidity": 0.0,
    "illuminance": 0.0,
    "tvoc": 0.0,
    "eco2": 0.0,
    "pressure": 0.0,
}

dat_unit = {
    "temperature": "C",
    "relative_humidity": "%",
    "illuminance": "lux",
    "tvoc": "ppb",
    "eco2": "ppm",
    "pressure": "kPa",
}

try:
    # OLED 显示屏
    i2c = SoftI2C(scl=Pin(22), sda=Pin(21))
    oled = ssd1306.SSD1306_I2C(128, 64, i2c)
    oled.text(wifi_info[0], 0, 0, 1)
    oled.show()
    # SHT30 温湿度传感器
    sht30 = SHT3x_Sensor(sdapin=14, sclpin=27)
    data["temperature"], data["relative_humidity"] = sht30.read_data()
    print("SHT30: ", data["temperature"], data["relative_humidity"])
    # BH1750光照度传感器
    bh1750 = BH1750_Sensor(sdapin=26, sclpin=25)
    data["illuminance"] = bh1750.read_data()
    print("BH1750: ", data["illuminance"])
    # SGP30气体传感器模块 TVOC/eCO2 空气质量甲醛 二氧化碳测量
    sgp30 = SGP30(sdapin=33, sclpin=32)
    sgp30.iaq_init()
    sgp30.set_iaq_rel_humidity(rh=data["relative_humidity"], temp=data["temperature"])
    data["eco2"], data["tvoc"] = sgp30.read_data()
    print("tVOC: ", data["tvoc"], " ppb")
    print("CO2eq: ", data["eco2"], " ppm")
    print("get_iaq_baseline", sgp30.get_iaq_baseline())
    # BMP280-3.3 高精度大气压强模块
    bmp280 = BMP280(sdapin=19, sclpin=18)
    data["pressure"] = bmp280.read_data()
    print("BMP280: ", data["pressure"], " kPa")

except Exception as e:
    print(e)
    time.sleep(0.5)
    # 传感器重新初始化
    try:
        sht30 = SHT3x_Sensor(sdapin=14, sclpin=27)
        data["temperature"], data["relative_humidity"] = sht30.read_data()
        bh1750 = BH1750_Sensor(sdapin=26, sclpin=25)
        sgp30 = SGP30(sdapin=33, sclpin=32)
        sgp30.iaq_init()
        sgp30.set_iaq_rel_humidity(rh=data["relative_humidity"], temp=data["temperature"])
        data["eco2"], data["tvoc"] = sgp30.read_data()
        bmp280 = BMP280(sdapin=19, sclpin=18)
        data["pressure"] = bmp280.read_data()
    except Exception as ee:
        print(ee)
        time.sleep(0.5)


def sub_callback(topic, msg):
    msg = msg.decode()
    print(topic, msg)
    if "reboot" in msg or "reset" in msg:
        print("device rebooting......")
        mqtt_c.publish(mqtt_resp, node_name + "rebooting", retain=True)
        oled.fill(0)
        oled.text("rebooting...", 10, 20)
        oled.show()
        time.sleep(0.5)
        reset()

    try:
        mqtt_c.publish(mqtt_resp, json.dumps(data), retain=True)
    except Exception as e:
        print(e)


def read_sensor_data(timer=None):
    global led, data
    led.value(not led.value())
    data["temperature"], data["relative_humidity"] = sht30.read_data()
    data["illuminance"] = bh1750.read_data()
    sgp30.set_iaq_rel_humidity(rh=data["relative_humidity"], temp=data["temperature"])
    data["eco2"], data["tvoc"] = sgp30.read_data()
    data["pressure"] = bmp280.read_data()
    print("temperature: ", data["temperature"], " C")
    print("humidity: ", data["relative_humidity"], " %")
    print("illuminance: ", data["illuminance"], " lx")
    print("tVOC: ", data["tvoc"], " ppb")
    print("CO2eq: ", data["eco2"], " ppm")
    print("pressure: ", data["pressure"], " kPa")

    try:
        oled.fill(0)
        oled.text(wifi_info[0], 1, 0)
        oled.text(("%.2f C" % data["temperature"]) + ("  %d" % data["relative_humidity"] + " %"), 1, 10)
        oled.text("illu: ", 1, 20)
        oled.text("%.2f lx" % data["illuminance"], 42, 20)
        oled.text("tVOC:%d" % data["tvoc"], 2, 30)
        oled.text(" eCO2:%d" % data["eco2"], 2, 38)
        oled.text("pre: %.3f kPa" % data["pressure"], 2, 46)

        time_tuple = time.localtime()
        oled.text(
            "%02d" % time_tuple[1] + "-%02d" % time_tuple[2] + " %02d" % time_tuple[3] + ":" + "%02d" % time_tuple[4] + ":" + "%02d" % time_tuple[5], 1, 56)
        oled.show()
        data["timestamp"] = time.mktime(time_tuple)
    except Exception as e:
        print(e)


def send_data(timer=None):
    led.value(not led.value())
    try:
        mqtt_c.publish(mqtt_send, json.dumps(data), retain=True)
    except Exception as e:
        print(e)
    led.value(not led.value())


mqtt_c = MQTTClient(node_name, mqtt_host, mqtt_port)
mqtt_c.set_callback(sub_callback)
mqtt_c.connect()
mqtt_c.subscribe(mqtt_recv)

# 定时器 定时读取传感器数据   1s一次
timer1 = Timer(0)
timer1.init(period=1000, mode=Timer.PERIODIC, callback=read_sensor_data)
# 定时器 定时发布数据
timer2 = Timer(1)
timer2.init(period=2000, mode=Timer.PERIODIC, callback=send_data)
# 定时器 自动校时
timer3 = Timer(2)
timer3.init(period=1000 * 60 * 60 * 7, mode=Timer.PERIODIC, callback=sync_ntp)

try:
    while True:
        # mqtt_c.wait_msg()
        mqtt_c.check_msg()
        time.sleep(0.2)
finally:
    mqtt_c.disconnect()
