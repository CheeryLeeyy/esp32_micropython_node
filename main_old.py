from machine import RTC, reset, Timer, I2C, Pin, SoftI2C
import network
import time
import ssd1306
import ntptime

from GY302_BH1750 import BH1750_Sensor
from bmp280 import BMP280
from sgp30 import SGP30
from sht3x import SHT3x_Sensor


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
led = Pin(2, Pin.OUT)
led.value(1)

wifi_info = do_connect(ssid="Yang_Home_2.4G", password="Liyang123", led=led)
if not wifi_info:
    reset()

# 获取NTP时间
rtc = RTC()
ntptime.NTP_DELTA = 3155644800  # 可选 UTC+8偏移时间（秒），不设置就是UTC0
ntptime.host = 'ntp1.aliyun.com'  # 可选，ntp服务器，默认是"pool.ntp.org" 这里使用阿里服务器
try:
    ntptime.settime()
except:
    reset()
datetime = rtc.datetime()
print(datetime)



# 传感器初始化
### 数据字典
data = {
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
    # i2c = SoftI2C(scl=Pin(5), sda=Pin(18), freq=100000)
    # i2c = I2C(0, freq=100000)
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
    reset()



while True:
    led.value(not led.value())
    data["temperature"], data["relative_humidity"] = sht30.read_data()
    data["illuminance"] = bh1750.read_data()
    sgp30.set_iaq_rel_humidity(rh=data["relative_humidity"], temp=data["temperature"])
    data["eco2"], data["tvoc"] = sgp30.read_data()
    data["pressure"] = bmp280.read_data()
    print("temperature: ", data["temperature"], " C")
    print("humidity: ", data["relative_humidity"], " %")
    print("illuminance: ", data["illuminance"] , " lx")
    print("tVOC: ", data["tvoc"], " ppb")
    print("CO2eq: ", data["eco2"], " ppm")
    print("ssure: ", data["pressure"], " kPa")

    oled.fill(0)
    oled.text(wifi_info[0], 1, 0)
    # oled.text("temp: ", 1, 10)
    # oled.text("humd: ", 1, 20)
    oled.text(("%.2f C" % data["temperature"]) + (" %d" % data["relative_humidity"] + " %"), 42, 10)
    oled.text("illu: ", 1, 20)
    oled.text("%.2f lx" % data["illuminance"], 42, 20)
    # oled.text("%.2f C" % data["temperature"], 42, 10)
    # oled.text("%d" % data["relative_humidity"] + " %", 42, 20)
    # oled.text("tVOC: %d ppb" % tvoc, 1, 40)
    # oled.text("CO2eq: %d ppm" % co2, 2, 50)
    oled.text(("tVOC:%d" % data["tvoc"]) + (" eCO2:%d" % data["eco2"]), 0, 30)
    oled.text("pre: %.3f kPa" % data["pressure"], 0, 30)

    datetime = rtc.datetime()
    # oled.text("%04d" % datetime[0] + "-" + "%02d" % datetime[1] + "-" + "%02d" % datetime[2], 5, 46)
    # oled.text("%02d" % datetime[4] + ":" + "%02d" % datetime[5] + ":" + "%02d" % datetime[6], 5, 56)
    oled.text("%02d" % datetime[1] + ".%02d" % datetime[2] + " %02d" % datetime[4] + ":" + "%02d" % datetime[5] + ":" + "%02d" % datetime[6], 1, 56)
    oled.show()

    time.sleep(1)




