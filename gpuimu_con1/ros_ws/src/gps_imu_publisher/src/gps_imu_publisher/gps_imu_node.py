import time
import serial
import re
import threading
import math
import rclpy
from rclpy.node import Node
from gps_imu_msgs.msg import GpsImuData

# --- 全局数据与锁 ---
acc = [0.0, 0.0, 0.0]
gyro = [0.0, 0.0, 0.0]
angle = [0.0, 0.0, 0.0]
utctime = lat = ulat = lon = ulon = numSv = msl = cogt = cogm = sog = kph = ''
gps_t = 0
lock = threading.Lock()

# --- GPS 串口初始化 ---
ser = serial.Serial("/dev/ttyUSB1", 9600)
if ser.isOpen():
    print("GPS Serial Opened! Baudrate=9600")
else:
    print("GPS Serial Open Failed!")

# --- IMU 读取参数 ---
buf_length = 11
RxBuff = [0]*buf_length
ACCData = [0.0]*8
GYROData = [0.0]*8
AngleData = [0.0]*8
FrameState = 0
CheckSum = 0
start = 0
data_length = 0

serimu = serial.Serial('/dev/ttyUSB2', 9600, timeout=0.5)

# --- GPS 读取与转换 ---
def gps84_to_gcj02(lat, lon):
    PI = 3.1415926535897932384626
    A = 6378245.0
    EE = 0.00669342162296594323
    
    def transformLat(x, y):
        ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
        ret += (20.0 * math.sin(y * PI) + 40.0 * math.sin(y / 3.0 * PI)) * 2.0 / 3.0
        ret += (160.0 * math.sin(y / 12.0 * PI) + 320 * math.sin(y * PI / 30.0)) * 2.0 / 3.0
        return ret
    
    def transformLon(x, y):
        ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
        ret += (20.0 * math.sin(x * PI) + 40.0 * math.sin(x / 3.0 * PI)) * 2.0 / 3.0
        ret += (150.0 * math.sin(x / 12.0 * PI) + 300.0 * math.sin(x / 30.0 * PI)) * 2.0 / 3.0
        return ret

    def outOfChina(lat, lon):
        if lon < 72.004 or lon > 137.8347:
            return True
        if lat < 0.8293 or lat > 55.8271:
            return True
        return False

    if outOfChina(lat, lon):
        return [lat, lon]
    
    dLat = transformLat(lon - 105.0, lat - 35.0)
    dLon = transformLon(lon - 105.0, lat - 35.0)
    radLat = lat / 180.0 * PI
    magic = math.sin(radLat)
    magic = 1 - EE * magic * magic
    sqrtMagic = math.sqrt(magic)
    dLat = (dLat * 180.0) / ((A * (1 - EE)) / (magic * sqrtMagic) * PI)
    dLon = (dLon * 180.0) / (A / sqrtMagic * math.cos(radLat) * PI)
    mgLat = lat + dLat
    mgLon = lon + dLon
    return [mgLat, mgLon]

# --- 数据转换函数 ---
def Convert_to_degrees(in_data1, in_data2):
    len_data1 = len(in_data1)
    str_data2 = "%05d" % int(in_data2)
    temp_data = int(in_data1)
    symbol = 1
    if temp_data < 0:
        symbol = -1
    degree = int(temp_data / 100.0)
    str_decimal = str(in_data1[len_data1-2]) + str(in_data1[len_data1-1]) + str(str_data2)
    f_degree = int(str_decimal)/60.0/100000.0
    return degree + f_degree if symbol > 0 else degree - f_degree

def GPS_read():
    global utctime, lat, ulat, lon, ulon, numSv, msl, cogt, cogm, sog, kph, gps_t
    if ser.inWaiting():
        if ser.read(1) == b'G' and ser.inWaiting() and ser.read(1) == b'N':
            choice = ser.read(1)
            if choice == b'G' and ser.inWaiting() and ser.read(1) == b'G' and ser.inWaiting() and ser.read(1) == b'A':
                GGA = ser.read(70)
                GGA_g = re.findall(r"\w+(?=,)|(?<=,)\w+", str(GGA))
                if len(GGA_g) >= 13:
                    with lock:
                        utctime = GGA_g[0]
                        lat = "%.8f" % Convert_to_degrees(str(GGA_g[2]), str(GGA_g[3]))
                        ulat = GGA_g[4]
                        lon = "%.8f" % Convert_to_degrees(str(GGA_g[5]), str(GGA_g[6]))
                        ulon = GGA_g[7]
                        numSv = GGA_g[9]
                        msl = GGA_g[12]+'.'+GGA_g[13]+GGA_g[14]
                        gps_t = 1
            elif choice == b'V' and ser.inWaiting() and ser.read(1) == b'T' and ser.inWaiting() and ser.read(1) == b'G':
                if gps_t == 1:
                    VTG = ser.read(40)
                    VTG_g = re.findall(r"\w+(?=,)|(?<=,)\w+", str(VTG))
                    with lock:
                        cogt = VTG_g[0]+'.'+VTG_g[1]
                        if VTG_g[3] == 'M':
                            cogm = '0.00'
                            sog = VTG_g[4]+'.'+VTG_g[5]
                            kph = VTG_g[7]+'.'+VTG_g[8]
                        else:
                            cogm = VTG_g[3]+'.'+VTG_g[4]
                            sog = VTG_g[6]+'.'+VTG_g[7]
                            kph = VTG_g[9]+'.'+VTG_g[10]

def GetDataDeal(list_buf):
    global acc, gyro, angle
    if list_buf[buf_length - 1] != CheckSum:
        return
    with lock:
        if list_buf[1] == 0x51:
            for i in range(6): ACCData[i] = list_buf[2+i]
            acc = list(get_acc(ACCData))
        elif list_buf[1] == 0x52:
            for i in range(6): GYROData[i] = list_buf[2+i]
            gyro = list(get_gyro(GYROData))
        elif list_buf[1] == 0x53:
            for i in range(6): AngleData[i] = list_buf[2+i]
            angle = list(get_angle(AngleData))

def DueData(inputdata):
    global start, CheckSum, data_length
    if inputdata == 0x55 and start == 0:
        start = 1
        data_length = 11
        CheckSum = 0
        for i in range(11):
            RxBuff[i] = 0
    if start == 1:
        CheckSum += inputdata
        RxBuff[buf_length - data_length] = inputdata
        data_length -= 1
        if data_length == 0:
            CheckSum = (CheckSum - inputdata) & 0xff
            start = 0
            GetDataDeal(RxBuff)

def get_acc(datahex):
    axl, axh, ayl, ayh, azl, azh = datahex[0:6]
    k = 16.0
    x = (axh << 8 | axl) / 32768.0 * k
    y = (ayh << 8 | ayl) / 32768.0 * k
    z = (azh << 8 | azl) / 32768.0 * k
    return x - 2*k if x >= k else x, y - 2*k if y >= k else y, z - 2*k if z >= k else z

def get_gyro(datahex):
    wxl, wxh, wyl, wyh, wzl, wzh = datahex[0:6]
    k = 2000.0
    x = (wxh << 8 | wxl) / 32768.0 * k
    y = (wyh << 8 | wyl) / 32768.0 * k
    z = (wzh << 8 | wzl) / 32768.0 * k
    return x - 2*k if x >= k else x, y - 2*k if y >= k else y, z - 2*k if z >= k else z

def get_angle(datahex):
    rxl, rxh, ryl, ryh, rzl, rzh = datahex[0:6]
    k = 180.0
    x = (rxh << 8 | rxl) / 32768.0 * k
    y = (ryh << 8 | ryl) / 32768.0 * k
    z = (rzh << 8 | rzl) / 32768.0 * k
    return x - 2*k if x >= k else x, y - 2*k if y >= k else y, z - 2*k if z >= k else z

def read_imu_loop():
    while True:
        RXdata = serimu.read(1)
        if RXdata:
            DueData(int(RXdata.hex(), 16))

def read_gps_loop():
    while True:
        GPS_read()

# --- ROS2 节点 ---
class GpsImuPublisher(Node):
    def __init__(self):
        super().__init__('gps_imu_node')
        self.publisher_ = self.create_publisher(GpsImuData, 'gps_imu_data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        threading.Thread(target=read_imu_loop, daemon=True).start()
        threading.Thread(target=read_gps_loop, daemon=True).start()

    def timer_callback(self):
        global lat, lon, ulat, ulon
        if lat != '' and lon != '':
            # 将 WGS-84 坐标转换为 GCJ-02
            lat_gcj02, lon_gcj02 = gps84_to_gcj02(float(lat), float(lon))
            
            # 将 lat_gcj02 和 lon_gcj02 转换为字符串，并将 ulat 和 ulon 拼接到它们后面
            lat_gcj02_str = "{:.8f}".format(lat_gcj02) + ulat  # 保留 8 位小数并添加 ulat
            lon_gcj02_str = "{:.8f}".format(lon_gcj02) + ulon  # 保留 8 位小数并添加 ulon
            
            with lock:
                msg = GpsImuData()
                msg.utc_time = utctime
                msg.latitude = lat_gcj02_str  # 将转换后的纬度作为字符串存储
                msg.longitude = lon_gcj02_str  # 将转换后的经度作为字符串存储
                msg.num_satellites = numSv
                msg.altitude = msl
                msg.heading_true = cogt
                msg.heading_magnetic = cogm
                msg.speed_knots = sog
                msg.speed_kmph = kph
                msg.accel_x = float(acc[0])
                msg.accel_y = float(acc[1])
                msg.accel_z = float(acc[2])
                msg.gyro_x = float(gyro[0])
                msg.gyro_y = float(gyro[1])
                msg.gyro_z = float(gyro[2])
                msg.roll = float(angle[0])
                msg.pitch = float(angle[1])
                msg.yaw = float(angle[2])
                self.publisher_.publish(msg)
                self.get_logger().info("Published GPS+IMU data")
        else:
            print("Warning: Invalid GPS data received, skipping conversion.")


def main(args=None):
    rclpy.init(args=args)
    node = GpsImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
