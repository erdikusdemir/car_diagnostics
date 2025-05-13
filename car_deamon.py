from apscheduler.schedulers.blocking import BlockingScheduler
import subprocess
from datetime import datetime, timedelta
import json
import time
import logging

import paho.mqtt.client as mqtt

import serial
import RPi.GPIO as GPIO
import board

from adafruit_ina219 import INA219
from adafruit_ssd1306 import SSD1306_I2C
from PIL import Image, ImageDraw


import obd

MQTT_BROKER = "192.168.1.5"
MQTT_PORT = 1883
MQTT_TOPIC = "sensors/"
MQTT_OBD_ERROR_TOPIC = "OBD_error/"
IOT_LOCATION = "ltedevice0"

PATH_GPS = "/dev/ttyUSB1"
PATH_OBD = "/dev/ttyUSB5"

MODE_SENSOR = True
MODE_OBD = True
MODE_GPIO = False
MODE_GPS = True
MODE_SYSTEM = True

DATA_INTERVAL = 60 # seconds



class OLEDLogHandler(logging.Handler):
    def __init__(self,screen_i2c,scheduler):
        super().__init__()
        # OLED display size
        self.WIDTH = 128
        self.HEIGHT = 64
        self.oled = SSD1306_I2C(self.WIDTH, self.HEIGHT, screen_i2c)

        # Clear the display
        self.oled.fill(0)
        self.oled.show()

        # Create an image buffer
        self.image = Image.new("1", (self.WIDTH, self.HEIGHT))
        self.draw = ImageDraw.Draw(self.image)
        self.LINE_HEIGHT = 16
        self.MAX_LINES = self.HEIGHT // self.LINE_HEIGHT
        self.lines = []
        self.scheduler = scheduler
        self.timeout = 5
        self.job_id = "clear_oled_display"

    def emit(self, record):
        msg = self.format(record)
        self.lines.append(msg)
        if len(self.lines) >  self.MAX_LINES:
            self.lines = self.lines[-self.MAX_LINES:]  # keep last N lines
        self.update_display()
        self.schedule_clear()

    def update_display(self):
        self.draw.rectangle((0, 0, self.WIDTH , self.HEIGHT ), outline=0, fill=0)
        for i, line in enumerate(self.lines):
            self.draw.text((0, i * self.LINE_HEIGHT), line, fill=255)
        self.oled.image(self.image)
        self.oled.show()

    def clear_display(self):
        self.oled.fill(0)
        self.oled.show()
        self.lines = []

    def schedule_clear(self):
        # Remove existing scheduled job if any
        try:
            self.scheduler.remove_job(self.job_id)
        except Exception:
            pass  # Ignore if job doesn't exist

        # Schedule a new one 5 seconds from now
        self.scheduler.add_job(self.clear_display, 'date',
                               run_date=datetime.now() + timedelta(seconds=self.timeout),
                               id=self.job_id, replace_existing=True)



#-----------------------------------------------------------------------------------

# Module1: internal meausurements.
class SystemModule:
    def __init__(self):
        self.logger_oled = logging.getLogger("oled")
            
    def get_cpu_temp_macos(self):
        try:
            output = subprocess.check_output(["osx-cpu-temp"]).decode("utf-8").strip()
            temp_value = float(output.replace("°C", ""))
            return [datetime.now().isoformat(), IOT_LOCATION, "CPU", "Temperature", round(temp_value, 2), "°C"] 
        except Exception as e:
            return None

    def get_cpu_temperature(self):
        """Reads CPU temperature in Celsius."""
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            temp_str = f.readline()
            #return float(temp_str) / 1000.0  # Convert millidegrees to degrees

            return [datetime.now().isoformat(), IOT_LOCATION, "CPU", "Temperature", float(temp_str) / 1000.0, "°C"]

    def get_internal_measurements(self):
        payload = {"Columns": ["time", "location", "device", "measurement", "value", "unit"],
                        "Values": []}
        payload["Values"].append(self.get_cpu_temperature())

        # Send time    
        if len(payload) > 0:
            publish_data(payload)
            self.logger_oled.debug(f"SysInfo: Sent")
        else:
            print("No data to send.")
            self.logger_oled.debug(f"SysInfo: No Data")



# Module2: Digital IO.
class GPIOModule():
    GPIO = GPIO
    def __init__(self):
        self.logger_oled = logging.getLogger("oled")
        # initilize GPIO:
        PINS_LIST = [4, 9, 10, 11, 17, 22, 27]
        self.GPIO.setmode(self.GPIO.BCM)
        for pin in PINS_LIST:
            self.GPIO.setup(pin, self.GPIO.IN, pull_up_down=self.GPIO.PUD_UP)


    def get_pin_states(self):
        """Returns a dictionary of GPIO pin states."""
        #PINS_LIST = [2, 3, 4, 17, 27, 22, 10, 9, 11]
        PINS_LIST = {'Over_Weight':4, 'Hand_Brake':9, 'Low_Oil':10, 'Low_Air':11 ,'17':17, '22':22,'27':27}
        payload = {"Columns": ["time", "location", "device", "measurement", "value", "unit"],
                        "Values": []}
        for key, value in PINS_LIST.items():
            payload["Values"].append([datetime.now().isoformat(), IOT_LOCATION, 'Digital_IO', key, self.GPIO.input(value), "boolean"])
        # Send time    
        if len(payload) > 0:
            publish_data(payload)
            self.logger_oled.debug(f"GPIO: Sent")
        else:
            print("No data to send.")
            self.logger_oled.debug(f"GPIO: No Data")
        self.GPIO.cleanup()

# Module3: Analog IO.

# pip3 install adafruit-circuitpython-ina219 adafruit-circuitpython-tca9548a
# sudo apt install python3-smbus
#
class I2cModule:
    MUX_ADDRESS = 0x70
    MUX_CHANNELS = {0: b'\x01', 1: b'\x02', 2: b'\x04', 3: b'\x08', 4: b'\x10', 5: b'\x20', 6: b'\x40', 7: b'\x80'}
    MUX_OFF =  b'\x00' 
    INA219_ADDRESSES = {0: 0x40, 1: 0x41, 2: 0x44, 3: 0x45}
    def __init__(self):
        self.logger_oled = logging.getLogger("oled")
        self.i2c_bus = board.I2C()
        self.init_i2c_devices()


    def init_i2c_devices(self):
        """Initialize I2C bus."""
        self.ina219_objects = []
        for channel, channel_address in self.MUX_CHANNELS.items():
            self.i2c_bus.writeto(self.MUX_ADDRESS, channel_address)
            for port, ina_219_address in self.INA219_ADDRESSES.items():
                #print(channel, port, ina_219_address)
                try:
                    self.ina219_objects.append([channel, port,INA219(self.i2c_bus, ina_219_address)])
                    #print("Found INA219 at address 0x%02X" % ina_219_address) 
                except ValueError:
                    #print("No INA219 found at address 0x%02X" % ina_219_address)
                    continue  # If no INA219 found, continue to next address
                except OSError:
                    #print("No I2C device found at address 0x%02X" % ina_219_address)
                    continue  # If no I2C device found, continue to next address
                # If no INA219 found, exit
        self.logger_oled.debug(f"Sensor: {len(self.ina219_objects)} found.")
        return

    # Main loop to read all sensors
    def read_ina219_sensors(self):
        """Read and print data from all INA219 sensors."""
        payload = {"Columns": ["time", "location", "device", "measurement", "value", "unit"],
                    "Values": []}
        # Read and print data from each sensor
        for channel, port, ina219 in self.ina219_objects: # one channel takes 0.025 seconds.
            self.i2c_bus.writeto(self.MUX_ADDRESS, self.MUX_CHANNELS[channel])
            bus_voltage = ina219.bus_voltage  # voltage on V- (load side)
            shunt_voltage = ina219.shunt_voltage  # voltage between V+ and V- across the shunt
            current = ina219.current  # current in mA
            power = ina219.power  # power in watts

            # Check internal calculations haven't overflowed (doesn't detect ADC overflows)
            if ina219.overflow:
                print("Internal Math Overflow Detected!")
                print("")

            time_stamp = datetime.now().isoformat()
            payload["Values"].append([
                time_stamp,  # Timestamp
                IOT_LOCATION,  # Location
                f"Analog_{channel}_{port}",  # Device
                "bus_voltage",  # Measurement type
                bus_voltage,  # Value
                "Volts" # Unit
            ])
            payload["Values"].append(
            [
                time_stamp,  # Timestamp
                IOT_LOCATION,  # Location
                f"Analog_{channel}_{port}",  # Device
                "shunt_voltage",  # Measurement type
                shunt_voltage,  # Value
                "mVolts" # Unit
            ])
            payload["Values"].append(
            [
                time_stamp,  # Timestamp
                IOT_LOCATION,  # Location
                f"Analog_{channel}_{port}",  # Device
                "current",  # Measurement type
                current,  # Value
                "mAmps" # Unit
            ])
            payload["Values"].append(
            [
                time_stamp,  # Timestamp
                IOT_LOCATION,  # Location
                f"Analog_{channel}_{port}",  # Device
                "power",  # Measurement type
                power,  # Value
                "mWatts" # Unit
            ])
            
        # Send time    
        if len(payload) > 0:
            publish_data(payload)
            self.logger_oled.debug(f"Sensor: Sent")
        else:
            print("No data to send.")
            self.logger_oled.debug(f"Sensor: No Data")
        return

# Module4: GPS measurements.
class GPSModule:
    def __init__(self, path_gps):
        self.logger_oled = logging.getLogger("oled")
        # Initialize serial port for GPS
        try:
            self.serial_gps = serial.Serial(path_gps,9600)
            self.serial_gps.flushInput()
            return
        except serial.SerialException as e:
            print(f"Error initializing GPS: {e}")
            self.logger_oled.error("GPS: Conn Error")
            return None


    def get_gps_coordinates(self):
        payload = {"Columns": ["time", "location", "device", "measurement", "value", "unit"],
                    "Values": []}

        while True:
            rec_buff = self.serial_gps.readline().decode(errors='ignore').strip()
            if rec_buff.startswith('$GPGGA'): #or rec_buff.startswith('$GPRMC'):
                parts = rec_buff.replace("$GPGGA,", "").split(',')
                if len(parts) < 8 or not parts[0] or not parts[2]:
                    print("GPS data not available")
                    self.logger_oled.error("GPS: No Data from GPS")
                    break

                time_stamp = datetime.now().isoformat()
                # Latitude
                lat_raw = parts[1]
                lat_deg = int(lat_raw[:2])
                lat_min = float(lat_raw[2:])
                lat = lat_deg + lat_min / 60.0
                if parts[2] == 'S':
                    lat = -lat

                payload["Values"].append([
                    time_stamp,  # Timestamp
                    IOT_LOCATION,  # Location
                    "GPS",  # Device
                    "latitude",  # Measurement type
                    round(lat, 8),  # Value
                    "Degrees" # Unit
                ])
                # Longitude
                lon_raw = parts[3]
                lon_deg = int(lon_raw[:3])
                lon_min = float(lon_raw[3:])
                lon = lon_deg + lon_min / 60.0
                if parts[4] == 'W':
                    lon = -lon

                payload["Values"].append(
                [
                    time_stamp,  # Timestamp
                    IOT_LOCATION,  # Location
                    "GPS",  # Device
                    "longitude",  # Measurement type
                    round(lon, 8),  # Value
                    "Degrees" # Unit
                ])
                fix_quality = int(parts[5]) if parts[5] else 0
                payload["Values"].append(
                [
                    time_stamp,  # Timestamp
                    IOT_LOCATION,  # Location
                    "GPS",  # Device
                    "fix_quality",  # Measurement type
                    fix_quality,  # Value
                    "number" # Unit
                ])
                satellites = int(parts[6]) if parts[6] else None
                payload["Values"].append(
                [
                    time_stamp,  # Timestamp
                    IOT_LOCATION,  # Location
                    "GPS",  # Device
                    "satellites",  # Measurement type
                    satellites,  # Value
                    "number" # Unit
                ])
                altitude = float(parts[8]) if parts[8] else None
                payload["Values"].append(
                [
                    time_stamp,  # Timestamp
                    IOT_LOCATION,  # Location
                    "GPS",  # Device
                    "altitude",  # Measurement type
                    altitude,  # Value
                    "meters" # Unit
                ])
                speed_knots = float(parts[7]) if parts[7] else 0.0
                speed_kmph = round(speed_knots * 1.852, 2)
                payload["Values"].append(
                [
                    time_stamp,  # Timestamp
                    IOT_LOCATION,  # Location
                    "GPS",  # Device
                    "speed",  # Measurement type
                    speed_kmph,  # Value
                    "kmph" # Unit
                ])
                # Date and Time
                date_str = today_date = datetime.now().date() # DDMMYY
                time_str = parts[0] # HHMMSS.s
                dt = datetime.strptime(f"{date_str} {time_str.split('.')[0]}", "%Y-%m-%d %H%M%S")
                timestamp_utc = dt.strftime("%Y-%m-%dT%H:%M:%SZ")
                '''
                payload["Values"].append(
                [
                    time_stamp,  # Timestamp
                    IOT_LOCATION,  # Location
                    "GPS",  # Device
                    "time",  # Measurement type
                    timestamp_utc,  # Value
                    "UTC" # Unit
                ])
                '''
                break


        # Send time    
        if len(payload["Values"]) > 0:
            publish_data(payload)
            self.logger_oled.debug("GPS: Sent")
        else:
            print("No GPS data is fetched.")
            self.logger_oled.debug("GPS: No Data")



# Module5: ODBII port measurements.
class OBDModule:
    def __init__(self, path_obd):
        self.logger_oled = logging.getLogger("oled")
        self.obd_conn = obd.OBD(path_obd) # connect to a specific port
        #ports = obd.scan_serial()      # return list of valid USB or RF ports
        #print (ports)                    # ['/dev/ttyUSB0', '/dev/ttyUSB1']
        #connection = obd.OBD(ports[0])
        print('elm status', self.obd_conn.status())
        if not self.obd_conn.is_connected():
            print("No connection to OBD-II adapter.")
            self.logger_oled.error("No connection to OBD-II adapter.")
            return
        return

    def get_obd_data(self):
        # get the current speed
        COMMAND_LIST = {'SPEED': obd.commands.SPEED, 
                        'RPM':obd.commands.RPM,
                        'Fuel_Level':obd.commands.FUEL_LEVEL,
                        'Oil_Temperature':obd.commands.OIL_TEMP,
                        }
        measurement = []

        for name, cmd in COMMAND_LIST.items():
            response = self.obd_conn.query(cmd) # send the command, and parse the response
            if response.is_null():
                print("No data received from OBD-II adapter.")
                continue
            time_stamp = datetime.now().isoformat()
            measurement.append([
                time_stamp, IOT_LOCATION, "ODB",name, response.value[0], response.value[1] 
            ])
        print(measurement)
        self.logger_oled.debug("OBD: Some Data")
        return measurement

    def get_obd_error_codes(self):
        response = self.obd_conn.query(obd.commands.GET_DTC) # send the command, and parse the response
        if response.is_null():
            print("No data received from OBD-II adapter.")
            return False
        payload = {"Columns": ["time", "location", "device", "measurement", "value"],
                    "Values": [datetime.now().isoformat(), IOT_LOCATION, "OBD","Error Codes", response.value[0], response.value[1] ]}
        message = json.dumps(payload)
        client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        client.connect(MQTT_BROKER, MQTT_PORT)
        client.publish(MQTT_OBD_ERROR_TOPIC, message)
        client.disconnect()
        print(f"Sent: {message}")
        self.logger_oled.debug("OBD: Error Codes")
                
        

# main code -------
def publish_data(payload):
    message = json.dumps(payload)
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.connect(MQTT_BROKER, MQTT_PORT)
    client.publish(MQTT_TOPIC, message)
    client.disconnect()
    print(f"Sent: {message}")

def main():
    scheduler = BlockingScheduler()
    i2c_obj = I2cModule()
    logger_oled = logging.getLogger("oled")
    logger_oled.setLevel(logging.DEBUG)

    logging_handler_obj = OLEDLogHandler(i2c_obj.i2c_bus,scheduler)
    formatter = logging.Formatter('%(levelname).1s: %(message)s')
    logging_handler_obj.setFormatter(formatter)
    logger_oled.addHandler(logging_handler_obj)

    logger_oled.info("Booting up...")
    if MODE_GPIO:
        gpio_obj = GPIOModule()
        logger_oled.info("GPIO: Started.")
    if MODE_GPS:
        gps_obj = GPSModule(PATH_GPS)
        logger_oled.info("GPS: Started.")
    if MODE_OBD:
        obd_obj = OBDModule(PATH_OBD)
        logger_oled.info("OBD: Started.")
    if MODE_SYSTEM:
        system_obj = SystemModule()
        logger_oled.info("System Info: Started.")

    if MODE_SYSTEM:
        system_obj.get_internal_measurements()
        scheduler.add_job(system_obj.get_internal_measurements, 'interval', seconds=DATA_INTERVAL)
    if MODE_GPIO:
        gpio_obj.get_pin_states()
        scheduler.add_job(gpio_obj.get_pin_states, 'interval', seconds=DATA_INTERVAL)
    if MODE_SENSOR:
        i2c_obj.read_ina219_sensors()
        logger_oled.info("Sensors: Started.")
        scheduler.add_job(i2c_obj.read_ina219_sensors, 'interval', seconds=DATA_INTERVAL)
    if MODE_GPS:    scheduler.add_job(gps_obj.get_gps_coordinates, 'interval', seconds=DATA_INTERVAL)
    if MODE_OBD:
        obd_obj.get_obd_data()
        scheduler.add_job(obd_obj.get_obd_data, 'interval', seconds=DATA_INTERVAL)
    #if MODE_OBD:    obd_obj.get_obd_error_codes()#this only works if the ignition on?
    #if MODE_OBD:    scheduler.add_job(obd_obj.get_obd_error_codes, 'interval', days=1, args=[obd_conn])
    try:
        print("Deamon started. Press Ctrl+C to exit.")
        scheduler.start()
    except (KeyboardInterrupt, SystemExit):
        print("\n[INFO] Scheduler stopped. Exiting.")
    return

if __name__ == "__main__":
    main()


    
''''
    logger.info("System initialized")
    logger.warning("Battery low")
    logger.error("Sensor failure")
    logger.debug("Debug: temp=21.4")
'''
