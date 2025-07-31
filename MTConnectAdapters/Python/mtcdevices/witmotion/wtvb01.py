"""
MTConnect Adapter for the WTVB01-485 vibration sensor from WitMotion

Uses RS485 Modbus RTU for communication
"""
import time
from ..mtcdevice import MTCDevice
from .device_model import DeviceModel
from adapters.mtcadapter import ImproperlyConfigured


class WTVB01(MTCDevice):

    # default factory configuration
    baud_rate = 9600
    sensor_address = 0x50
    port = "COM6"

    # MTConnect IDs
    # If set to None, will not return the value to the Agent
    temperature_id = 'temp'
    #acceleration
    acc_x_id = 'acc_x'
    acc_y_id = 'acc_y'
    acc_z_id = 'acc_z'
    #velocity
    vx_id = 'vx'
    vy_id = 'vy'
    vz_id = 'vz'
    #angles
    ax_id = 'angle_x'
    ay_id = 'angle_y'
    az_id = 'angle_z'
    #displacements
    dx_id = 'dx'
    dy_id = 'dy'
    dz_id = 'dz'
    #frequencies
    hx_id = 'hx'
    hy_id = 'hy'
    hz_id = 'hz'

    DEBUG = False

    # sensor registers
    ACCX_REGISTER = 52
    VX_REGISTER = 58
    AX_REGISTER = 61
    TEMP_REGISTER = 64
    DX_REGISTER = 65
    HZX_REGISTER = 68

    __available__ = 1

    def __init__(self):
        # Configuration validations
        if self.port is None:
            raise ImproperlyConfigured("WTVB01 requires the attribute 'port' to be defined")
        print("============================================================")
        print("MTConnect Adapter for WTVB01-485 vibration sensor")
        print("(c) Concordia University DemoFactory 2024")
        print("============================================================")
        print(f"USB port: {self.port}")

        self.sensor = DeviceModel("WTVB01-485", self.port, self.baud_rate, self.sensor_address)
        self.sensor.openDevice()
        self.sensor.startLoopRead()
        time.sleep(0.5)

        if self.sensor.isOpen:
            print("Sensor connected")
        else:
            print(f"The device connected on {self.port} is not a WTVB01 sensor")

    @property
    def name(self) -> str:
        """Device name"""
        return "WTVB01-485"
    
    def serialNumber(self) -> str:
        """Device serial number."""
        return "WT202405140267"
    
    def on_connect(self):
        """Data sent on first connection."""
        return {
            "avail": "AVAILABLE",
        }

    def health_check(self)-> bool:
        """
        Return health status of device connection
        """
        try:
            self.temperature()
            self.__available__ = 1
            return True
        except IOError as e:
            print(f"Health check failed: {e}")
            self.__available__ = 0
            return False

    def manufacturer(self)-> str:
        """ WTVB01-485 manufacturer """
        return 'WitMotion'

    def temperature(self):
        """
        Read temperature in degree Celcius
        """
        return self.sensor.get(str(self.TEMP_REGISTER))

    def displacements(self):
        """
        Read displacements in microns
        """
        disp_microns =  [self.sensor.get(str(self.DX_REGISTER)), self.sensor.get(str(self.DX_REGISTER+1)), self.sensor.get(str(self.DX_REGISTER+2))]
        return [disp / 1000.0 for disp in disp_microns]  # Convert to mm

    def vibration_frequencies(self):
        """
        Read vibration frequencies in Hz
        """
        return [self.sensor.get(str(self.HZX_REGISTER)), self.sensor.get(str(self.HZX_REGISTER+1)), self.sensor.get(str(self.HZX_REGISTER+2))]

    def angles(self):
        """
        Read angles in degrees
        """
        return [self.sensor.get(str(self.AX_REGISTER)), self.sensor.get(str(self.AX_REGISTER+1)), self.sensor.get(str(self.AX_REGISTER+2))]
    
    def velocity(self):
        """ Read velocity in mm/s """
        return [self.sensor.get(str(self.VX_REGISTER)), self.sensor.get(str(self.VX_REGISTER+1)), self.sensor.get(str(self.VX_REGISTER+2))]
    
    def acceleration(self):
        """ Read acceleration in g """ 
        acc_g = [self.sensor.get(str(self.ACCX_REGISTER)), self.sensor.get(str(self.ACCX_REGISTER+1)), self.sensor.get(str(self.ACCX_REGISTER+2))]
        return [acc*9806.65 for acc in acc_g]  # Convert to mm/s^2

    def read_data(self):
        """
        Read and return device data
        """
        try:
            temp = self.temperature()
            disp = self.displacements()
            freq = self.vibration_frequencies()
            velocity = self.velocity()
            angles = self.angles()
        except IOError as e:
            print(f"Error reading data: {e}")
            self.__available__ = 0
            return {'avail': 'UNAVAILABLE'}

        if self.__available__ == 0:
            if self.health_check():
                return {'avail': 'AVAILABLE'}
            
            
        return {
            key: value
            for key, value in {
                self.temperature_id: temp,
                self.dx_id: disp[0],
                self.dy_id: disp[1],
                self.dz_id: disp[2],
                self.hx_id: freq[0],
                self.hy_id: freq[1],
                self.hz_id: freq[2],
                self.vx_id: velocity[0],
                self.vy_id: velocity[1],
                self.vz_id: velocity[2],
                self.ax_id: angles[0],
                self.ay_id: angles[1],
                self.az_id: angles[2],
                self.acc_x_id: self.acceleration()[0],
                self.acc_y_id: self.acceleration()[1],
                self.acc_z_id: self.acceleration()[2]
            }.items()
            if key is not None
        }