""" EtherCat Communication with motor, receives instructions from talker

** Listens for instructions from the talker (e. g. setClock(12))
** Sets up an etherCAT connection with the motor's hardware interface
** Fetches motor status from it

** Fetches hall sensor state

"""
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import pysoem
import ctypes

import time

LOGGING = False

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if LOGGING:
            self.get_logger().info('I heard: "%s"' % msg.data)

def init_etherCAT():
    master = pysoem.Master()

    adapters = pysoem.find_adapters()
    for i, adapter in enumerate(adapters):
        print(f"Adapter {i}: {adapter.name}")
        print(f"  Description: {adapter.desc}")

    master.open("enx00e04c1c1328")

    if master.config_init() > 0:
        device = master.slaves[0]
        print(f"device <{device.name}> has been found")
    else:
        print("device not found")

    
    return device

def home(device):
    
    #Enable Voltage
    device.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(6)))
    time.sleep(0.5)
    resp = ctypes.c_uint16.from_buffer_copy(device.sdo_read(0x6041, 0)).value
    #print(hex(resp))
    
    #Switch on
    device.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(7)))
    time.sleep(0.5)
    resp = ctypes.c_uint16.from_buffer_copy(device.sdo_read(0x6041, 0)).value
    #print(hex(resp))
    
    #Enable Operation
    device.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(15)))
    time.sleep(0.5)
    resp = ctypes.c_uint16.from_buffer_copy(device.sdo_read(0x6041, 0)).value
    #print(hex(resp))
    
    #Mode of Operation = Homing Mode
    device.sdo_write(0x6060, 0, bytes(ctypes.c_uint8(6)))
    time.sleep(0.5)
    resp = ctypes.c_uint8.from_buffer_copy(device.sdo_read(0x6041, 0)).value
    print("homing mode")
    print(hex(resp))

    #Homing Offset. It does not make the drive phsically move a more distance automatically, it just reset the internal coordinates.
    device.sdo_write(0x607C, 0, bytes(ctypes.c_int32(0)))
    time.sleep(0.5)
    resp = ctypes.c_int32.from_buffer_copy(device.sdo_read(0x607C, 0)).value
    print("Offset = ", resp)
    
    #Encoder Revolution 4096
    #resp = ctypes.c_int32.from_buffer_copy(device.sdo_read(0x608F, 1)).value
    #print("resolution")
    #print(resp)
    #Gear ratio 1:1
    #resp = ctypes.c_int32.from_buffer_copy(device.sdo_read(0x6091, 1)).value
    #print("Gear ratio1")
    #print(resp)
    #resp = ctypes.c_int32.from_buffer_copy(device.sdo_read(0x6091, 2)).value
    #print("Gear ratio2")
    #print(resp)
    
    #Homing Method
    device.sdo_write(0x6098, 0, bytes(ctypes.c_int8(23)))
    time.sleep(0.5)
    
    
    #Homing Speed
    device.sdo_write(0x6099, 2, bytes(ctypes.c_uint32(2000)))
    time.sleep(0.5)
    
    #Homing Acceleration
    
    #Homing Operation Start
    device.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(31)))
    #Homing Mode Statusword 0b0011 0100 0000 0000=0x3400
    Mask=0x3400
    resp = ctypes.c_uint16.from_buffer_copy(device.sdo_read(0x6041, 0)).value
    print(hex(resp))
    resp=resp & Mask
    #resp==0x2400 means that now the drive has already been at the home position
    if(resp!=0x2400):   
        while(resp!=0x1400):
            resp = ctypes.c_uint16.from_buffer_copy(device.sdo_read(0x6041, 0)).value
            resp=resp & Mask
            
    setTarget(device, -15)        
    print("Homing is finished.")
    
    
    

def setTarget(device: pysoem.CdefCoeObjectEntry, angle: float):
    # ToDo: convert to correct type e. g. ctypes
    print(f"setting target {angle} starts")
    #Enable Voltage
    device.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(6)))
    time.sleep(0.5)
    resp = ctypes.c_uint16.from_buffer_copy(device.sdo_read(0x6041, 0)).value # we expect 1
    #print(hex(resp))
    
    #Switch on
    device.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(7)))
    time.sleep(0.5)
    resp = ctypes.c_uint16.from_buffer_copy(device.sdo_read(0x6041, 0)).value
    #print(hex(resp))
    
    #Enable Operation
    device.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(15)))
    time.sleep(0.5)
    resp = ctypes.c_uint16.from_buffer_copy(device.sdo_read(0x6041, 0)).value
    #print(hex(resp))
    #print("operation enabled")
    
    #Profile Position Mode
    device.sdo_write(0x6060, 0, bytes(ctypes.c_uint8(1)))
    time.sleep(0.5)
    resp = ctypes.c_uint8.from_buffer_copy(device.sdo_read(0x6061, 0)).value
    resp_abs = ctypes.c_uint8.from_buffer_copy(device.sdo_read(0x6041, 0)).value
    #print(hex(resp))
    #print(bin(resp_abs)) # bit 6 tells abs 0 / rel 1
    
    #Write Target Position
    #steps = (int)(angle / 360 * 1325000)
    #print(f"steps: {steps} angle: {angle}")
    steps=int(angle/360*1325000)
    device.sdo_write(0x607A, 0, bytes(ctypes.c_int32(steps)))
    time.sleep(0.5)
    
    #Start Motion
    device.sdo_write(0x6040, 0, bytes(ctypes.c_int16(63)))
    time.sleep(0.5)
    resp = ctypes.c_int16.from_buffer_copy(device.sdo_read(0x6041, 0)).value
    print(hex(resp))
    print(f"set position {angle} finished")
    
def calibrate():
    # ToDo: find out how to get hall sensor information
    pass


def main(args=None):
    rclpy.init(args=args)

    device = init_etherCAT()
    
    minimal_subscriber = MinimalSubscriber()
    
    #resp = ctypes.c_uint8.from_buffer_copy(device.sdo_read(0x6091, 2)).value
    #print(hex(resp))
    #time.sleep(0.5)
    
    #setTarget(device, 3*30)
    #print("waiting...")
    #time.sleep(2)
    home(device)
    setTarget(device, 30)
    home(device)
    setTarget(device, 15)
    #time.sleep(2)
    #setTarget(device, 390)
    #time.sleep(10)
    
    
    
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

