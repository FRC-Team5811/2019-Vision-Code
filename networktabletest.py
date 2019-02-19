import threading
from networktables import NetworkTables
import os

cond = threading.Condition()
notified = [False]


def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()


NetworkTables.initialize(server='10.58.11.2')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

# Insert your processing code here
print("Connected!")

sd = NetworkTables.getTable('SmartDashboard')

while True:
    print('left_area', sd.getNumber('left_area', 0))
    print('right_area', sd.getNumber('right_area', 0))
    print('total_area', sd.getNumber('total_area', 0))
    print('center_x', sd.getNumber('center_x', 0))
    print('center_y', sd.getNumber('center_y', 0))
    os.system('clear')