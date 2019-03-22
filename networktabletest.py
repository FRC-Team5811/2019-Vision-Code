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


NetworkTables.initialize(server='roboRIO-5811-FRC.local')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

# Insert your processing code here
print("Connected!")

sd = NetworkTables.getTable('SmartDashboard')

sd.putNumber('left_select_mode', 0)
# sd.putNumber('stream_vision', 0)


while True:
    # print('left_area', sd.getNumber('left_area', 0))
    # print('right_area', sd.getNumber('right_area', 0))
    # print('total_area', sd.getNumber('total_area', 0))
    # print('difference_area', sd.getNumber('difference_area', 0))
    # print('offset', sd.getNumber('offset', 0))
    # print('center_x', sd.getNumber('center_x', 0))
    # print('center_y', sd.getNumber('center_y', 0))

    # print('loop_rate', sd.getNumber('loop_rate', 0))
    # print('left_select_mode', sd.getNumber('left_select_mode', 0))
    # os.system('clear')

    stream = float(input("stream_vision state: "))
    sd.putNumber('stream_vision', stream)
