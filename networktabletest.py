import threading
from networktables import NetworkTables

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
    print(sd.getNumber('center_x', 5), end=" ")
    print(sd.getNumber('center_y', 5), end=" ")
    print(sd.getNumber('left_area', 5), end=" ")
    print(sd.getNumber('right_area', 5))