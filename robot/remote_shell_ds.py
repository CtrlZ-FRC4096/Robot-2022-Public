import time
import socket
from networktables import NetworkTables
# import readline # just importing this modifies behavior
import networktables

local_ip = socket.gethostbyname(socket.gethostname())

remote_ip = '10.40.96.2' if '40.96' in local_ip else '127.0.0.1'

# logging.basicConfig(level=logging.DEBUG)

NetworkTables.startClient(remote_ip)

while not NetworkTables.isConnected():
    pass

print("---------------------------")
print("---------------------------")
if remote_ip == '127.0.0.1':
    print("Connected to SIMULATOR")
else:
    print("Connected to ROBOT")
print("---------------------------")
print("You are now in a pseudo python shell within the robot.")
print("Your robot is located at 'robot' or at 'r'.")
print("Ex. 'robot.oi' is the OI object of your robot.")
print("---------------------------")
print("---------------------------")

table = NetworkTables.getTable("Remote Shell")

def r():
    user_input = input(">>> ")
    if user_input == "exit()":
        exit()
    user_input += f" T{time.time_ns():<20}"
    table.putString("stdin", user_input)
    # print("sent ->", user_input)

def pr(entry):
    print(entry.value.getRaw()[:-22])
    r()

table.getEntry("stdout").addListener(pr, networktables.NetworkTablesInstance.NotifyFlags.UPDATE)

r()

while True: pass
