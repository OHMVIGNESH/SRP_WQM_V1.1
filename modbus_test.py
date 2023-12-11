import time
import json
import datetime
import uuid
mac_id = hex(uuid.getnode())
from pymodbus.constants import Endian
from pymodbus.constants import Defaults
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.transaction import ModbusRtuFramer
from pymodbus.payload import BinaryPayloadBuilder
from paho.mqtt import client as mqtt_client
global temprature 
global turbidity

# set Modbus defaults
SERIAL = '/dev/ttyUSB0'
BAUD = 9600
Defaults.UnitId = 1
Defaults.Retries = 5
counter = 1
client = ModbusClient(method='rtu', port=SERIAL, stopbits=1, bytesize=8, timeout=5, baudrate=BAUD, parity='N')
connection = client.connect()
print(bool(connection))
def on_connect(client, userdata, flags, rc):
    print(rc)
    if rc == 0:
        print(rc)
        clientmqtt.subscribe("SRP/MAX/SUB")
        global Connected      
        Connected = True    
    else:
        print("Connection failed")
Connected = False  

#mqtt connection
broker_address= "broker.emqx.io"
port = 1883
clientmqtt = mqtt_client.Client("srp/max")
clientmqtt.on_connect= on_connect
mqttstatus = clientmqtt.connect(broker_address, port)
clientmqtt.loop_start()
while Connected != True: 
    time.sleep(0.1)



try:
    while True:
        temp = client.read_input_registers(address=2, count=2, unit=1)
        temprature = BinaryPayloadDecoder.fromRegisters(temp.registers, Endian.Big, wordorder=Endian.Little)
        temprature=((temprature.decode_32bit_float()))
        time.sleep(1)
        trub = client.read_input_registers(address=4, count=2, unit=1)
        turbidity = BinaryPayloadDecoder.fromRegisters(trub.registers, Endian.Big, wordorder=Endian.Little)
        turbidity=((turbidity.decode_32bit_float()))
        time.sleep(1)
        current_datetime = datetime.datetime.now()
        current_datetime_str = current_datetime.strftime("%Y-%m-%d %H:%M:%S")
        print("Mac_id : " +str(mac_id)+" , Time:"+str(current_datetime_str) +" , Tempratuer : " + str(round(temprature,2)) + " , Trubidity : "+ str(round(turbidity,2)))
        data = {
        'Macid': str(mac_id),
        'Time': str(current_datetime_str),
        'parameter1': str(temprature),
        'parameter2': str(turbidity),
        'parameter3':"0",
        'parameter4':"0",
        'parameter5':"0"
        }
     #json create
  #"Macid": "94b555acf1b0",
  #"Time": "16/12/2022,12:31:12",
  #"parameter1": 332.5039673,
  #"parameter2": 0,
  #"parameter3": 332.5039673,
  #"parameter4": 997.5119019,
  #"parameter5": 1
        sensor_json_data = json.dumps(data, indent=4)
        print(sensor_json_data)
        #mqtt data send
        mqtt_data_send = clientmqtt.publish("MAX/SRP",sensor_json_data)
        status = mqtt_data_send[0]
        if status == 0:
            print("Sended")
        else:
            print("Failed ")
   
except:
    print("error")

    
