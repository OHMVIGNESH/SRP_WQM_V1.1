import logging
import logging.handlers

import time
import serial
import json
import os 
import datetime
import uuid
import traceback
from pymodbus.constants import Endian
from pymodbus.constants import Defaults
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.transaction import ModbusRtuFramer
from paho.mqtt import client as mqtt_client
from sympy import symbols, Eq, solve, ln
#import RPi.GPIO as GPIO
#GPIO.setmode(GPIO.BCM)
#led_pin = 18
#GPIO.setup(led_pin, GPIO.OUT)
log_file_path = os.environ.get("LOGFILE", "/home/maxuser/Desktop/SRp/SRP_WQM/SRP_log.log")

# Create a formatter that includes date and time
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

# Create a WatchedFileHandler for the log file
handler = logging.handlers.WatchedFileHandler(log_file_path)
handler.setFormatter(formatter)

# Get the root logger and set its level
root = logging.getLogger()
root.setLevel(os.environ.get("LOGLEVEL", "INFO"))

# Add the handler to the root logger
root.addHandler(handler)

# Create a RotatingFileHandler with max log size and backup count
max_log_size = 10 * 1024 * 1024  # 10 MB
file_handler = logging.handlers.RotatingFileHandler(log_file_path, maxBytes=max_log_size, backupCount=5)
file_handler.setFormatter(formatter)
root.addHandler(file_handler)

# Example log messages
root.debug("This is a debug message")
root.info("This is an info message")
root.warning("This is a warning message")
root.error("This is an error message")
root.critical("This is a critical message")
# Declare global variables
Macid = None
Time = None
Resid = None
Typeid = None
cal_1 = None
cal_2 = None
cal_3 = None
cal_4 = None
Th_1 = None
Th_2 = None
Th_3 = None
Th_4 = None
delay = None
ssid = None
pass_val = None
Broker_id = None
Port_id = None
Config_topic = None
Livedata_topic = None
Res_topic = None
Alert_topic = None
global temprature
global turbidity
global tds_value
global ts_value
global turbidity_cal
global tds_cal
global ts_cal
global tss_cal

def calibrate_sensor(sensor_value, max_voltage_sensor=5, max_tds=10000):
    # Ensure the sensor value is within the valid range
    sensor_value = max(0, min(sensor_value, max_voltage_sensor))

    # Perform linear mapping
    calibrated_tds = (sensor_value / max_voltage_sensor) * max_tds

    return calibrated_tds


def load_config_data():
    
    global Macid, Time, Resid, Typeid, cal_1, cal_2, cal_3, cal_4, Th_1, Th_2, Th_3, Th_4, delay, ssid, pass_val, Broker_id, Port_id, Config_topic, Livedata_topic, Res_topic, Alert_topic
    config_file = open("Config.json","r")
    data = json.load(config_file)
    # Store each data element in a variable
    Macid = data["Macid"]
    Time = data["Time"]
    Resid = data["Resid"]
    Typeid = data["Typeid"]
    cal_1 = data["cal_1"]
    cal_2 = data["cal_2"]
    cal_3 = data["cal_3"]
    cal_4 = data["cal_4"]
    Th_1 = data["Th_1"]
    Th_2 = data["Th_2"]
    Th_3 = data["Th_3"]
    Th_4 = data["Th_4"]
    delay = data["delay"]
    ssid = data["ssid"]
    pass_val = data["pass"]
    Broker_id = data["Broker_id"]
    Port_id = data["Port_id"]
    Config_topic = data["Config_topic"]
    Livedata_topic = data["Livedata_topic"]
    Res_topic = data["Res_topic"]
    Alert_topic = data["Alert_topic"]

    # Print the variables
    # print("Macid:", Macid)
    # print("Time:", Time)
    # print("Resid:", Resid)
    # print("Typeid:", Typeid)
    # print("cal_1:", cal_1)
    # print("cal_2:", cal_2)
    # print("cal_3:", cal_3)
    # print("cal_4:", cal_4)
    # print("Th_1:", Th_1)
    # print("Th_2:", Th_2)
    # print("Th_3:", Th_3)
    # print("Th_4:", Th_4)
    # print("delay:", delay)
    # print("ssid:", ssid)
    # print("pass:", pass_val)
    # print("Broker_id:", Broker_id)
    # print("Port_id:", Port_id)
    # print("Config_topic:", Config_topic)
    # print("Livedata_topic:", Livedata_topic)
    # print("Res_topic:", Res_topic)
    # print("Alert_topic:", Alert_topic)
def map_range(value, from_min, from_max, to_min, to_max):
    # Map the 'value' from the range [from_min, from_max] to [to_min, to_max]
    from_range = from_max - from_min
    to_range = to_max - to_min

    # Scale and shift the value to the new range
    scaled_value = (value - from_min) / from_range
    mapped_value = to_min + (scaled_value * to_range)

    # Make sure the mapped value is within the target range
    return max(min(mapped_value, to_max), to_min)
#check config
check_config_json_file = open("check_config.json", "r")  
check_config_json = json.load(check_config_json_file)
#print(check_config_json) 
check_config = check_config_json['config_device']

#print("Check config : " + str (check_config))
check_config_json_file.close() 

#check conditon for config
if check_config == 'True':
    #print("****************Already configered****************")
    load_config_data()

send_config_topic = "NC/"+ str(hex(uuid.getnode()))+ "/WQM"
broker_address = Broker_id
port = 1883   
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.subscribe(send_config_topic)  # CONFIG TOPIC
        client.subscribe(Res_topic) 
        global Connected
        Connected = True
        print(f"Subscribed to topic: {Res_topic}")  # Print the subscribed topic
    #else:
        #print("Connection failed")

def on_message(client, userdata, msg):
    print(f"Received message on topic '{msg.topic}': {msg.payload.decode()}")  # Print received data
    time.sleep(1)
    data_checking = msg.payload.decode()
    test_data = json.loads(data_checking)
    check_con = test_data["Typeid"]
    #print("Typeid : " +str(check_con))
    if check_con == 0:
        #print("<----------Config cmd received----------->")
        data_sub_json = msg.payload.decode()
        with open("Config.json", "w") as save_json_file:
            # Use json.dump() to write the JSON data to the file
            json.dump(json.loads(data_sub_json), save_json_file, indent=4)
        #print("***** JSON data saved to Config.json ****")
        current_datetime = datetime.datetime.now()
        current_datetime_str = current_datetime.strftime("%Y-%m-%d %H:%M:%S")
        reconfig_data = {
                       "last_update": str(current_datetime_str),
                        "config_device": str(True),
                        "broker_id" : "jitsi.maxelerator.org",
                        "port": "1883"
                        }
                       
        rewrite_config_json = open("check_config.json", "w")
        rewrite_config_json.write(json.dumps(reconfig_data))
        time.sleep(1)
        #print(rewrite_config_json)
        #print("*****New json check saved form config file  ****")
        rewrite_config_json.close()
        time.sleep(1)
    if check_con == 1:
        print("<----------ReConfig cmd received----------->")
        data_sub_json = msg.payload.decode()
        deserialized_data = json.loads(data_sub_json)
        Macid = deserialized_data["Macid"]
        Time = deserialized_data["Time"]
        Typeid = deserialized_data["Typeid"]
        Resid = deserialized_data["Resid"]
        cal_1 = deserialized_data["cal_1"]
        cal_2 = deserialized_data["cal_2"]
        cal_3 = deserialized_data["cal_3"]
        cal_4 = deserialized_data["cal_4"]
        Th_1 = deserialized_data["Th_1"]
        Th_2 = deserialized_data["Th_2"]
        Th_3 = deserialized_data["Th_3"]
        Th_4 = deserialized_data["Th_4"]
        delay = deserialized_data["delay"]
    
        # Print each variable
        print("Macid:", Macid)
        print("Time:", Time)
        print("Typeid:", Typeid)
        print("Resid:", Resid)
        print("cal_1:", cal_1)
        print("cal_2:", cal_2)
        print("cal_3:", cal_3)
        print("cal_4:", cal_4)
        print("Th_1:", Th_1)
        print("Th_2:", Th_2)
        print("Th_3:", Th_3)
        print("Th_4:", Th_4)
        print("delay:", delay)
        with open("Config.json", "r") as json_file:
            data = json.load(json_file)
            # Modify the values with new values
            data["Macid"] = str(Macid)
            data["Time"] = str(Time)
            #data["Typeid"] = 2  # Change to a new integer value
            #data["Resid"] = 456  # Change to a new integer value
            data["cal_1"] =  str(cal_1) # Change to a new float value
            data["cal_2"] = str(cal_2)  # Change to a new float value
            data["cal_3"] = str(cal_3)  # Change to a new float value
            data["cal_4"] = str(cal_4)  # Change to a new float value
            data["Th_1"] = str(Th_1)  # Change to a new integer value
            data["Th_2"] = str(Th_2)  # Change to a new integer value
            data["Th_3"] = str(Th_3)  # Change to a new integer value
            data["Th_4"] = str(Th_4)  # Change to a new integer value
            data["delay"] = str(delay)  # Change to a new integer value

        # Write the modified data back to the file
        with open("Config.json", "w") as json_file:
            json.dump(data, json_file, indent=4)

        # Verify the changes by reading and printing the updated JSON file
        with open("Config.json", "r") as json_file:
            updated_data = json.load(json_file)
            print("******Updated_data******")
            print(updated_data)
        
    if check_con == 2:
        os.system("shutdown -r")


Connected = False

# MQTT connection
clientmqtt = mqtt_client.Client("maxelerator/1")
clientmqtt.on_connect = on_connect
clientmqtt.on_message = on_message  # Set the on_message callback
mqttstatus = clientmqtt.connect(broker_address, port)
clientmqtt.loop_start()
while Connected != True:
    time.sleep(0.1)
if check_config == 'False':
    #print("****************Not configered****************")
    while(1):
        check_config_json_file = open("check_config.json", "r")  
        check_config_json = json.load(check_config_json_file)
        #print(check_config_json) 
        check_config = check_config_json['config_device']
        if check_config == 'True':
            load_config_data()
            break
        current_datetime = datetime.datetime.now()
        current_datetime_str = current_datetime.strftime("%Y-%m-%d %H:%M:%S")
        #create json
        json_send_config = {"Macid" : str(hex(uuid.getnode())),"Time" :str(current_datetime_str),"Location":"SRP","Category":"WQM"} 
        jsons_send_config = json.dumps(json_send_config, indent = 4)
        result = clientmqtt.publish("MAXI0001/NEWDEVICE", jsons_send_config)
        #print("JSON_SENT TO SERVER :")
        #print(jsons_send_config)
        # if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
        #     #print("Msg Successfully sent")
        # else:
            #print("Msg failed to be sent")
        time.sleep(5)    
while True:
    
    try:
        time_now = time.time()
        time_now1 = time.time()
        time_now2 = time.time()
        time_now3 = time.time()
    #     port = '/dev/ttyUSB1'
    #     baud_rate = 9600
    #     timeout = 10

    # # Open the serial port
    #     ser = serial.Serial(port, baud_rate, timeout=timeout)
        try:
            port = '/dev/tds'
            baud_rate = 9600
            timeout = 10
            ser = serial.Serial(port, baud_rate, timeout=timeout)
        except serial.SerialException as e:
            print(f"Serial port error: {e}")
            current_datetime = datetime.datetime.now()
            current_datetime_str = current_datetime.strftime("%Y-%m-%d %H:%M:%S")
            data_heartbeat = {
            'Macid': str(hex(uuid.getnode())),
            'Time': str(current_datetime_str),
            'status':"2",
                }
            # Create JSON data
            heart_json_data = json.dumps(data_heartbeat, indent=4)
            #print(heart_json_data )
            # MQTT data send
            mqtt_data_alert_send = clientmqtt.publish("MAX/SRP/HEARTBEAT", heart_json_data)
            status = mqtt_data_alert_send.rc
            #if status == mqtt_client.MQTT_ERR_SUCCESS:


                #print("Sent")
            #else:
                #print("Failed")
            time.sleep(1)

        SERIAL = '/dev/trub'
        BAUD = 9600
        Defaults.UnitId = 1
        Defaults.Retries = 5
        counter = 1
        client = ModbusClient(method='rtu', port=SERIAL, stopbits=1, bytesize=8, timeout=5, baudrate=BAUD, parity='N')
        connection = client.connect()
        #print(bool(connection))
        while (bool(connection) != True):
            #print("modbus loading...")
            connection = client.connect()
            current_time2 = time.time()
        
            if current_time2 > time_now2+ int(2):
                time_now2 = current_time2
                current_datetime = datetime.datetime.now()
                current_datetime_str = current_datetime.strftime("%Y-%m-%d %H:%M:%S")
                data_heartbeat = {
                'Macid': str(hex(uuid.getnode())),
                'Time': str(current_datetime_str),
                'status':"1",
                    }
                # Create JSON data
                heart_json_data = json.dumps(data_heartbeat, indent=4)
                #print(heart_json_data )
                # MQTT data send
                mqtt_data_alert_send = clientmqtt.publish("MAX/SRP/HEARTBEAT", heart_json_data)
                status = mqtt_data_alert_send.rc
               # if status == mqtt_client.MQTT_ERR_SUCCESS:
                    #print("Sent")
                #else:
                    #print("Failed")
            print(bool(connection))
            time.sleep(1)
        while True:
            heartbeat_delay = 5
            current_time1 = time.time()
            if current_time1 > time_now1+ int(heartbeat_delay):
                time_now1 = current_time1
                current_datetime = datetime.datetime.now()
                current_datetime_str = current_datetime.strftime("%Y-%m-%d %H:%M:%S")
                data_heartbeat = {
                'Macid': str(hex(uuid.getnode())),
                'Time': str(current_datetime_str),
                'status':"Alive",
                    }
                # Create JSON data
                heart_json_data = json.dumps(data_heartbeat, indent=4)
               # print(heart_json_data )
                # MQTT data send
                mqtt_data_alert_send = clientmqtt.publish("MAX/SRP/HEARTBEAT", heart_json_data)
                status = mqtt_data_alert_send.rc
                #if status == mqtt_client.MQTT_ERR_SUCCESS:
                    #print("Sent")
                #else:
                   # print("Failed")
            temp = client.read_input_registers(address=2, count=2, unit=1)
            temprature = BinaryPayloadDecoder.fromRegisters(temp.registers, Endian.Big, wordorder=Endian.Little)
            #temprature = round(temprature.decode_32bit_float(),3)
            temprature = round(temprature.decode_32bit_float(),3)
            time.sleep(0.1)
            trub = client.read_input_registers(address=4, count=2, unit=1)
            turbidity = BinaryPayloadDecoder.fromRegisters(trub.registers, Endian.Big, wordorder=Endian.Little)
            # turbidity = round(turbidity.decode_32bit_float()*cal_1,3)
            turbidity = round(turbidity.decode_32bit_float(),3)
            TSS, Turb = symbols('TSS Turb')
            equation = Eq(ln(TSS), 0.979 * ln(turbidity) + 0.574)
            solution = solve(equation, TSS)
            #print("-------------------------------------------------------")
            #print("TSS :", solution[0])
            tss_value = solution[0]


            #GPIO.output(led_pin, GPIO.HIGH)
            #print("** read from arduino ****")
            
            try:
                line = ser.readline().decode('utf-8').strip()

                if line:
                    #print(f"Received data: {line}")

                    try:
                        data = json.loads(line)
                        voltage = data['voltage']
                        print("VOLTAGE: ")
                        print(voltage)
                        tds = calibrate_sensor(voltage)
                        print("tds: ")
                        print(tds)
                        #tds = data['tds']
                        #print(f"Voltage: {voltage}, TDS: {tds}")
                    except json.JSONDecodeError as e:
                        #print(f"Error decoding JSON: {e}")
                        continue
            except serial.SerialException as se:
               # print(f"Serial exception: {se}")
               sc= "temp"

            tds_value = float(tds)
           # print("TDS :"+str(tds_value))
            ts_value = (float(tds_value)*float(cal_3)) + (float(tss_value)*float(cal_2))
            # print("TS :" + str(ts_value))
            # print("-------------------------------------------------------")
            current_time = time.time()
            #print("delay: "+str(delay) + " current_time:" + str(current_time)+" time_now:"+str(time_now))
            
            if current_time > time_now + int(delay):
                
                time_now = current_time
                current_datetime = datetime.datetime.now()
                current_datetime_str = current_datetime.strftime("%Y-%m-%d %H:%M:%S")
                
                #print("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------")
                print("Mac_id : " + str(uuid.getnode()) + " , Time:" + str(current_datetime_str) + " , Temperature : " + str(temprature) + " , Turbidity : " + str(turbidity)+ ", TSS:"+str(tss_value) + ", TDS : " + str(tds_value) + ", ts :" + str(ts_value))
                #print("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------")
                ts_cal = float(ts_value)*float(cal_1)
                tss_cal = float(tss_value)*float(cal_2)
                tds_cal = float(tds_value)*float(cal_3)
                turbidity_cal = float(turbidity)*float(cal_4)
                # print("After calibration----------------------------------------------------------------------------------------")
                print( "Turbidity : " + str(turbidity_cal)+ ", TSS:"+str(tss_cal) + ", TDS : " + str(tds_cal) + ", ts :" + str(ts_cal))
                # print("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------")
                
                data = {
                'Macid': str(hex(uuid.getnode())),
                'Time': str(current_datetime_str),
                'parameter1': str(ts_cal),
                'parameter2': str(tss_cal),
                'parameter3': str(tds_cal),
                'parameter4': str(turbidity_cal),
                'parameter5': "0"
                    }
                # Create JSON data
                sensor_json_data = json.dumps(data, indent=4)
                #print(sensor_json_data)
                # MQTT data send
                mqtt_data_send = clientmqtt.publish(Livedata_topic, sensor_json_data)
                
                status = mqtt_data_send.rc
               # if status == mqtt_client.MQTT_ERR_SUCCESS:
                   #print("Sent")
                #else:
                    #print("Failed")

                
            if (float(ts_value)*float(cal_1)) > int(Th_1):
                current_datetime = datetime.datetime.now()
                current_datetime_str = current_datetime.strftime("%Y-%m-%d %H:%M:%S")
                data_alert_t1 = {
                'Macid': str(hex(uuid.getnode())),
                'Time': str(current_datetime_str),
                'Alert':"1",
                'Value': str(ts_value),
                "Alert_t":"0"
                    }
                
                # Create JSON data
                Alert_json_data = json.dumps(data_alert_t1, indent=4)
                print(Alert_json_data)
                # MQTT data send
                mqtt_data_alert_send = clientmqtt.publish(Alert_topic, Alert_json_data)
                status = mqtt_data_alert_send.rc
               # if status == mqtt_client.MQTT_ERR_SUCCESS:
                   # print("Sent")
               # else:
                   # print("Failed")

            if (float(tss_value)*float(cal_2)) > int(Th_2):
                current_datetime = datetime.datetime.now()
                current_datetime_str = current_datetime.strftime("%Y-%m-%d %H:%M:%S")
                data_alert_t1 = {
                'Macid': str(hex(uuid.getnode())),
                'Time': str(current_datetime_str),
                'Alert':"2",
                'Value': str(tss_value),
                "Alert_t":"0"
                    }
                
                # Create JSON data
                Alert_json_data = json.dumps(data_alert_t1, indent=4)
                print(Alert_json_data)
                # MQTT data send
                mqtt_data_alert_send = clientmqtt.publish(Alert_topic, Alert_json_data)
                status = mqtt_data_alert_send.rc
               # if status == mqtt_client.MQTT_ERR_SUCCESS:
                    #print("Sent")
                #else:
                   # print("Failed")
            
            if (float(tds_value)*float(cal_3)) > int(Th_3):
                current_datetime = datetime.datetime.now()
                current_datetime_str = current_datetime.strftime("%Y-%m-%d %H:%M:%S")
                data_alert_t1 = {
                'Macid': str(hex(uuid.getnode())),
                'Time': str(current_datetime_str),
                'Alert':"3",
                'Value': str(tds_value),
                "Alert_t":"0"
                    }
                
                # Create JSON data
                Alert_json_data = json.dumps(data_alert_t1, indent=4)
                print(Alert_json_data)
                # MQTT data send
                mqtt_data_alert_send = clientmqtt.publish(Alert_topic, Alert_json_data)
                status = mqtt_data_alert_send.rc
                if status == mqtt_client.MQTT_ERR_SUCCESS:
                    print("Sent")
                #else:
                   # print("Failed")
            
            if (float(turbidity)*float(cal_4)) > int(Th_4):
                current_datetime = datetime.datetime.now()
                current_datetime_str = current_datetime.strftime("%Y-%m-%d %H:%M:%S")
                data_alert_t1 = {
                'Macid': str(hex(uuid.getnode())),
                'Time': str(current_datetime_str),
                'Alert':"4",
                'Value': str(turbidity),
                "Alert_t":"0"
                    }
                
                # Create JSON data
                Alert_json_data = json.dumps(data_alert_t1, indent=4)
                print(Alert_json_data)
                # MQTT data send
                mqtt_data_alert_send = clientmqtt.publish(Alert_topic, Alert_json_data)
                status = mqtt_data_alert_send.rc
                if status == mqtt_client.MQTT_ERR_SUCCESS:
                    print("Sent")
                #else:
                   # print("Failed")
    except Exception as e:
        #print("Error:", str(e))
        traceback.print_exc()
       #  print("****************ERROR*********************")
        time.sleep(2)
        continue 
