# SRP_WQM
Jetson nano to take data using IOT

# json for send server
'''
     {"Macid" : cid,
      "Time" :formattedDate,
      "Location":"SRP",
      "Category":"wm"}'''
     
"""
#Default value in json
config_json = {"usbserial" : '/dev/ttyUSB0',
               "baudrate":"9600",
               "cal_1": 1.0,
               "cal_2": 1.00,
               "cal_3": 1.00,
               "cal_4": 1.00,
               "Th_1" : 50,
               "Th_2" : 50,
               "Th_3" : 50,
               "Th_4" : 50,
               "delay":1,
               "subscribed_topic" : "MAX/SRP/SUB",  
               "publish_topic_data" : "MAX/SRP/PUB/DATA",
               "publish_topic_alert" : "MAX/SRP/PUB/ALERT"
                }
save_json_file = open("Config.json", "w")  
json.dump(config_json, save_json_file, indent = 4)  
save_json_file.close() 
"""
