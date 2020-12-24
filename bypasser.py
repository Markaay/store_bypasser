###official store bypasser code
#author: mark ebergen
#owner: Markaay
#version: v20201224
###official store bypasser code

#Libraries
import RPi.GPIO as GPIO
import time
import csv
from datetime import datetime
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
    
#calibration variables in centimeters
calibration_distance = 0 #centimeters
calibration_distance_tolerance = 10 #centimeters
calibration_points_required = 60 #tracking units
calibration_points_list = []

#configuration
STORE_BYPASSER_ID = "dist1"
bypasser_detected = False
ticker = 0.1 #seconds
ticker_heartbeat = 60 #seconds
ticker_calibration = 1 #seconds
ticker_bypasser = 1 #seconds
log_directory = "logs"
log_suffix = "_log"
tmp_event_queue = []
#set loop values to align with ticker
calibration_loop_value = 1 #seconds
heartbeat_loop_value = 60 #seconds
bypasser_loop_value = 1 #seconds



#write row to a logfile
def write_to_log(filename, row):
  with open(r'%s.txt' % filename, 'a') as f:
    writer = csv.writer(f)
    writer.writerow(row)
  
  return row
 
#get distance from ultrasoon sensor 
def retrieve_distance():
  # set Trigger to HIGH
  GPIO.output(GPIO_TRIGGER, True)
 
  # set Trigger after 0.01ms to LOW
  time.sleep(0.00001)
  GPIO.output(GPIO_TRIGGER, False)
 
  StartTime = time.time()
  StopTime = time.time()
 
  # save StartTime
  while GPIO.input(GPIO_ECHO) == 0:
    StartTime = time.time()
 
  # save time of arrival
  while GPIO.input(GPIO_ECHO) == 1:
    StopTime = time.time()
 
  # time difference between start and arrival
  TimeElapsed = StopTime - StartTime
  # multiply with the sonic speed (34300 cm/s)
  # and divide by 2, because there and back
  distance = (TimeElapsed * 34300) / 2
 
  return distance

#detect bypasser from tracked distance
def detect_bypasser(tracked_distance, log_destination, timestamp_string):
  #get global variables in bypasser scope
  global calibration_distance
  global calibration_distance_tolerance
  global bypasser_detected
  global ticker
  global ticker_bypasser
  global bypasser_loop_value
  if tracked_distance < (calibration_distance + calibration_distance_tolerance):
    #set bypasser_detected variable to true to start bypasser ticker events
    bypasser_detected = True
    #only tick events on first ticker hit within the ticker of the bypasser
    if bypasser_loop_value == ticker_bypasser:
      write_to_log(log_destination, [timestamp_string, STORE_BYPASSER_ID, "bypasser", tracked_distance])
      print("bypasser detected!")
      #count down loop value untill 0 before next bypasser ticker
      bypasser_loop_value = bypasser_loop_value - ticker

    else:
      #count down loop value untill 0 before next bypasser ticker
      bypasser_loop_value = bypasser_loop_value - ticker
      if bypasser_loop_value == 0:
        #reset bypasser ticker
        bypasser_loop_value = ticker_bypasser
    
  else:
    #set bypasser_detected variable to false and stop bypasser ticker process
    bypasser_detected = False
    #reset bypasser loop value to initial tickervalue
    bypasser_loop_value = ticker_bypasser
    print("no bypasser detected")

  return bypasser_detected

#calibration function
def calibrate_distance(tracked_distance):
  global calibration_points_list
  global calibration_points_required
  global calibration_distance_tolerance
  global calibration_distance
  #check if 30 calibration points are collected
  if len(calibration_points_list) < calibration_points_required:
    calibration_points_list.append(tracked_distance)
    #print("calibration point added: ", tracked_distance, len(calibration_points_list))
      
  else:
    #set new calibration distance
    print("calibration points collected, start calibration process...")
    calibration_points_average = sum(calibration_points_list) / len(calibration_points_list)
    calibration_valid = True
    #control for outliers in calibration_points
    for calibration_point in calibration_points_list:
      #control if outlier is outside of tolerance range
      if calibration_point > (calibration_points_average + calibration_distance_tolerance) or calibration_point < (calibration_points_average - calibration_distance_tolerance):
        #calibration point outside of tolerance range, stop calibration
        calibration_valid = False    
      
    #calibration process succesfull
    if calibration_valid == True:
      #set new calibration distance default
      calibration_distance = calibration_points_average
      print("new calibration distance: ", calibration_points_average)
      write_to_log()
      #reset calibration points list
      calibration_points_list = []
      return calibration_points_average
    
    else:
      print("calibration process unsuccesfull/unrequired, restart process..")
      #reset calibration points list
      calibration_points_list = []
      return calibration_distance
    
#start main application
if __name__ == '__main__':
  try:
    while True:
      ###ticker context
      #CONTEXT
      timestamp_string = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
      date_string = datetime.now().strftime("%Y%m%d")
      log_destination = log_directory + "/" + date_string + log_suffix

      ###distance process in ticker
      #DEFAULT
      dist = retrieve_distance()
      #print ("tracked distance = %.1f cm" % dist)
      
      ###anomaly process in ticker
      #BYPASSER
      bypasser = detect_bypasser(dist, log_destination, timestamp_string)
      print(bypasser)

      ###heartbeat process in ticker
      #HEARTBEAT
      if heartbeat_loop_value > ticker:
        #use the heartbeat loop value to avoid hearbeats too fast!
        heartbeat_loop_value = heartbeat_loop_value - ticker

      else:
        #log heartbeat
        write_to_log(log_destination, [timestamp_string, STORE_BYPASSER_ID, "heartbeat"])
        heartbeat_loop_value = ticker_heartbeat
        
      ###calibration process in ticker
      #CALIBRATION
      if calibration_loop_value > ticker:
        #use the calibration loop value to avoid calibrating too fast!
        calibration_loop_value = calibration_loop_value - ticker
      else:
        calibrate_distance(dist)
        write_to_log(log_destination, [timestamp_string, STORE_BYPASSER_ID, "calibration_distance", calibrate_distance])
        #reset calibration interval to the ticker of the calibration
        calibration_loop_value = ticker_calibration

      ###sleep equal to the ticker interval configuration time
      #TICKER SPEED
      time.sleep(ticker)

  # Reset by pressing CTRL + C
  except KeyboardInterrupt:
      print("Tracking stopped")
      GPIO.cleanup()
