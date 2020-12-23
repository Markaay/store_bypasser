#official store bypasser code
#author: mark ebergen

#Libraries
import RPi.GPIO as GPIO
import time
 
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
ticker_interval = 0.1 #seconds
ticker_calibration = 1 #seconds
calibration_interval = 1 
 
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


def detect_distance_anomaly(tracked_distance):
    global calibration_distance
    global calibration_distance_tolerance
    if tracked_distance < (calibration_distance + calibration_distance_tolerance):
        return "got you boi!"
    else:
        return "nothing interesting"


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
            #reset calibration points list
            calibration_points_list = []
            return calibration_points_average
        
        else:
            print("calibration process unsuccesfull/unrequired, restart process..")
            #reset calibration points list
            calibration_points_list = []
            return 0
        


 
if __name__ == '__main__':
    try:
        while True:
            #distance process in ticker
            dist = retrieve_distance()
            #print ("tracked distance = %.1f cm" % dist)
            
            #anomaly process in ticker
            anomaly = detect_distance_anomaly(dist)
            print(anomaly)
            
            #calibration process in ticker
            if calibration_interval > ticker_interval:
                #use the calibration interval to avoid calibrating too fast!
                calibration_interval = calibration_interval - ticker_interval
            else:
                calibrate_distance(dist)
                #reset calibration interval to the ticker of the calibration
                calibration_interval = ticker_calibration

                
            #sleep equal to the ticker interval configuration time
            time.sleep(ticker_interval)
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Tracking stopped")
        GPIO.cleanup()
