# Tentative Quest Plan

## Division of labor

### Ayush
* Steering and speed control thru web page DONE
* Database output (table?) thru web page DONE
* Webcam livestream thru web page DONE
* Write socket commands

### David
* UDP receiving
* TCP connection with web page (?)
* Webcam video processing (?)
* IR signal receiver
* Wifi & UDP networking

### Kyle
* Range sensors/collision sensor (microlidar or ultrasonic?)
* PID control for speed
* PID control for steering
* Alphanumeric display thru I2C
* Speedometer (?)
* Database management/recording split times

## Interfaces
#### ESP -> Backend
Update split time:
{"Id": 1, "Split": 10.15}

#### Backend -> ESP
Format:

speed steering status

-3..3  -3..3    bool
