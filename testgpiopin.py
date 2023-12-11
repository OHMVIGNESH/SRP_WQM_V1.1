import RPi.GPIO as GPIO
import time
# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)
# Define the GPIO pin for the LED
led_pin = 18
# Set up the LED pin as an output
GPIO.setup(led_pin, GPIO.OUT)
try:
    while True:
        # Turn on the LED
        GPIO.output(led_pin, GPIO.HIGH)
        print("***read signal cmd sned ****")
        time.sleep(1)  # Wait for 1 second

        # Turn off the LED
        GPIO.output(led_pin, GPIO.LOW)
        print("LED OFF")
        time.sleep(1)  # Wait for 1 second

except KeyboardInterrupt:
    # Clean up GPIO on keyboard interrupt
    GPIO.cleanup()

