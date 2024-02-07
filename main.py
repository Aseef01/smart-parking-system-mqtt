from machine import Pin, SoftI2C, ADC, PWM
from time import sleep
import network
import dht

from servo import Servo
from lcd_api import LcdApi
from i2c_lcd import I2cLcd
from umqtt.simple import MQTTClient
import ujson

# Wifi Declaration
# WIFI_SSID = "13-6_2.4G"
# WIFI_PASSWORD = "01110292882ABC"

WIFI_SSID = "iPhone AZFAR"
WIFI_PASSWORD = "87878787"

# MQTT Configuration
# BROKER_ADDRESS = "test.mosquitto.org"
# BROKER_PORT = 1883
# BROKER_USER = ""
# BROKER_PASSWORD = ""
# client_id = ""

# mqtt_broker = "test.mosquitto.org"
mqtt_broker = "broker.mqttdashboard.com"
mqtt_port = 1883
mqtt_user = ""
mqtt_password = ""
mqtt_client_id = ""

TOPIC1_PUBLISH = b"data_sensor"
TOPIC2_PUBLISH = b"parking_slot"
subscribe_topic = b"button_control"

# Connect to WiFi
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(WIFI_SSID, WIFI_PASSWORD)

while not wifi.isconnected():
    print(".", end="")
    sleep(1)

print("Connected to WiFi")

# Pin Declaration
gatePin = Pin(17, Pin.IN, Pin.PULL_UP)
lightPin = Pin(18, Pin.IN, Pin.PULL_UP)

gateState = False;
lightState = False;

lightSensorPin = Pin(35, Pin.IN)

enterPin = Pin(34, Pin.IN)
exitPin = Pin(33, Pin.IN)

parking_1 = Pin(32, Pin.IN)
parking_2 = Pin(27, Pin.IN)
parking_3 = Pin(26, Pin.IN)
parking_4 = Pin(25, Pin.IN)

servoPin = Servo(pin=19)
relay_pin = Pin(23, Pin.OUT)

# adc3 = MQ135(12)
sensor = dht.DHT22(Pin(16))

buzzer_pwm = PWM(Pin(13))
buzzer_pwm.duty(0)

I2C_ADDR = 0x27
totalRows = 4
totalColumns = 20

i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=10000)     #initializing the I2C method for ESP32
lcd = I2cLcd(i2c, I2C_ADDR, totalRows, totalColumns)

def callback(topic, msg):
    global gateState, lightState
    command = msg.decode("utf-8")
    print("Received message on topic {}: {}".format(topic, command))
    if command == "lightOn":
        print("Light On")
        lightState = not lightState;
    elif command == "lightOff":
        print("Light Off")
        lightState = not lightState;
    elif command == "gateOn":
        print("Gate On")
        gateState = not gateState;
    elif command == "gateOff":
        print("Gate Off")
        gateState = not gateState;
    else:
        print("Unknown command")
        
# Connect to the MQTT broker
mqtt = MQTTClient(mqtt_client_id, mqtt_broker, port=mqtt_port, user=mqtt_user, password=mqtt_password)
mqtt.connect()
mqtt.set_callback(callback)
mqtt.subscribe(subscribe_topic)

def play_tone(frequency, duration):
    # Set the frequency and duty cycle to play a tone
    buzzer_pwm.freq(frequency)
    buzzer_pwm.duty(50)  # 50% duty cycle
    sleep(duration)

    # Stop the tone
    buzzer_pwm.duty(0)
    sleep(0.5)
    
def lcdDisplay():
    lcd.move_to(0, 0)
    lcd.putstr("Parking 1:")
    lcd.move_to(10, 0)
    lcd.putstr(" empty    " if parking_2.value() == 1 else " fill      ")
    
    lcd.move_to(0, 1)
    lcd.putstr("Parking 2:")
    lcd.move_to(10, 1)
    lcd.putstr(" empty    " if parking_3.value() == 1 else " fill      ")
    
    lcd.move_to(0, 2)
    lcd.putstr("Parking 3:")
    lcd.move_to(10, 2)
    lcd.putstr(" empty    " if parking_1.value() == 1 else " fill      ")
    
    lcd.move_to(0, 3)
    lcd.putstr("Parking 4:")
    lcd.move_to(10, 3)
    lcd.putstr(" empty" if parking_4.value() == 1 else " fill      ")
    
while True:
    if parking_1.value() == 1 or  parking_2.value() == 1 or parking_3.value() == 1 or parking_4.value() == 1:
        lcdDisplay()
    elif parking_1.value() == 0 and  parking_2.value() == 0 and parking_3.value() == 0 and parking_4.value() == 0:
        if enterPin.value() == 0 or exitPin.value() == 0:
            lcd.move_to(0, 0)
            lcd.putstr("All parking is full.  Please come back ")
            lcd.move_to(0, 2)
            lcd.putstr("  at another time.                     ")
            play_tone(1000, 1)  # Play a 1000 Hz tone for 1 second
            sleep(1)
        else:
            lcdDisplay()
            
    mqtt.check_msg()
    
    if gatePin.value() == 0:
        gateState = not gateState;
#         print("Gate Button pressed!")
    
    if lightPin.value() == 0:
        lightState = not lightState;
#         print(lightState)

    print(lightState)
    
        
    if lightSensorPin.value() == 1 or lightState == True:
        print("Light Button pressed!")
        relay_pin.off()
    else:
        relay_pin.on()
        
#     servoState = 0;

    if enterPin.value() == 0 or exitPin.value() == 0 or gateState == True:
        if parking_1.value() == 1 or  parking_2.value() == 1 or parking_3.value() == 1 or parking_4.value() == 1 or gateState == True:
            servoPin.move(170)
            servoState = 1;
    else:
        servoPin.move(100)
        servoState = 0;
    
    payload = {
        "parking1": parking_2.value(),
        "parking2": parking_3.value(),
        "parking3": parking_1.value(),
        "parking4": parking_4.value(),
        "light": relay_pin.value(),
        "gate": servoState
        }

    # Convert the JSON object to a string
    payload_str = ujson.dumps(payload)
    
    mqtt.publish(TOPIC2_PUBLISH, payload_str)
    
    sensor.measure()
    temp = sensor.temperature()
    hum = sensor.humidity()
    
    payload = {"temperature": temp, "humidity": hum}

    # Convert the JSON object to a string
    payload_str = ujson.dumps(payload)
    
    mqtt.publish(TOPIC1_PUBLISH, payload_str)
    
    sleep(1)
