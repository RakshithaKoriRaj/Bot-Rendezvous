# Bot-Rendezvous
The bot uses microcontroller Adafruit HUZZAH32-ESP32 Feather Board with WiFi built-in communication, I2C protocol that guides the bot to navigate by calculating the relative distance to the beacon through signal strength measurements.

## Components used:
Adafruit Huzzah esp8622, L298N Motor Drive Controller, Ultrasonic sensor, servo motor, two motors, one lithium polymer battery and 4 AA batteries

## Obstacle avoidance: 
Ultrasonic sensor is used for obstacle detection. Ultrasonic sensor is integrated with the servo motor to check the obstacles at four directions: 0°, 45°, 90°, 135°, 180°.  
- The bot is travelling in forward direction if the distance between the obstacle and the bot is more than the 20cm. [The reading is taken every 100ms approximately]
- Every 2 seconds, bot checks distance at 45°(right distance) and 135°(left distance), bot makes a slight right turn or left turn respectively, if the distance to obstacle is less than 30cm. 
- If the distance to the obstacle is less than the 20cm, while travelling straight it takes the following steps: 
  - The bot stops and goes backward for 500ms.
  - Then bot with the help of servo motor rotates the ultrasonic sensor in 90° and 180° to take right and left distance respectively.
  - If the right distance is more then left distance, it takes 90° right turn and continue to go straight, else takes left turn. 

## Tracking the WIFI beacon:
Before bot moving, it establishes connection with the WIFI beacon and takes the signal strength.
- Every 1.5seconds, current signal strength is compared with previous signal strength. If the current signal strength is less than 2db of previous signal strength, then the bot takes following steps to adjust its direction towards the beacon: 
  - The bot stops and notes signal strength [left signal strength].
  - The bot takes a 90° right turn and moves forward for 750ms and notes the signal strength [right signal strength].
  - If the left signal strength was better than the right, bot takes a U-turn else continue moving straight. 
  - When moving forward for 750ms to get right signal strength, if obstacle is encountered, bot takes a U-turn and continues moving straight
- Every 100ms, bot checks WIFI signal strength, if the signal strength is more than -37db, bot stops.

## Fine tuning:
As we get closer to the WIFI beacon, we make the following adjustments: 
- Frequency of checking for reduction in WIFI signal strength is increased. 
- The time bot travels to get right signal strength is reduced.
## Reference:
- https://howtomechatronics.com/
- https://www.youtube.com/watch?v=4CFO0MiSlM8

