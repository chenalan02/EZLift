const int rightMotorPins[4] = {2, 3, 4, 5};
const int leftMotorPins[4] = {11, 10, 12, 13};
const int motorEnable = 6;

const int stepperIn3 = 7;
const int stepperIn4 = 8;

const int encoderA = A0;
const int encoderB = A5;


const int trigPin = A0;
const int echoPins[3] = {A1, A2, A3};
const int stopDistance = 8;


const int limitSwitch = 9;

bool liftDown = false;

void setup() {
    Serial.begin(9600);
    
    for (int i = 0; i < 4; i++) {
        pinMode(rightMotorPins[i], OUTPUT);
        pinMode(leftMotorPins[i], OUTPUT);
    }
    pinMode(motorEnable, OUTPUT);
    
    pinMode(stepperIn3, OUTPUT);
    pinMode(stepperIn4, OUTPUT);
    
    pinMode(limitSwitch, INPUT_PULLUP);
    
    digitalWrite(motorEnable, HIGH);
    
    // Lower lift to ground at startup
    while (digitalRead(limitSwitch) == HIGH) {
        digitalWrite(stepperIn3, LOW);
        digitalWrite(stepperIn4, HIGH);
    }

    pinMode(trigPin, OUTPUT);
    for (int i = 0; i < 3; i++) {
        pinMode(echoPins[i], INPUT);
    }

    digitalWrite(stepperIn3, LOW);
    digitalWrite(stepperIn4, LOW);
    liftDown = true;
}

void raiseLift(int duration) {
    digitalWrite(stepperIn3, HIGH);
    digitalWrite(stepperIn4, LOW);
    delay(duration);
    digitalWrite(stepperIn3, LOW);
    digitalWrite(stepperIn4, LOW);
}

void lowerLift(int duration) {
    digitalWrite(stepperIn3, LOW);
    digitalWrite(stepperIn4, HIGH);
    delay(duration);
    digitalWrite(stepperIn3, LOW);
    digitalWrite(stepperIn4, LOW);
}

float getDistance(int echoPin) {
    long duration;
    float distance;

    // Ensure the trigger pin is LOW before starting
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    
    // Send a 10µs HIGH pulse to trigger the sensor
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the echo pin and measure the response duration
    duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout

    // If pulseIn returns 0, retry once after a short delay
    if (duration == 0) {
        delay(10);
        duration = pulseIn(echoPin, HIGH, 30000);
    }

    // Convert duration to inches (Sound speed in air ~0.0133 inches/microsecond)
    distance = duration * 0.0133 / 2;

    // Print debug info
    Serial.print("Echo on pin ");
    Serial.print(echoPin);
    Serial.print(" duration: ");
    Serial.print(duration);
    Serial.print(" distance: ");
    Serial.println(distance);

    return (duration > 0) ? distance : -1;  // Return -1 for invalid readings
}
bool obstacleDetected() {
    for (int i = 0; i < 3; i++) {
        float distance = getDistance(echoPins[i]);

        if (distance != -1) {  // Ignore invalid readings
            Serial.print("Sensor ");
            Serial.print(i + 1);
            Serial.print(" distance: ");
            Serial.println(distance);
            
            if (distance < stopDistance) {
                return true;  // Obstacle detected
            }
        }
        
        delay(50);  // Small delay to prevent sensor interference
    }
    return false;
}


void moveRobot(int speed, int r1, int r2, int l1, int l2) {
    // if (obstacleDetected()) {
    //     moveRobot(0, LOW, LOW, LOW, LOW);
    //     return;
    // }
    analogWrite(motorEnable, abs(speed));
    digitalWrite(rightMotorPins[0], r1);
    digitalWrite(rightMotorPins[1], r2);
    digitalWrite(rightMotorPins[2], r1);
    digitalWrite(rightMotorPins[3], r2);
    digitalWrite(leftMotorPins[0], l1);
    digitalWrite(leftMotorPins[1], l2);
    digitalWrite(leftMotorPins[2], l1);
    digitalWrite(leftMotorPins[3], l2);
}

void moveLiftToGround() {
    while (digitalRead(limitSwitch) == HIGH) {
        digitalWrite(stepperIn3, LOW);
        digitalWrite(stepperIn4, HIGH);
    }
    digitalWrite(stepperIn3, LOW);
    digitalWrite(stepperIn4, LOW);
    liftDown = true;
}

void grabOffShelf() {
    raiseLift(3900);
    moveRobot(150, HIGH, LOW, HIGH, LOW);
    delay(2200);
    moveRobot(0, LOW, LOW, LOW, LOW);
    delay(200);
    raiseLift(800);
    delay(200);
    moveRobot(150, LOW, HIGH, LOW, HIGH);
    delay(2500);
    moveRobot(0, LOW, LOW, LOW, LOW);
    moveRobot(150, LOW, HIGH, HIGH, LOW);
    delay(4000);
    moveRobot(0, LOW, LOW, LOW, LOW);
    moveRobot(150, HIGH, LOW, HIGH, LOW);
    delay(1500);
    moveRobot(0, LOW, LOW, LOW, LOW);
    moveLiftToGround();
    moveRobot(255, LOW, HIGH, LOW, HIGH);
    delay(1000);
    moveRobot(0, LOW, LOW, LOW, LOW);
}

void grabOffGround() {
    moveRobot(150, HIGH, LOW, HIGH, LOW);
    delay(2200);
    moveRobot(0, LOW, LOW, LOW, LOW);
    delay(200);
    raiseLift(800);
    delay(200);
    moveRobot(150, LOW, HIGH, LOW, HIGH);
    delay(2500);
    moveRobot(0, LOW, LOW, LOW, LOW);
    moveRobot(150, LOW, HIGH, HIGH, LOW);
    delay(4000);
    moveRobot(0, LOW, LOW, LOW, LOW);
    moveRobot(150, HIGH, LOW, HIGH, LOW);
    delay(1500);
    moveRobot(0, LOW, LOW, LOW, LOW);
    moveLiftToGround();
    moveRobot(255, LOW, HIGH, LOW, HIGH);
    delay(1000);
    moveRobot(0, LOW, LOW, LOW, LOW);
}

void topToBottom() {
    raiseLift(3900);
    moveRobot(150, HIGH, LOW, HIGH, LOW);
    delay(2200);
    moveRobot(0, LOW, LOW, LOW, LOW);
    delay(200);
    raiseLift(800);
    delay(200);
    moveRobot(150, LOW, HIGH, LOW, HIGH);
    delay(2000);
    moveRobot(0, LOW, LOW, LOW, LOW);
    moveLiftToGround();
    moveRobot(150, HIGH, LOW, HIGH, LOW);
    delay(2000);
    moveRobot(0, LOW, LOW, LOW, LOW);
    delay(200);
    moveRobot(255, LOW, HIGH, LOW, HIGH);
    delay(1000);
    moveRobot(0, LOW, LOW, LOW, LOW);
}

void bottomToTop() {
    moveRobot(150, HIGH, LOW, HIGH, LOW);
    delay(2200);
    moveRobot(0, LOW, LOW, LOW, LOW);
    delay(200);
    raiseLift(800);
    delay(200);
    moveRobot(150, LOW, HIGH, LOW, HIGH);
    delay(2000);
    moveRobot(0, LOW, LOW, LOW, LOW);
    raiseLift(4000);
    moveRobot(150, HIGH, LOW, HIGH, LOW);
    delay(2000);
    moveRobot(0, LOW, LOW, LOW, LOW);
    delay(200);
    lowerLift(800);
    moveRobot(255, LOW, HIGH, LOW, HIGH);
    delay(1000);
    moveRobot(0, LOW, LOW, LOW, LOW);
}

void pickOffGround() {
    moveRobot(150, HIGH, LOW, HIGH, LOW);
    delay(2200);
    moveRobot(0, LOW, LOW, LOW, LOW);
    delay(200);
    raiseLift(800);
}

void dropOnShelf() {
    raiseLift(3900);
    moveRobot(150, HIGH, LOW, HIGH, LOW);
    delay(2000);
    moveRobot(0, LOW, LOW, LOW, LOW);
    delay(200);
    lowerLift(800);
    moveRobot(255, LOW, HIGH, LOW, HIGH);
    delay(1200);
    moveRobot(0, LOW, LOW, LOW, LOW);
    moveLiftToGround();
}


unsigned long obstacleStartTime = 0;  // Stores the time when the obstacle is first detected
bool obstacleActive = false;          // Flag to track obstacle status
const unsigned long OBSTACLE_STOP_TIME = 3000; // 3 seconds

void loop() {
    // if (obstacleDetected()) {
    //     if (!obstacleActive) {
    //         // First time obstacle is detected, store the start time
    //         obstacleStartTime = millis();
    //         obstacleActive = true;
    //     }

    //     // Stop the robot for at least 3 seconds
    //     moveRobot(0, LOW, LOW, LOW, LOW);
    //     digitalWrite(stepperIn3, LOW);
    //     digitalWrite(stepperIn4, LOW);
    // } else {
    //     // If 3 seconds have passed, reset the obstacle state
    //     if (obstacleActive && millis() - obstacleStartTime >= OBSTACLE_STOP_TIME) {
    //         obstacleActive = false; // Reset flag
    //     }
    // }

    // Only read serial commands if the obstacle stop time has passed
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command == "lift1") {
            raiseLift(2000);
        } else if (command == "lift2") {
            raiseLift(4000);
        } else if (command == "liftgnd") {
            moveLiftToGround();
        } else if (command == "stop" || command == "s") {
            moveRobot(0, LOW, LOW, LOW, LOW);
            digitalWrite(stepperIn3, LOW);
            digitalWrite(stepperIn4, LOW);
        } else if (command == "gos") {
            grabOffShelf();
        } else if (command == "tob") {
            topToBottom();
        } else if (command == "gog") {
            grabOffGround();
        } else if (command == "bot") {
            bottomToTop();
        } else if (command == "pog") {
            pickOffGround();
        } else if (command == "dos") {
            dropOnShelf();
        } else {
            int spaceIndex = command.indexOf(':');
            if (spaceIndex != -1) {
                String cmdType = command.substring(0, spaceIndex);
                int speed = command.substring(spaceIndex + 1).toInt();
                
                if (cmdType == "forwards") {
                    if (speed > 0) {
                        moveRobot(speed, HIGH, LOW, HIGH, LOW);
                    } else {
                        moveRobot(-speed, LOW, HIGH, LOW, HIGH);
                    }
                } else if (cmdType == "turn") {
                    if (speed > 0) {
                        moveRobot(speed, HIGH, LOW, LOW, HIGH);
                    } else {
                        moveRobot(-speed, LOW, HIGH, HIGH, LOW);
                    }
                } else if (cmdType == "side") {
                    analogWrite(motorEnable, abs(speed));
                    if (speed > 0) {
                        digitalWrite(rightMotorPins[0], LOW);
                        digitalWrite(rightMotorPins[1], HIGH);
                        digitalWrite(rightMotorPins[2], HIGH);
                        digitalWrite(rightMotorPins[3], LOW);
                        digitalWrite(leftMotorPins[0], LOW);
                        digitalWrite(leftMotorPins[1], HIGH);
                        digitalWrite(leftMotorPins[2], HIGH);
                        digitalWrite(leftMotorPins[3], LOW);
                    } else {
                        digitalWrite(rightMotorPins[0], HIGH);
                        digitalWrite(rightMotorPins[1], LOW);
                        digitalWrite(rightMotorPins[2], LOW);
                        digitalWrite(rightMotorPins[3], HIGH);
                        digitalWrite(leftMotorPins[0], HIGH);
                        digitalWrite(leftMotorPins[1], LOW);
                        digitalWrite(leftMotorPins[2], LOW);
                        digitalWrite(leftMotorPins[3], HIGH);
                    }
                } else {
                    moveRobot(0, LOW, LOW, LOW, LOW);
                    digitalWrite(stepperIn3, LOW);
                    digitalWrite(stepperIn4, LOW);
                }
            }
        }
    }
}
