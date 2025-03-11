// Arduino Mecanum Drive + Stepper Control

// Motor Control Pins (DC Motors)
const int rightMotorPins[4] = {2, 3, 4, 5}; // Right side L298N
const int leftMotorPins[4] = {11, 10, 12, 13}; // Left side L298N
const int motorEnable = 6; // Enable pin for all DC motors

// Stepper Motor Pins
const int stepperIn3 = 7;
const int stepperIn4 = 8;

// Encoder Pins (Not used in this simple implementation, but defined)
const int encoderA = A0;
const int encoderB = A5;

// Limit Switch Pin
const int limitSwitch = 9;

bool liftDown = false;

void setup() {
    Serial.begin(9600);
    
    // Set motor control pins as OUTPUT
    for (int i = 0; i < 4; i++) {
        pinMode(rightMotorPins[i], OUTPUT);
        pinMode(leftMotorPins[i], OUTPUT);
    }
    pinMode(motorEnable, OUTPUT);
    
    // Stepper motor pins
    pinMode(stepperIn3, OUTPUT);
    pinMode(stepperIn4, OUTPUT);
    
    // Limit switch
    pinMode(limitSwitch, INPUT_PULLUP);
    
    // Enable motors
    digitalWrite(motorEnable, HIGH);
}

void moveLift(int duration) {
    digitalWrite(stepperIn3, HIGH);
    digitalWrite(stepperIn4, LOW);
    delay(duration);
    digitalWrite(stepperIn3, LOW);
    digitalWrite(stepperIn4, LOW);
}

void moveRobot(int speed, int r1, int r2, int l1, int l2) {
    analogWrite(motorEnable, speed);
    digitalWrite(rightMotorPins[0], r1);
    digitalWrite(rightMotorPins[1], r2);
    digitalWrite(rightMotorPins[2], r1);
    digitalWrite(rightMotorPins[3], r2);
    digitalWrite(leftMotorPins[0], l1);
    digitalWrite(leftMotorPins[1], l2);
    digitalWrite(leftMotorPins[2], l1);
    digitalWrite(leftMotorPins[3], l2);
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "lift1") {
            moveLift(2000);
        } else if (command == "lift2") {
            moveLift(4000);
        } else if (command == "liftgnd") {
            while (digitalRead(limitSwitch) == HIGH) {
                digitalWrite(stepperIn3, LOW);
                digitalWrite(stepperIn4, HIGH);
            }
            digitalWrite(stepperIn3, LOW);
            digitalWrite(stepperIn4, LOW);
            liftDown = true;
        } else {
            int speed = 0;
            int spaceIndex = command.indexOf(' ');
            if (spaceIndex != -1) {
                speed = command.substring(spaceIndex + 1).toInt();
                command = command.substring(0, spaceIndex);
            }

            if (command == "fw") {
                moveRobot(speed, HIGH, LOW, HIGH, LOW);
            } else if (command == "bw") {
                moveRobot(speed, LOW, HIGH, LOW, HIGH);
            } else if (command == "rs") {
                moveRobot(speed, LOW, HIGH, HIGH, LOW);
            } else if (command == "ls") {
                moveRobot(speed, HIGH, LOW, LOW, HIGH);
            } else if (command == "flip" || command == "left") {
                moveRobot(speed, LOW, HIGH, HIGH, LOW);
                delay(2000); // Adjust for 180-degree turn based on speed
                moveRobot(0, LOW, LOW, LOW, LOW);
            } else if (command == "right") {
                moveRobot(speed, HIGH, LOW, LOW, HIGH);
                delay(2000); // Adjust for 180-degree turn based on speed
                moveRobot(0, LOW, LOW, LOW, LOW);
            } else {
                moveRobot(0, LOW, LOW, LOW, LOW);
                digitalWrite(stepperIn3, LOW);
                digitalWrite(stepperIn4, LOW);
            }
        }
    }
}
