const int rightMotorPins[4] = {2, 3, 4, 5};
const int leftMotorPins[4] = {11, 10, 12, 13};
const int motorEnable = 6;

const int stepperIn3 = 7;
const int stepperIn4 = 8;

const int encoderA = A0;
const int encoderB = A5;

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
    digitalWrite(stepperIn3, LOW);
    digitalWrite(stepperIn4, LOW);
    liftDown = true;
}

void moveLift(int duration) {
    digitalWrite(stepperIn3, HIGH);
    digitalWrite(stepperIn4, LOW);
    delay(duration);
    digitalWrite(stepperIn3, LOW);
    digitalWrite(stepperIn4, LOW);
}

void moveRobot(int speed, int r1, int r2, int l1, int l2) {
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
    moveLift(4000);
    moveRobot(150, HIGH, LOW, HIGH, LOW);
    delay(2200);
    moveRobot(0, LOW, LOW, LOW, LOW);
    delay(200);
    moveLift(800);
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

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command == "lift1") {
            moveLift(2000);
        } else if (command == "lift2") {
            moveLift(4000);
        } else if (command == "liftgnd") {
            moveLiftToGround();
        } else if (command == "stop" || command == "s" ) {
            moveRobot(0, LOW, LOW, LOW, LOW);
            digitalWrite(stepperIn3, LOW);
            digitalWrite(stepperIn4, LOW);
        } else if (command == "gos") {
            grabOffShelf();
        } else {
            int spaceIndex = command.indexOf(':');
            if (spaceIndex != -1) {
                String cmdType = command.substring(0, spaceIndex);
                int speed = command.substring(spaceIndex + 1).toInt();
                if (cmdType == "lift") {
                    moveLift(speed);
                } else if (cmdType == "forwards") {
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