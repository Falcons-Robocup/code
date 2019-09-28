/*
Author : Sofia Ntella
Date     : 11 September 2018

Goalkeeper frame extraction left, up or right

Conventions:
RIGHT = INH_1
UP    = INH_2
LEFT    = INH_3
*/
int IN_RIGHT = A7;     // ENABLE_1
int IN_UP    = 4;    // ENABlE_2
int IN_LEFT    = 7;    // ENABLE_3

int INH_1 = 2;     // enables right actuator
int INH_2 = 8;     // enables up actuator
int INH_3 = 12;    // enables left actuator

// PWM pins 
int IN_1A    = 3;    
int IN_1B    = 5;
int IN_2A    = 6;
int IN_2B    = 9;
int IN_3A    = 10;
int IN_3B    = 11;

void frame_out(int INH, int IN_A, int IN_B);
void frame_in(int INH, int IN_A, int IN_B);

// the setup function runs once when you press reset or power the board
void setup() {
    pinMode(IN_UP,INPUT);
    pinMode(IN_LEFT,INPUT);
    pinMode(IN_RIGHT,INPUT);
    
    pinMode(INH_1, OUTPUT);
    pinMode(INH_2, OUTPUT);
    pinMode(INH_3, OUTPUT);

    pinMode(IN_1A, OUTPUT);
    pinMode(IN_1B, OUTPUT);
    pinMode(IN_2A, OUTPUT);
    pinMode(IN_2B, OUTPUT);
    pinMode(IN_3A, OUTPUT);
    pinMode(IN_3B, OUTPUT);

    // initialize serial communication at 9600 bits per second:
    Serial.begin(9600);
    Serial.println("--- Start Serial Monitor SEND_RCVE ---");
}

// the loop function runs over and over again forever
void loop() {
    if (analogRead(IN_RIGHT) < 512) {
        frame_out(INH_1, IN_1A, IN_1B);
        while(analogRead(IN_RIGHT) < 512) {
            continue;
        }
        frame_in(INH_1, IN_1A, IN_1B);
    }

    else if (digitalRead(IN_UP) == LOW) {
    
        frame_out(INH_2, IN_2A, IN_2B);
        while(digitalRead(IN_UP) == LOW) {
            continue;
        }
        frame_in(INH_2, IN_2A, IN_2B);
    }
    
    else if (digitalRead(IN_LEFT) == LOW) {
        frame_out(INH_3, IN_3A, IN_3B);
        while(digitalRead(IN_LEFT) == LOW) {
            continue;
        }
        frame_in(INH_3, IN_3A, IN_3B);
    }
}

void frame_out(int INH, int IN_A, int IN_B) {
    digitalWrite(INH, 1);
    analogWrite(IN_A, 245);
    analogWrite(IN_B, 0);
    delay(1000);
}
void frame_in(int INH, int IN_A, int IN_B) {
    digitalWrite(INH,1);
    analogWrite(IN_A, 0);
    analogWrite(IN_B, 150);
    delay(500);
    analogWrite(IN_B, 50);
    delay(1000);
    analogWrite(IN_B, 0);
}
