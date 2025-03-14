#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Constants
const int PWM_FREQ = 1000;
const int I2C_SLAVE_ADDR = 0x42;
const float WHEEL_RADIUS = 0.05;
const float ROBOT_WIDTH = 0.2;
const float ROBOT_LENGTH = 0.2;
const int ENCODER_RESOLUTION = 1000;

// Motor pins
const int MOTOR_FL_ENC_A = 2;
const int MOTOR_FL_ENC_B = 3;
const int MOTOR_FL_PWM = 4;
const int MOTOR_FL_DIR_A = 5;
const int MOTOR_FL_DIR_B = 6;
const int MOTOR_FL_DIR_MULT = -1;

const int MOTOR_RL_PWM = 9;
const int MOTOR_RL_DIR_A = 7;
const int MOTOR_RL_DIR_B = 8;
const int MOTOR_RL_ENC_A = 10;
const int MOTOR_RL_ENC_B = 11;
const int MOTOR_RL_DIR_MULT = 1;

const int MOTOR_RR_PWM = 18;
const int MOTOR_RR_DIR_A = 19;
const int MOTOR_RR_DIR_B = 20;
const int MOTOR_RR_ENC_A = 16;
const int MOTOR_RR_ENC_B = 17;
const int MOTOR_RR_DIR_MULT = 1;

const int MOTOR_FR_PWM = 26;
const int MOTOR_FR_DIR_A = 21;
const int MOTOR_FR_DIR_B = 22;
const int MOTOR_FR_ENC_A = 27;
const int MOTOR_FR_ENC_B = 28;
const int MOTOR_FR_DIR_MULT = -1;

// Encoder counts
volatile int encoder_counts_A[4] = {0, 0, 0, 0};

// Motor class
class Motor {
public:
    Motor(int pwm_pin, int dir_pin_A, int dir_pin_B, int direction)
        : pwm_pin(pwm_pin), dir_pin_A(dir_pin_A), dir_pin_B(dir_pin_B), direction(direction) {
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin_A, OUTPUT);
        pinMode(dir_pin_B, OUTPUT);
    }

    void set_speed(float speed) {
        speed *= direction;
        if (speed >= 0) {
            digitalWrite(dir_pin_A, HIGH);
            digitalWrite(dir_pin_B, LOW);
            analogWrite(pwm_pin, int(speed * 255));  // Correct usage for Pico
        } else {
            digitalWrite(dir_pin_A, LOW);
            digitalWrite(dir_pin_B, HIGH);
            analogWrite(pwm_pin, int(-speed * 255));  // Correct usage for Pico
        }
    }

private:
    int pwm_pin;
    int dir_pin_A;
    int dir_pin_B;
    int direction;
};

// Motor instances
Motor motorFL(MOTOR_FL_PWM, MOTOR_FL_DIR_A, MOTOR_FL_DIR_B, MOTOR_FL_DIR_MULT);
Motor motorRL(MOTOR_RL_PWM, MOTOR_RL_DIR_A, MOTOR_RL_DIR_B, MOTOR_RL_DIR_MULT);
Motor motorRR(MOTOR_RR_PWM, MOTOR_RR_DIR_A, MOTOR_RR_DIR_B, MOTOR_RR_DIR_MULT);
Motor motorFR(MOTOR_FR_PWM, MOTOR_FR_DIR_A, MOTOR_FR_DIR_B, MOTOR_FR_DIR_MULT);

// Move function
void move(float x_speed, float y_speed, float rotation_speed) {
    float fl_speed = x_speed + y_speed + rotation_speed;
    float rl_speed = x_speed - y_speed - rotation_speed;
    float rr_speed = x_speed + y_speed - rotation_speed;
    float fr_speed = x_speed - y_speed + rotation_speed;

    float max_speed = max(max(abs(fl_speed), abs(rl_speed)), max(abs(rr_speed), abs(fr_speed)));
    if (max_speed > 1) {
        fl_speed /= max_speed;
        rl_speed /= max_speed;
        rr_speed /= max_speed;
        fr_speed /= max_speed;
    }

    motorFL.set_speed(fl_speed);
    motorRL.set_speed(rl_speed);
    motorRR.set_speed(rr_speed);
    motorFR.set_speed(fr_speed);
}

// Calculate distance function
float calculate_distance(float target_x, float target_y, float current_x, float current_y) {
    return sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
}

// Ramp speed function
float ramp_speed(float current_speed, float target_speed, float ramp_rate) {
    if (abs(current_speed - target_speed) < ramp_rate) {
        return target_speed;
    } else if (current_speed < target_speed) {
        return current_speed + ramp_rate;
    } else {
        return current_speed - ramp_rate;
    }
}

// Go to position function
void go_to_position(float target_x, float target_y, float ramp_rate = 0.1, float max_speed = 1.0) {
    float current_x = 0, current_y = 0;
    float current_speed = 0;

    while (calculate_distance(target_x, target_y, current_x, current_y) > 0.01) {
        float distance = calculate_distance(target_x, target_y, current_x, current_y);
        float angle_to_target = atan2(target_y - current_y, target_x - current_x);
        float x_speed = cos(angle_to_target) * current_speed;
        float y_speed = sin(angle_to_target) * current_speed;

        float target_speed = min(max_speed, distance);
        current_speed = ramp_speed(current_speed, target_speed, ramp_rate);

        move(x_speed, y_speed, 0);
        delay(100);

        current_x += x_speed * 0.1;
        current_y += y_speed * 0.1;
    }

    move(0, 0, 0);
}

// Process instruction function
void process_instruction(String instruction) {
    int space1 = instruction.indexOf(' ');
    int space2 = instruction.indexOf(' ', space1 + 1);
    int space3 = instruction.indexOf(' ', space2 + 1);

    if (instruction[0] == 'M' && space1 > 0 && space2 > 0 && space3 > 0) {
        float x_speed = instruction.substring(space1 + 1, space2).toFloat();
        float y_speed = instruction.substring(space2 + 1, space3).toFloat();
        float rotation_speed = instruction.substring(space3 + 1).toFloat();
        move(x_speed, y_speed, rotation_speed);
    } else {
        Serial.println("Unknown instruction");
    }
}

void setup() {
    // Initialisation série pour le diagnostic
    Serial.begin(115200);

    // Initialisation des moteurs
    motorFL.set_speed(0);
    motorRL.set_speed(0);
    motorRR.set_speed(0);
    motorFR.set_speed(0);

    // Test du moteur avant gauche
    motorFL.set_speed(100);
    delay(1000);  // Teste pendant 1 seconde
    motorFL.set_speed(0);
    delay(500);

    // Test du moteur arrière gauche
    motorRL.set_speed(100);
    delay(1000);  // Teste pendant 1 seconde
    motorRL.set_speed(0);
    delay(500);

    // Test du moteur arrière droit
    motorRR.set_speed(100);
    delay(1000);  // Teste pendant 1 seconde
    motorRR.set_speed(0);
    delay(500);

    // Test du moteur avant droit
    motorFR.set_speed(100);
    delay(1000);  // Teste pendant 1 seconde
    motorFR.set_speed(0);
    delay(500);
}

void loop() {
    // Le loop peut être vide si tu fais juste un test unique dans setup()
}


