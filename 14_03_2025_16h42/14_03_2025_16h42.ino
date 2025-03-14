#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ---------------- Constantes ----------------
const float WHEEL_DIAMETER = 0.065; // en mètres (65 mm)
const int ENCODER_RESOLUTION = 6533; // Impulsions par tour sortie
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159; // Périmètre de roue
const float DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / ENCODER_RESOLUTION; // mm par impulsion
const float TARGET_DISTANCE = 0.10; // 10 cm en mètres
const int TARGET_TICKS = 800; // Correction du nombre de ticks

// ---------------- Déclaration des broches ----------------
// Encodeurs
const int MOTOR_FL_ENC_A = 2, MOTOR_FL_ENC_B = 3;
const int MOTOR_RL_ENC_A = 10, MOTOR_RL_ENC_B = 11;
const int MOTOR_RR_ENC_A = 16, MOTOR_RR_ENC_B = 17;
const int MOTOR_FR_ENC_A = 27, MOTOR_FR_ENC_B = 28;

// Moteurs
const int MOTOR_FL_PWM = 4, MOTOR_FL_DIR_A = 5, MOTOR_FL_DIR_B = 6;
const int MOTOR_RL_PWM = 9, MOTOR_RL_DIR_A = 7, MOTOR_RL_DIR_B = 8;
const int MOTOR_RR_PWM = 18, MOTOR_RR_DIR_A = 19, MOTOR_RR_DIR_B = 20;
const int MOTOR_FR_PWM = 26, MOTOR_FR_DIR_A = 21, MOTOR_FR_DIR_B = 22;

// Variables encodeurs
volatile int encoder_FL = 0, encoder_RL = 0, encoder_RR = 0, encoder_FR = 0;

// ---------------- Classes ----------------
class Motor {
public:
    Motor(int pwm_pin, int dir_pin_A, int dir_pin_B, bool invert = false)
        : pwm_pin(pwm_pin), dir_pin_A(dir_pin_A), dir_pin_B(dir_pin_B), invert(invert) {
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin_A, OUTPUT);
        pinMode(dir_pin_B, OUTPUT);
    }

    void set_speed(float speed) {
        if (invert) speed = -speed; // Inversion si nécessaire
        
        if (speed >= 0) {
            digitalWrite(dir_pin_A, HIGH);
            digitalWrite(dir_pin_B, LOW);
        } else {
            digitalWrite(dir_pin_A, LOW);
            digitalWrite(dir_pin_B, HIGH);
            speed = -speed;
        }

        // Limiter la vitesse entre 0 et 255 (intensité PWM)
        int pwm_value = int(speed * 255);
        if (pwm_value < 0) pwm_value = 0;
        if (pwm_value > 255) pwm_value = 255;

        analogWrite(pwm_pin, pwm_value);  
    }

    void stop() { set_speed(0); }

private:
    int pwm_pin, dir_pin_A, dir_pin_B;
    bool invert;
};

// ---------------- Création des moteurs ----------------
// ⚠️ On inverse seulement les moteurs avant (FL et FR)
Motor motorFL(MOTOR_FL_PWM, MOTOR_FL_DIR_A, MOTOR_FL_DIR_B, true);
Motor motorRL(MOTOR_RL_PWM, MOTOR_RL_DIR_A, MOTOR_RL_DIR_B, false);
Motor motorRR(MOTOR_RR_PWM, MOTOR_RR_DIR_A, MOTOR_RR_DIR_B, false);
Motor motorFR(MOTOR_FR_PWM, MOTOR_FR_DIR_A, MOTOR_FR_DIR_B, true);

// ---------------- Interruptions des encodeurs ----------------
void encoderFL_ISR() { encoder_FL++; }
void encoderRL_ISR() { encoder_RL++; }
void encoderRR_ISR() { encoder_RR++; }
void encoderFR_ISR() { encoder_FR++; }

// ---------------- Setup ----------------
void setup() {
    Serial.begin(115200);

    pinMode(MOTOR_FL_ENC_A, INPUT_PULLUP);
    pinMode(MOTOR_RL_ENC_A, INPUT_PULLUP);
    pinMode(MOTOR_RR_ENC_A, INPUT_PULLUP);
    pinMode(MOTOR_FR_ENC_A, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(MOTOR_FL_ENC_A), encoderFL_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RL_ENC_A), encoderRL_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RR_ENC_A), encoderRR_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_FR_ENC_A), encoderFR_ISR, RISING);

    Serial.println("Système prêt !");
}

// ---------------- Boucle principale ----------------
void loop() {
    // Réinitialisation des encodeurs
    encoder_FL = encoder_RL = encoder_RR = encoder_FR = 0;

    // Démarrage des moteurs avec une vitesse initiale de 80%
    motorFL.set_speed(0.8);
    motorRL.set_speed(0.8);
    motorRR.set_speed(0.8);
    motorFR.set_speed(0.8);

    // Distance parcourue initiale
    int avg_ticks = 0;

    // Déplacement avec décélération progressive
    while (avg_ticks < TARGET_TICKS) {
        // Moyenne des encodeurs
        avg_ticks = (encoder_FL + encoder_RL + encoder_RR + encoder_FR) / 4;

        // Calcul de la distance restante en fonction des ticks
        float distance_remaining = (TARGET_TICKS - avg_ticks) * DISTANCE_PER_TICK;

        // Calcul de la vitesse en fonction de la distance restante
        float speed = 0.8 * (distance_remaining / TARGET_DISTANCE);

        // Limiter la vitesse entre 0.3 et 0.8 (pas de trop faibles vitesses)
        if (speed > 0.8) speed = 0.8;
        if (speed < 0.3) speed = 0.3;

        // Appliquer la vitesse aux moteurs
        motorFL.set_speed(speed);
        motorRL.set_speed(speed);
        motorRR.set_speed(speed);
        motorFR.set_speed(speed);

        // Afficher l'état pour le suivi
        Serial.print("Ticks: "); Serial.print(avg_ticks);
        Serial.print(" | Distance restante: "); Serial.print(distance_remaining);
        Serial.print(" | Vitesse: "); Serial.println(speed);

        delay(100);
    }

    // Arrêt des moteurs après avoir atteint la distance
    motorFL.stop();
    motorRL.stop();
    motorRR.stop();
    motorFR.stop();

    Serial.println("Arrêt après 10 cm !");
    while (true);
}
