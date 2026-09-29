#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>

// ============================================================
// NETWORK
// ============================================================
#define WIFI_SSID  "oni"
#define WIFI_PASS  "ahmedsidi"
#define AGENT_IP   "10.236.95.81"   // Pi's IP on the hotspot
#define AGENT_PORT 8888

// ============================================================
// PINS
// ============================================================
const int MOTOR_IN1 = 26;
const int MOTOR_IN2 = 27;
const int MOTOR_ENA = 25;

const int ENCODER_A = 33;
const int ENCODER_B = 32;

const int SERVO_PIN = 14;

// ============================================================
// LIMITS
// ============================================================
const int SERVO_MIN    = 30;   // max RIGHT
const int SERVO_CENTER = 50;
const int SERVO_MAX    = 70;   // max LEFT

const int MOTOR_MIN_PWM = 100; // minimum PWM that moves the car under load
const unsigned long CMD_TIMEOUT_MS = 500;  // failsafe
const unsigned long ENCODER_PERIOD_MS = 50; // 20 Hz
const unsigned long IMU_PERIOD_MS = 20;     // 50 Hz

// Dedicated LEDC channel for the motor (away from the servo)
const int MOTOR_PWM_CH   = 4;
const int MOTOR_PWM_FREQ = 20000;
const int MOTOR_PWM_RES  = 8;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    #define MOTOR_PWM_SETUP() ledcAttachChannel(MOTOR_ENA, MOTOR_PWM_FREQ, MOTOR_PWM_RES, MOTOR_PWM_CH)
    #define MOTOR_PWM_WRITE(x) ledcWrite(MOTOR_ENA, (x))
#else
    #define MOTOR_PWM_SETUP() do { \
        ledcSetup(MOTOR_PWM_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES); \
        ledcAttachPin(MOTOR_ENA, MOTOR_PWM_CH); \
    } while (0)
    #define MOTOR_PWM_WRITE(x) ledcWrite(MOTOR_PWM_CH, (x))
#endif

// ============================================================
// GLOBALS
// ============================================================
Servo servo;
Adafruit_BNO055 bno(55, 0x28, &Wire);
bool bnoOk = false;

volatile long encoderCount = 0;
int currentMotorPWM = 0;
unsigned long lastMotorCmdMs = 0;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_subscription_t motor_sub;
rcl_subscription_t servo_sub;
rcl_publisher_t encoder_pub;
rcl_publisher_t imu_pub;

std_msgs__msg__Int16 motor_msg;
std_msgs__msg__Int16 servo_msg;
std_msgs__msg__Int32 encoder_msg;
sensor_msgs__msg__Imu imu_msg;

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agentState = WAITING_AGENT;

#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)

// ============================================================
// ENCODER
// ============================================================
void IRAM_ATTR encoderISR()
{
    bool A = digitalRead(ENCODER_A);
    bool B = digitalRead(ENCODER_B);
    if (A == B) encoderCount++;
    else        encoderCount--;
}

void publishEncoder()
{
    noInterrupts();
    long count = encoderCount;
    interrupts();

    encoder_msg.data = (int32_t)count;
    rcl_publish(&encoder_pub, &encoder_msg, NULL);
}

// ============================================================
// IMU
// ============================================================
void publishImu()
{
    if (!bnoOk) return;

    imu::Quaternion q = bno.getQuat();
    imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);   // deg/s (check your lib)
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); // m/s^2

    imu_msg.orientation.w = q.w();
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();

    imu_msg.angular_velocity.x = gyro.x() * DEG_TO_RAD;
    imu_msg.angular_velocity.y = gyro.y() * DEG_TO_RAD;
    imu_msg.angular_velocity.z = gyro.z() * DEG_TO_RAD;

    imu_msg.linear_acceleration.x = accel.x();
    imu_msg.linear_acceleration.y = accel.y();
    imu_msg.linear_acceleration.z = accel.z();

    rcl_publish(&imu_pub, &imu_msg, NULL);
}

// ============================================================
// MOTOR
// ============================================================
void setMotorRaw(int pwm)
{
    pwm = constrain(pwm, -255, 255);

    if (pwm == 0)
    {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        MOTOR_PWM_WRITE(0);
    }
    else if (pwm > 0)
    {
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        MOTOR_PWM_WRITE(pwm);
    }
    else
    {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        MOTOR_PWM_WRITE(-pwm);
    }
    currentMotorPWM = pwm;
}

// cmd: -255..255. 0 = stop, otherwise |cmd| 1..255 is scaled to MOTOR_MIN_PWM..255
void setMotorCmd(int cmd)
{
    cmd = constrain(cmd, -255, 255);
    if (cmd == 0) { setMotorRaw(0); return; }

    int mag = abs(cmd);
    int pwm = MOTOR_MIN_PWM + (mag - 1) * (255 - MOTOR_MIN_PWM) / 254;
    setMotorRaw(cmd > 0 ? pwm : -pwm);
}

// ============================================================
// SERVO
// ============================================================
void setServo(int angle)
{
    servo.write(constrain(angle, SERVO_MIN, SERVO_MAX));
}

// ============================================================
// micro-ROS CALLBACKS
// ============================================================
void motorCallback(const void *msgin)
{
    const std_msgs__msg__Int16 *m = (const std_msgs__msg__Int16 *)msgin;
    lastMotorCmdMs = millis();
    setMotorCmd(m->data);
}

void servoCallback(const void *msgin)
{
    const std_msgs__msg__Int16 *m = (const std_msgs__msg__Int16 *)msgin;
    setServo(m->data);
}

// ============================================================
// micro-ROS ENTITIES
// ============================================================
bool createEntities()
{
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "esp32_node", "", &support) != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(&motor_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/esp32/motor_cmd") != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(&servo_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/esp32/servo_cmd") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(&encoder_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/esp32/encoder") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(&imu_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/esp32/imu") != RCL_RET_OK) return false;

    // 2 handles: two subscriptions, no timer
    if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(&executor, &motor_sub, &motor_msg, &motorCallback, ON_NEW_DATA) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servoCallback, ON_NEW_DATA) != RCL_RET_OK) return false;

    return true;
}

void destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_subscription_fini(&motor_sub, &node);
    rcl_subscription_fini(&servo_sub, &node);
    rcl_publisher_fini(&encoder_pub, &node);
    rcl_publisher_fini(&imu_pub, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ============================================================
// SETUP
// ============================================================
void setup()
{
    Serial.begin(115200);

    // Servo first, reserving its LEDC timer
    ESP32PWM::allocateTimer(0);
    servo.setPeriodHertz(50);
    servo.attach(SERVO_PIN, 500, 2400);
    setServo(SERVO_CENTER);

    // Motor
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    MOTOR_PWM_SETUP();
    setMotorRaw(0);

    // Encoder
    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

    // IMU
    Wire.begin(21, 22);
    bnoOk = bno.begin();
    if (bnoOk)
    {
        delay(1000);
        bno.setExtCrystalUse(true);
    }
    // Mark orientation/velocity/accel covariance as "unknown" (first element = -1)
    imu_msg.orientation_covariance[0] = -1;
    imu_msg.angular_velocity_covariance[0] = -1;
    imu_msg.linear_acceleration_covariance[0] = -1;

    // WiFi transport to the micro-ROS agent
    set_microros_wifi_transports((char *)WIFI_SSID, (char *)WIFI_PASS, (char *)AGENT_IP, AGENT_PORT);
    WiFi.setSleep(false);   // lower latency

    agentState = WAITING_AGENT;
}

// ============================================================
// LOOP
// ============================================================
void loop()
{
    // Failsafe: stop the motor if commands stop arriving
    if (currentMotorPWM != 0 && millis() - lastMotorCmdMs > CMD_TIMEOUT_MS)
    {
        setMotorRaw(0);
    }

    switch (agentState)
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500,
                Serial.println("connecting...");
                agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;

        case AGENT_AVAILABLE:
            if (createEntities())
            {
                agentState = AGENT_CONNECTED;
                Serial.println("micro-ROS agent connected");
            }
            else
            {
                destroyEntities();
                agentState = WAITING_AGENT;
            }
            break;

        case AGENT_CONNECTED:
        {
            static unsigned long lastPing = 0, lastPub = 0, lastImuPub = 0;

            if (millis() - lastPing >= 1000)
            {
                lastPing = millis();
                agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            }

            if (agentState == AGENT_CONNECTED)
            {
                if (millis() - lastPub >= ENCODER_PERIOD_MS)
                {
                    lastPub = millis();
                    publishEncoder();
                }

                if (millis() - lastImuPub >= IMU_PERIOD_MS)
                {
                    lastImuPub = millis();
                    publishImu();
                }

                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
            }
            break;
        }

        case AGENT_DISCONNECTED:
            setMotorRaw(0);                 // never keep driving blind
            destroyEntities();
            agentState = WAITING_AGENT;
            break;
    }
}