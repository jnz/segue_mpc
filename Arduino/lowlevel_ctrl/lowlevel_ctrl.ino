/**

    +-------------------------------+
    |                               |
    |  Raspberry Pi                 |
    |                               |
    |  Running high-level control   +-----------+ WiFi connection
    |                               |
    |                               +-----------+ Dedicated Battery
    +----------+--------------------+
               |
               | 250000 baud serial connection
               | To Raspberry at 50 Hz:
               |    - IMU data (accelerometer and gyroscope)
               |    - Wheel ticks (left/right)
               |
               | Read in from Raspberry Pi:
               |    - Motor setpoint speed
               |
    +----------+------------+
    |                       | I²C +----------+
    |  Arduino (This Code)  +-----| IMU      |
    |                       |     +----------+
    |                       |     +------------------------------------------+
    |                       +-----| Battery, Ultrasonic dist., RGB leds, ... |
    |                       |     +------------------------------------------+
    +----------+-------+----+
               |       | PWM
               |       | Discretes for wheel ticks
    +----------+       +------------+
    |Motor left|       | Motor right|
    +----------+       +------------+
*
*/

/******************************************************************************
 * SYSTEM INCLUDE FILES
 ******************************************************************************/

#include <stdint.h>

/******************************************************************************
 * PROJECT INCLUDE FILES
 ******************************************************************************/

/* Elegoo Tumbller Arduino libraries */
#include "MsTimer2.h" /* call the interrupt() function at a fixed interval */
#include "Wire.h"  /* required for I2C of MPU6050 */
#include "MPU6050.h" /* Invensense MPU6050 IMU (accelerometer + gyroscope) */
#include "Motor.h" /* Elegoo Tumbller motor control */
#include "PinChangeInt.h"
// #include "rgb.h" /* Elegoo Tumbller rgb LED control */

/******************************************************************************
 * DEFINES
 ******************************************************************************/

#define DT_MS               20   /* call interrupt with 50 Hz */
#define VOL_MEASURE_PIN     A2   /* battery voltage level of Elegoo Tumbller */

/******************************************************************************
 * GLOBAL DATA DEFINITIONS
 ******************************************************************************/

MPU6050 MPU6050; /**< helper class to read IMU data via IIC */
Motor Motor; /**< helper class for PWM motor control */

/* Wheel odometry tick counter:
   one tick = 0.00027195 meter distance traveled */
unsigned long Motor::encoder_count_left_a; /**< left wheel encoder ticks */
unsigned long Motor::encoder_count_right_a; /**< right wheel encoder ticks */

/******************************************************************************
 * LOCAL DATA DEFINITIONS
 ******************************************************************************/

static int g_encoder_delta_left; // +/- pos/neg. encoder ticks since last frame
static int g_encoder_delta_right; // +/- pos/neg. encoder ticks since last frame
static int g_pwm_left; /* current left motor command setpoint (-255 to 255) */
static int g_pwm_right; /* current motor command setpoint (-255 to 255) */
static unsigned long g_vol_measure_time = 0; /* timestamp last measurement */
static int g_voltage = 0; /* last voltage measurement (scaled by *10) */
static unsigned long g_last_output; /* timestamp of the last UART output message (to RPi) */

static int g_hz; /* monitor main interrupt function frequency in Hz */
static int g_hz_input; /* monitor UART input message frequency from Raspberry Pi in Hz */
static unsigned g_epoch; /* Counter to calculate g_hz */
static unsigned g_epochInput; /* Counter to calculate g_hz_input */
static unsigned long g_timestamp_last_hz; /* Timestamp of last g_hz and g_hz_input update */

/* Helper variables for motor controlinput messages from the Raspberry Pi host: */
static char inputBuffer[64]; /* input bytes from UART connection */
static int inputIndex; /* position in inputBuffer[...] */
static bool inputBufferStart; /* sync byte found in input stream */
static unsigned long inputLastReceived; /* timestamp of last complete input message */

/******************************************************************************
 * FUNCTION BODIES
 ******************************************************************************/

/* Interrupt callback for left wheel tick */
static void EncoderCountLeftA() //Getting Left Wheel Speed.
{
    Motor::encoder_count_left_a++;
    g_encoder_delta_left += g_pwm_left < 0 ? -1 : 1;
}

/* Interrupt callback for right wheel tick */
static void EncoderCountRightA()
{
    Motor::encoder_count_right_a++;
    g_encoder_delta_right += g_pwm_right < 0 ? -1 : 1;
}

/* Setup wheel tick interrupt callbacks */
void Motor::Encoder_init()
{
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), EncoderCountLeftA, CHANGE);
    attachPinChangeInterrupt(ENCODER_RIGHT_A_PIN, EncoderCountRightA, CHANGE);
}

/* Basic main loop running at 1/DT_SEC Hz
    Sending sensor data via UART to Raspberry Pi Host.  */
void interrupt()
{
    int16_t ax, ay, az, gx, gy, gz; /* accelerometer and gyroscope data in body frame */
    char textbuf[128]; /* UART send buffer */

    sei(); /* enable the global interrupt */
    g_epoch++;
    MPU6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); /* latest IMU data */

    const int dt_ms = millis() - g_last_output; /* time since last UART message */
    g_last_output = millis();
    sprintf(textbuf, "L %02i %i %i %i %i %i %i %i %i %i %i", dt_ms, (int)ax, (int)ay, (int)az, (int)gx, (int)gy, (int)gz, g_encoder_delta_left, g_encoder_delta_right, (int)g_voltage, (int)g_hz_input);
    g_encoder_delta_left = 0;   /* only send wheel ticks since last message */
    g_encoder_delta_right = 0;  /* potential issue on lost UART messages? */
    Serial.println(textbuf);    /* send to the wire */
    Serial.flush();             /* make sure it goes to the Raspberry Pi ASAP */

    /* calculate input/output message frequency to monitor real-time behavior */
    if (millis() - g_timestamp_last_hz >= 1000)
    {
        g_timestamp_last_hz = millis();
        g_hz = g_epoch;
        g_hz_input = g_epochInput;
        g_epoch = 0;
        g_epochInput = 0;
    }
}

/* Set the motor speed for the left and right motor.
    @param[in] pwm_left value between -255 and 255.
    @param[in] pwm_right value between -255 and 255. */
void motor_control(int pwm_left, int pwm_right)
{
    g_pwm_left  = constrain(pwm_left,  -255, 255);
    g_pwm_right = constrain(pwm_right, -255, 255);

    (g_pwm_left < 0) ? (Motor.Control(AIN1,1,PWMA_LEFT,-g_pwm_left)):
        (Motor.Control(AIN1,0,PWMA_LEFT,g_pwm_left));

    (g_pwm_right < 0) ? (Motor.Control(BIN1,1,PWMB_RIGHT,-g_pwm_right)):
        (Motor.Control(BIN1,0,PWMB_RIGHT,g_pwm_right));
}

void setup()
{
    /* rgb.initialize(); */
    analogReference(INTERNAL); /* setup pin for voltage measurement */

    Motor.Pin_init();
    Motor.Encoder_init();
    Wire.begin(); /* prepare I2C for MPU6050 IMU */
    MPU6050.initialize();
    MPU6050.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    MPU6050.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    MPU6050.setDLPFMode(MPU6050_DLPF_BW_42);

    Serial.begin(250000); /* Raspberry Pi communication UART */

    MsTimer2::set(DT_MS, interrupt); /* call interrupt function every DT_MS ms */
    g_last_output = millis();
    MsTimer2::start();
}

void loop()
{
    if (millis() - inputLastReceived > 1000) /* after 1 sec. of no input, shut off motors */
    {
        motor_control(0, 0);
    }

    /* Read battery voltage level every 5 seconds and store result in "g_voltage" */
    if(millis() - g_vol_measure_time >= 5000)
    {
        g_vol_measure_time = millis();
        const double voltage =
            (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5);
        g_voltage = (int)(voltage*10.0); /* scale by 10 and store as integer */
    }

    /* parse Raspberry Pi motor input commands from UART */
    while (Serial.available() > 0)
    {
        char rb = Serial.read();
        if (inputBufferStart) /* sync byte found? */
        {
            if (rb == '>') /* is message complete? */
            {
                /* message format on wire: "<RPM_LEFT,RPM_RIGHT>" */
                inputBuffer[inputIndex++] = '\0';
                const char* t1 = strtok(inputBuffer, ",");
                const char* t2 = strtok(NULL, ",");
                int pwm_left = atoi(t1);
                int pwm_right = atoi(t2);
                if (pwm_left < -255 || pwm_left > 255 || pwm_right < -255 || pwm_right > 255 ||
                    t2 == NULL || t1 == NULL)
                {
                    /* invalid text message, better set the motor commands to zero. */
                    pwm_left = pwm_right = 0;
                }

                motor_control(pwm_left, pwm_right);
                inputLastReceived = millis();
                inputBufferStart = false;
                g_epochInput++; /* +1 valid message from Raspberry */
            }
            else
            {
                /* store byte within message */
                inputBuffer[inputIndex++] = rb;
                /* prevent input buffer overflow if input data is random */
                if (inputIndex >= sizeof(inputBuffer))
                {
                    inputIndex = 0;
                    inputBufferStart = false; /* wait for sync byte */
                }
            }
        }
        else if (rb == '<') /* sync byte found */
        {
            inputBufferStart = true;
            inputIndex = 0;
        }
        else
        {
            /* ignore data, nothing to do */
        }
    }
}

