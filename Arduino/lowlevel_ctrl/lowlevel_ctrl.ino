#include <stdint.h>
#include "MsTimer2.h"
#include "Wire.h"  /* required for I2C of MPU6050 */
#include "MPU6050.h"
#include "Motor.h"
#include "PinChangeInt.h"
// #include "rgb.h"

MPU6050 MPU6050;
Motor Motor;

#define DT_MS     10
#define VOL_MEASURE_PIN A2

unsigned long Motor::encoder_count_left_a; // encoder ticks
unsigned long Motor::encoder_count_right_a; // encoder ticks
static int g_encoder_delta_left; // +/- pos/neg. encoder ticks since last frame
static int g_encoder_delta_right; // +/- pos/neg. encoder ticks since last frame
static int g_pwm_left;
static int g_pwm_right;
static unsigned long g_vol_measure_time = 0;
static int g_voltage = 0; // voltage in millivolts
static unsigned long g_last_output;
static int g_hz;
static int g_hz_input;
static unsigned g_epoch;
static unsigned g_epochInput;
static unsigned long g_last_hz;

static void EncoderCountRightA() //Getting Right Wheel Speed.
{
  Motor::encoder_count_right_a++;
  g_encoder_delta_right += g_pwm_right < 0 ? -1 : 1;
}
static void EncoderCountLeftA() //Getting Left Wheel Speed.
{
  Motor::encoder_count_left_a++;
  g_encoder_delta_left += g_pwm_left < 0 ? -1 : 1;
}

void Motor::Encoder_init()
{
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), EncoderCountLeftA, CHANGE);
  attachPinChangeInterrupt(ENCODER_RIGHT_A_PIN, EncoderCountRightA, CHANGE);
}

void interrupt()
{
  g_epoch++;
  sei();//enable the global interrupt
  int16_t ax, ay, az, gx, gy, gz;
  char textbuf[128];

  MPU6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  const int dt_ms = millis() - g_last_output;
  g_last_output = millis();
  sprintf(textbuf, "L %02i %i %i %i %i %i %i %i %i %i %i", dt_ms, (int)ax, (int)ay, (int)az, (int)gx, (int)gy, (int)gz, g_encoder_delta_left, g_encoder_delta_right, (int)g_voltage, (int)g_epochInput);
  g_encoder_delta_left = 0;
  g_encoder_delta_right = 0;
  Serial.println(textbuf);
  Serial.flush();

  if (millis() - g_last_hz >= 1000)
  {
    g_last_hz = millis();
    g_hz = g_epoch;
    g_hz_input = g_epochInput;
    g_epoch = 0;
    g_epochInput = 0;
  }
}

void motor_control(int pwm_left, int pwm_right)
{
  g_pwm_left = constrain(pwm_left, -255, 255);
  g_pwm_right = constrain(pwm_right, -255, 255);

  (g_pwm_left < 0) ?  (Motor.Control(AIN1,1,PWMA_LEFT,-g_pwm_left)):
                      (Motor.Control(AIN1,0,PWMA_LEFT,g_pwm_left));

  (g_pwm_right < 0) ? (Motor.Control(BIN1,1,PWMB_RIGHT,-g_pwm_right)):
                      (Motor.Control(BIN1,0,PWMB_RIGHT,g_pwm_right));

}

void setup()
{
  // rgb.initialize();

  analogReference(INTERNAL); // voltage measurement

  Motor.Pin_init(); // Motor control
  Motor.Encoder_init(); // Motor odometry
  Wire.begin(); // I2C for MPU6050
  MPU6050.initialize();
  MPU6050.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
  MPU6050.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  MPU6050.setDLPFMode(MPU6050_DLPF_BW_42);
  Serial.begin(250000);

  MsTimer2::set(DT_MS, interrupt);
  g_last_output = millis();
  MsTimer2::start();
}

static char inputBuffer[64];
static int inputIndex;
static bool inputBufferStart;
static unsigned long inputLastReceived; // timestamp of last complete message
void loop()
{
  if (millis() - inputLastReceived > 1000) /* after 1 sec. of no input, shut off motors */
  {
    motor_control(0, 0);
  }

  if(millis() - g_vol_measure_time >= 5000)
  {
    g_vol_measure_time = millis();
    double voltage = (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5); //Read voltage value
    g_voltage = (int)(voltage*10.0);
  }

  while (Serial.available() > 0)
  {
    char rb = Serial.read();
    if (inputBufferStart)
    {
      if (rb == '>') // message complete
      {
        inputBuffer[inputIndex++] = '\0';
        // consume message
        const char* t1 = strtok(inputBuffer, ",");
        const char* t2 = strtok(NULL, ",");
        int pwm_left = atoi(t1);
        int pwm_right = atoi(t2);
        if (pwm_left < -255 || pwm_left > 255 || pwm_right < -255 || pwm_right > 255 || t2 == NULL || t1 == NULL)
        {
          pwm_left = pwm_right = 0; // invalid text message, better set the motors to zero.
        }

        motor_control(pwm_left, pwm_right);
        inputLastReceived = millis();
        inputBufferStart = false;
        g_epochInput++;
      }
      else
      {
        inputBuffer[inputIndex++] = rb;
        if (inputIndex >= sizeof(inputBuffer))
        {
          inputIndex = 0;
          inputBufferStart = false;
        }
      }
    }
    else if (rb == '<') // sync byte found
    {
      inputBufferStart = true;
      inputIndex = 0;
    }
    else
    {

    }
  }


}
