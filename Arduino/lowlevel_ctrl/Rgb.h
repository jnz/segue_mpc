#include "Adafruit_NeoPixel.h"

#define RGB_PIN 3
#define NUMPIXELS 4

class RGB : public Adafruit_NeoPixel
{
public:
  RGB() : Adafruit_NeoPixel(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800) {}
  uint8_t gamma[256] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2,
      2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5,
      5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10,
      10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
      17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
      25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
      37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
      51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
      69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
      90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
      115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
      144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
      177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
      215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255};
  uint32_t led_rgb_new[NUMPIXELS];
  uint32_t led_rgb_old[NUMPIXELS];
  int brightness = 50;
  unsigned long rgb_delay_time = 0;
  unsigned long dispaly_timeout = 0;
  char flag = 0;

  uint32_t Wheel(byte WheelPos)
  {
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85)
    {
      return Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if (WheelPos < 170)
    {
      WheelPos -= 85;
      return Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  uint8_t red(uint32_t c)
  {
    return (c >> 8);
  }
  uint8_t green(uint32_t c)
  {
    return (c >> 16);
  }
  uint8_t blue(uint32_t c)
  {
    return (c);
  }
  bool pulseWhite(uint8_t wait)
  {
    for (int j = 0; j < 256; j++)
    {
      for (uint16_t i = 0; i < numPixels(); i++)
      {
        setPixelColor(i, Color(0, 0, 0, gamma[j]));
      }
   
      show();
    }
    for (int j = 255; j >= 0; j--)
    {
      for (uint16_t i = 0; i < numPixels(); i++)
      {
        setPixelColor(i, Color(0, 0, 0, gamma[j]));
      }
      
      show();
    }
    return false;
  }

  bool theaterChase(uint8_t r, uint8_t g, uint8_t b, uint8_t wait)
  {
    for (int j = 0; j < 200; j++)
    {
      for (int q = 0; q < 3; q++)
      {
        for (uint16_t i = 0; i < numPixels(); i = i + 3)
        {
          setPixelColor(i + q, Color(r, g, b));
        }
        show();
        delay(wait);
      
        for (uint16_t i = 0; i < numPixels(); i = i + 3)
        {
          setPixelColor(i + q, 0);
        }
      }
    }
    return false;
  }

  bool rainbow(uint8_t wait)
  {
    for (uint16_t k = 0; k <= 100; k++)
    {
      for (uint16_t j = 0; j < 256; j++)
      {
        for (uint16_t i = 0; i < numPixels(); i++)
        {
          setPixelColor(i, Wheel((i + j) & 255));
        }
        show();
        delay(wait);
        
      }
    }
    return false;
  }

  bool rainbowCycle(uint8_t wait)
  {
    for (uint16_t j = 0; j < 256 * 100; j++)
    {
      for (uint16_t i = 0; i < numPixels(); i++)
      {
        setPixelColor(i, Wheel(((i * 256 / numPixels()) + j) & 255));
      }
      show();
      delay(wait);
   
    }
    return false;
  }

  bool theaterChaseRainbow(uint8_t wait)
  {
    for (int k = 0; k < 30; k++)
    {
      for (int j = 0; j < 256; j++)
      {
        for (int q = 0; q < 3; q++)
        {
          for (uint16_t i = 0; i < numPixels(); i = i + 3)
          {
            setPixelColor(i + q, Wheel((i + j) % 255));
          }
          show();
          delay(wait);
         
          for (uint16_t i = 0; i < numPixels(); i = i + 3)
          {
            setPixelColor(i + q, 0);
          }
        }
      }
    }
    return false;
  }

  bool rainbowFade2White(uint8_t wait, int rainbowLoops, int whiteLoops)
  {
    float fadeMax = 100.0;
    int fadeVal = 0;
    uint32_t wheelVal;
    int redVal, greenVal, blueVal;
    for (int k = 0; k < rainbowLoops; k++)
    {
      for (int j = 0; j < 256; j++)
      {
        for (int i = 0; i < numPixels(); i++)
        {
          wheelVal = Wheel(((i * 256 / numPixels()) + j) & 255);
          redVal = red(wheelVal) * float(fadeVal / fadeMax);
          greenVal = green(wheelVal) * float(fadeVal / fadeMax);
          blueVal = blue(wheelVal) * float(fadeVal / fadeMax);
          setPixelColor(i, Color(redVal, greenVal, blueVal));
        }
        if (k == 0 && fadeVal < fadeMax - 1)
        {
          fadeVal++;
        }
        else if (k == rainbowLoops - 1 && j > 255 - fadeMax)
        {
          fadeVal--;
        }
        show();
        delay(wait);
       
      }
    }

  

    for (int k = 0; k < whiteLoops; k++)
    {

      for (int j = 0; j < 256; j++)
      {

        for (uint16_t i = 0; i < numPixels(); i++)
        {
          setPixelColor(i, Color(0, 0, 0, gamma[j]));
        }
        show();
      }
    
      for (int j = 255; j >= 0; j--)
      {

        for (uint16_t i = 0; i < numPixels(); i++)
        {
          setPixelColor(i, Color(0, 0, 0, gamma[j]));
        }
        show();
      }
    }

   

    return false;
  }

  bool whiteOverRainbow(uint8_t wait, uint8_t whiteSpeed, uint8_t whiteLength)
  {
    if (whiteLength >= numPixels())
      whiteLength = numPixels() - 1;

    unsigned int head = whiteLength - 1;
    unsigned int tail = 0;

    int loops = 100;
    int loopNum = 0;

    static unsigned long lastTime = 0;

    while (true)
    {
      for (int j = 0; j < 256; j++)
      {
        for (uint16_t i = 0; i < numPixels(); i++)
        {
          if ((i >= tail && i <= head) || (tail > head && i >= tail) || (tail > head && i <= head))
          {
            setPixelColor(i, Color(0, 0, 0, 255));
          }
          else
          {
            setPixelColor(i, Wheel(((i * 256 / numPixels()) + j) & 255));
          }
        }

        if (millis() - lastTime > whiteSpeed)
        {
          head++;
          tail++;
          if (head == numPixels())
          {
            loopNum++;
          }
          lastTime = millis();
        }

        if (loopNum == loops)
          return false;

        head %= numPixels();
        tail %= numPixels();
        show();
        delay(wait);
      }
    }
    return false;
  }

  void initialize()
  {
    begin();
    setBrightness(brightness);
    show();
  }
  //闪烁颜色设置
  void setColorNew(unsigned char r0, unsigned char g0, unsigned char b0,
                   unsigned char r1, unsigned char g1, unsigned char b1,
                   unsigned char r2, unsigned char g2, unsigned char b2,
                   unsigned char r3, unsigned char g3, unsigned char b3)
  {
    led_rgb_new[0] = Color(r0, g0, b0);
    led_rgb_new[1] = Color(r1, g1, b1);
    led_rgb_new[2] = Color(r2, g2, b2);
    led_rgb_new[3] = Color(r3, g3, b3);
  }
  //闪烁颜色设置
  void setColorOld(unsigned char r0, unsigned char g0, unsigned char b0,
                   unsigned char r1, unsigned char g1, unsigned char b1,
                   unsigned char r2, unsigned char g2, unsigned char b2,
                   unsigned char r3, unsigned char g3, unsigned char b3)
  {
    led_rgb_old[0] = Color(r0, g0, b0);
    led_rgb_old[1] = Color(r1, g1, b1);
    led_rgb_old[2] = Color(r2, g2, b2);
    led_rgb_old[3] = Color(r3, g3, b3);
  }
  void blink(unsigned long delay_time) //灯光闪烁
  {
    if ((millis() - previous_millis < delay_time) && (delay_flag == 0))
    {
      delay_flag = 1;
      for (size_t i = 0; i < numPixels(); i++)
      {
        setPixelColor(i, led_rgb_new[i]);
      }
      show();
    }
    else if ((millis() - previous_millis < delay_time * 2) && (millis() - previous_millis > delay_time) && (delay_flag == 1))
    {
      delay_flag = 2;
      for (size_t i = 0; i < numPixels(); i++)
      {
        setPixelColor(i, led_rgb_old[i]);
      }
      show();
    }
    else if (millis() - previous_millis >= delay_time * 2)
    {
      delay_flag = 0;
      previous_millis = millis();
    }
  }
  void lightOff() //关闭灯光
  {
    setColorNew(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void brightRedColor() //亮红灯
  {
    setColorNew(255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0);
    setColorOld(255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0);
  }
  void flashRedColor() //闪红灯
  {
    setColorNew(255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void brightBlueColor() //亮蓝灯
  {
    setColorNew(0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255);
    setColorOld(0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255);
  }
  void flashBlueColorFront() //前面闪蓝灯
  {
    setColorNew(0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 255);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashBlueColorback() //后面闪蓝灯
  {
    setColorNew(0, 0, 255, 0, 0, 255, 0, 0, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashBlueColorLeft() //左侧闪蓝灯
  {
    setColorNew(0, 0, 0, 0, 0, 255, 0, 0, 255, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashBlueColorRight() //右侧闪蓝灯
  {
    setColorNew(0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 255);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void brightYellowColor() //亮黄灯
  {
    setColorNew(255, 255, 0, 255, 255, 0, 255, 255, 0, 255, 255, 0);
    setColorOld(255, 255, 0, 255, 255, 0, 255, 255, 0, 255, 255, 0);
  }
  void flashYellowColorFront() //前面闪黄灯
  {
    setColorNew(0, 0, 0, 0, 0, 0, 255, 255, 0, 255, 255, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashYellowColorback() //后面闪黄灯
  {
    setColorNew(255, 255, 0, 255, 255, 0, 0, 0, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashYellowColorLeft() //左侧闪黄灯
  {
    setColorNew(0, 0, 0, 255, 255, 0, 255, 255, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashYellowColorRight() //右侧闪黄灯
  {
    setColorNew(255, 255, 0, 0, 0, 0, 0, 0, 0, 255, 255, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void brightGreenColor() //亮绿灯
  {
    setColorNew(0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0);
    setColorOld(0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0);
  }
  void flashGreenColorFront() //前面闪绿灯
  {
    setColorNew(0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 255, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashGreenColorback() //后面闪绿灯
  {
    setColorNew(0, 255, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashGreenColorLeft() //左侧闪绿灯
  {
    setColorNew(0, 0, 0, 0, 255, 0, 0, 255, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashGreenColorRight() //右侧闪绿灯
  {
    setColorNew(0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

private:
  unsigned char delay_flag = 0;
  unsigned long previous_millis = 0;
} rgb;
