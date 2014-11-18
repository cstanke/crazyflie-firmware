/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * neopixelring.c: NeoPixel Ring 16 Leds effects/driver
 */

#include "neopixelring.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "timers.h"

#include "ws2812.h"
#include "worker.h"
#include "param.h"
#include "pm.h"
#include "log.h"


/*
 * To add a new effect just add it as a static function with the prototype
 * void effect(uint8_t buffer[][3], bool reset)
 *
 * Then add it to the effectsFct[] list bellow. It will automatically be
 * activable using the ring.effect parameter.
 *
 * The ring color needs to be written in the buffer argument. The buffer is not
 * modified in memory as long as reset is not 'true', see the spin effects for
 * and example.
 *
 * The log subsystem can be used to get the value of any log variable of the
 * system. See tiltEffect for an example.
 */


/**************** Some useful macros ***************/

#define RED {0x10, 0x00, 0x00}
#define YELLOW {0xff, 0xff, 0x00}
#define GREEN {0x00, 0x10, 0x00}
#define BLUE {0x00, 0x00, 0x10}
#define WHITE {0xff, 0xff, 0xff}
#define BLACK {0x00, 0x00, 0x00}

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)
#define COPY_COLOR(dest, orig) dest[0]=orig[0]; dest[1]=orig[1]; dest[2]=orig[2]
#define LIMIT(a) ((a>255)?255:(a<0)?0:a)
#define SIGN(a) ((a>=0)?1:-1)

#define MAX_PIXELS 12
#define MAIN_TIMER_MS 40 // Bitcraze default
//#define MAIN_TIMER_MS 80
//#define MAIN_TIMER_MS 1000 // VERY slow

static xTimerHandle timer;

/**************** Black (LEDs OFF) ***************/

static void blackEffect(uint8_t buffer[][3], bool reset)
{
    int i;
  
    // Set the timer speed...
    xTimerChangePeriod(timer, M2T(MAIN_TIMER_MS), 100);
  
    if (reset)
    {
        for (i=0; i<MAX_PIXELS; i++) {
            buffer[i][0] = 0;
            buffer[i][1] = 0;
            buffer[i][2] = 0;
        }
    }
}

/**************** ledTest ***************/

static void ledTest(uint8_t buffer[][3], bool reset)
{
    int i;

    //uint8_t dark_green[3] = {0, 20, 0};
    //uint8_t black[3] = {0, 0, 0};
    uint8_t green[3] = GREEN;
    uint8_t blue[3] = BLUE;
    uint8_t red[3] = RED;
    //uint8_t orange[3] = {255, 90, 0};
    
    // Set the timer speed...
    //xTimerChangePeriod(timer, M2T(MAIN_TIMER_MS), 1000);

    if (reset)
    {
        for (i=0; i<MAX_PIXELS; i++) {
            buffer[i][0] = 0;
            buffer[i][1] = 0;
            buffer[i][2] = 0;
        }
    }
    
    COPY_COLOR(buffer[0], red);
    COPY_COLOR(buffer[1], green);
    COPY_COLOR(buffer[2], blue);
    
   
    
        
    /*    
    COPY_COLOR(buffer[11], green);
    COPY_COLOR(buffer[0], dark_green);
    COPY_COLOR(buffer[1], green);
    
    COPY_COLOR(buffer[3], blue);
    COPY_COLOR(buffer[9], blue);
    
    COPY_COLOR(buffer[4], red);
    COPY_COLOR(buffer[5], red);
    COPY_COLOR(buffer[6], orange);
    COPY_COLOR(buffer[7], red);
    COPY_COLOR(buffer[8], red);
    */
    
    /*
    COPY_COLOR(buffer[3], test_led[1]);
    COPY_COLOR(buffer[6], test_led[0]);
    COPY_COLOR(buffer[9], test_led[1]);
    */
}

/**************** Green Spin ***************/

static const uint8_t greenSpin[][3] = {
                                        {0,20.0}, BLACK, BLACK, {0,20.0}, BLACK, BLACK,
                                        {0,20.0}, BLACK, BLACK, {0,20.0}, BLACK, BLACK,
                                      };

static void greenSpinEffect(uint8_t buffer[][3], bool reset)
{
    int i;
    uint8_t temp[3];
    
    // Set the timer speed a litte slower...
    xTimerChangePeriod(timer, M2T(100), 100);

    if (reset)
    {
        for (i=0; i<MAX_PIXELS; i++)
        {
            COPY_COLOR(buffer[i], greenSpin[i]);
        }
    }
    
    COPY_COLOR(temp, buffer[MAX_PIXELS-1]);
    for (i=MAX_PIXELS-1;i>0;i--)
    {
        COPY_COLOR(buffer[i], buffer[i-1]);
    }
    COPY_COLOR(buffer[0], temp);
}



/**************** Thrust ***************/

static void thrustEffect(uint8_t buffer[][3], bool reset)
{
    static int thrustid;
    int i;
    int thrust_value = 0;
    
    // Set the timer speed...
    xTimerChangePeriod(timer, M2T(MAIN_TIMER_MS), 100);

    if (reset) {
        //Init
        thrustid = logGetVarId("stabilizer", "thrust");
    } else {
        float thrust = logGetFloat(thrustid);
        thrust_value = LIMIT(thrust/236);
        
        for (i=0; i<MAX_PIXELS; i++) {
            buffer[i][0] = thrust_value;
            buffer[i][1] = 0;
            buffer[i][2] = 0;
        }
    }
}


/**************** White spin ***************/

static const uint8_t cw_whiteRing[][3] = {BLACK, BLACK, BLACK, BLACK, BLACK,
                                          {1,1,1}, {2,2,2}, {4,4,4}, {8,8,8}, 
                                          {16,16,16}, {32, 32, 32}, {40, 40, 40}
                                         };

static const uint8_t ccw_whiteRing[][3] = {{40, 40, 40}, {32, 32, 32}, {16,16,16}, 
                                           {8,8,8}, {4,4,4}, {2,2,2}, {1,1,1}, 
                                           BLACK, BLACK, BLACK, BLACK, BLACK
                                         };


static void whiteSpinEffect(uint8_t buffer[][3], bool reset)
{
  int i;
  uint8_t temp[3];
  
    // Set the timer speed...
    xTimerChangePeriod(timer, M2T(MAIN_TIMER_MS), 100);
  
  if (reset)
  {
    for (i=0; i<MAX_PIXELS; i++) {
      COPY_COLOR(buffer[i], cw_whiteRing[i]);
    }
  }

    /* Runs Counter Clockwise */
    /*
    COPY_COLOR(temp, buffer[0]);
    for (i=0; i<11; i++) {
        COPY_COLOR(buffer[i], buffer[i+1]);
    }
    COPY_COLOR(buffer[11], temp);
    */
    
    /* Runs Clockwise */
    COPY_COLOR(temp, buffer[11]);
    for (i=11; i>0; i--) {
        COPY_COLOR(buffer[i], buffer[i-1]);
    }
    COPY_COLOR(buffer[0], temp);  

}

/**************** Color spin ***************/

static const uint8_t colorRing[][3] = {
                                        BLACK, BLACK, {3,0,0}, {13,0,0}, {50,0,0}, {200,0,0},
                                        BLACK, BLACK, {0,0,3}, {0,0,13}, {0,0,50}, {0,0,200},
                                      };

static void colorSpinEffect(uint8_t buffer[][3], bool reset)
{
    int i;
    uint8_t temp[3];
  
    // Set the timer speed...
    xTimerChangePeriod(timer, M2T(80), 100);
  
    if (reset)
    {
        for (i=0; i<MAX_PIXELS; i++)
        {
            COPY_COLOR(buffer[i], colorRing[i]);
        }
    }

    /* Runs Counter Clockwise */
    /*
    COPY_COLOR(temp, buffer[0]);
    for (i=0; i<11; i++) {
        COPY_COLOR(buffer[i], buffer[i+1]);
    }
    COPY_COLOR(buffer[11], temp);
    */
    
    /* Runs Clockwise */
    COPY_COLOR(temp, buffer[11]);
    for (i=11; i>0; i--)
    {
        COPY_COLOR(buffer[i], buffer[i-1]);
    }
    COPY_COLOR(buffer[0], temp);  
}

/**************** Dynamic tilt effect ***************/

static void tiltEffect(uint8_t buffer[][3], bool reset)
{
  static int pitchid, rollid, thrust=-1;
  
    // Set the timer speed...
    xTimerChangePeriod(timer, M2T(MAIN_TIMER_MS), 100);
  
  if (reset)
    {
        int i;
        for (i=0; i<MAX_PIXELS; i++) {
            buffer[i][0] = 0;
            buffer[i][1] = 0;
            buffer[i][2] = 0;
        }
    }
  
    if (thrust<0)
    {
        //Init
        pitchid = logGetVarId("stabilizer", "pitch");
        rollid = logGetVarId("stabilizer", "roll");
        thrust = logGetVarId("stabilizer", "thrust");
    } 
    else
    {
        const int led_middle = 10;
        float pitch = -1*logGetFloat(pitchid);
        float roll  = -1*logGetFloat(rollid);
    
        pitch = (pitch>20)?20:(pitch<-20)?-20:pitch;
        roll = (roll>20)?20:(roll<-20)?-20:roll;
      
        pitch=SIGN(pitch)*pitch*pitch;
        roll*=SIGN(roll)*roll;
    
        // 12 pixels
        buffer[11][0] = LIMIT(led_middle + pitch);
        buffer[0][0] = LIMIT(led_middle + pitch);
        buffer[1][0] = LIMIT(led_middle + pitch);
    
        buffer[2][2] = LIMIT(led_middle + roll);
        buffer[3][2] = LIMIT(led_middle + roll);
        buffer[4][2] = LIMIT(led_middle + roll);
    
        buffer[5][0] = LIMIT(led_middle - pitch);
        buffer[6][0] = LIMIT(led_middle - pitch);
        buffer[7][0] = LIMIT(led_middle - pitch);

        buffer[8][2] = LIMIT(led_middle - roll);
        buffer[9][2] = LIMIT(led_middle - roll);
        buffer[10][2] = LIMIT(led_middle - roll);
    }
}

/**************** Effect list ***************/

NeopixelRingEffect effectsFct[] = {blackEffect,
                                   ledTest,
                                   greenSpinEffect,
                                   whiteSpinEffect, 
                                   colorSpinEffect,
                                   thrustEffect,
                                   tiltEffect,
                                  }; //TODO Add more


/********** Ring init and switching **********/

// static xTimerHandle timer;

static uint32_t effect;
static uint32_t neffect;

static uint8_t black[][3] = {BLACK, BLACK, BLACK, BLACK,
                             BLACK, BLACK, BLACK, BLACK,
                             BLACK, BLACK, BLACK, BLACK,
                            };

void neopixelringWorker(void * data)
{
    static int current_effect = 0;
    static uint8_t buffer[MAX_PIXELS][3];
    bool reset = true;
  
    if (!pmIsDischarging() || (effect > neffect))
    {
        ws2812Send(black, MAX_PIXELS);
        return;
    }
  
    if (current_effect != effect)
    {
        reset = true;
    }
    else
    {
        reset = false;
    }
    current_effect = effect;
  
    effectsFct[current_effect](buffer, reset);
    ws2812Send(buffer, MAX_PIXELS);
}

static void neopixelringTimer(xTimerHandle timer)
{
    workerSchedule(neopixelringWorker, NULL);
}

void neopixelringInit(void)
{
    ws2812Init();
  
    neffect = sizeof(effectsFct)/sizeof(effectsFct[0])-1;  
  
    timer = xTimerCreate( (const signed char *)"ringTimer", M2T(MAIN_TIMER_MS), 
                                     pdTRUE, NULL, neopixelringTimer );
    xTimerStart(timer, 100);
}

PARAM_GROUP_START(ring)
PARAM_ADD(PARAM_UINT32, effect, &effect)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect)
PARAM_GROUP_STOP(ring)
