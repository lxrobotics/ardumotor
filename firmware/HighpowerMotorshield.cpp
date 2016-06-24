/* Copyright (c) 2016, Alexander Entinger / LXRobotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of motor-controller-highpower-motorshield nor the names of its
 *  contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "HighpowerMotorshield.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>

/**************************************************************************************
 * DEFINES
 **************************************************************************************/

/* IN1 = D5 = PD5 = OC0B */
#define IN1_DDR         (DDRD)
#define IN1_PORT        (PORTD)
#define IN1             (1<<5)
/* IN2 = D6 = PD6 = OC0A */
#define IN2_DDR	        (DDRD)
#define IN2_PORT        (PORTD)
#define IN2             (1<<6)
/* INH = D7 = PD7 */
#define INH_DDR         (DDRD)
#define INH_PORT        (PORTD)
#define INH             (1<<7)

/**************************************************************************************
 * TYPEDEFS
 **************************************************************************************/

typedef struct 
{
  uint8_t pwm;
  EDirection dir;
} SMotorParams;

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static volatile SMotorParams m_motor_params = {0, FWD};

/**************************************************************************************
 * PUBLIC FUNCTIONS
 **************************************************************************************/

void HighpowerMotorshield::begin() 
{
  /* set INH to output and to low (halfbridges deactivated) */
  INH_PORT &= ~INH;
  INH_DDR |= INH;

  /* set IN1/2 pins to outputs with value low */
  IN1_PORT &= ~IN1;
  IN1_DDR |= IN1;
  IN2_PORT &= ~IN2;
  IN2_DDR |= IN2;

  /* Clear TCCR2A from whatever might be still left there */
  TCCR2A = 0x00;	
  /* Reset the timer value */
  TCNT2 = 0;
  /* Enable compare and overflow interrupts */
  TIMSK2 = (1<<OCIE2A) | (1<<TOIE2);
  /* activate timer with prescaler 32 => f_PWM = 1,96 kHz */
  TCCR2B = (1<<CS21) | (1<<CS20);

  /* Set pwm and direction */
  HighpowerMotorshield::setPwm      (m_motor_params.pwm);
  HighpowerMotorshield::setDirection(m_motor_params.dir);

  /* Activate h brigde */
  INH_PORT |= INH;
}

void HighpowerMotorshield::setPwm(uint8_t const pwm) 
{
  m_motor_params.pwm = pwm;

  OCR2A = m_motor_params.pwm;
}

void HighpowerMotorshield::setDirection(EDirection const dir) 
{
  m_motor_params.dir = dir;
}

/**************************************************************************************
 * INTERRUPTS
 **************************************************************************************/

ISR(TIMER2_OVF_vect) 
{
  if(m_motor_params.pwm > 0) 
  {
    if     (m_motor_params.dir == FWD) IN1_PORT |= IN1;
    else if(m_motor_params.dir == BWD) IN2_PORT |= IN2;
  }
}

ISR(TIMER2_COMPA_vect)
{
  IN1_PORT &= ~IN1;	
  IN2_PORT &= ~IN2;
}
