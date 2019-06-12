/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          app_occupancy_buttons.h
 *
 * DESCRIPTION:        DK4 (DR1175/DR1199) Button Press detection (Implementation)
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5179].
 * You, and any third parties must reproduce the copyright and warranty notice
 * and any other legend of ownership on each copy or partial copy of the
 * software.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright NXP B.V. 2017. All rights reserved
 *
 ***************************************************************************/

#ifndef APP_OCCUPANCY_BUTTONS_H_
#define APP_OCCUPANCY_BUTTONS_H_

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

#define DIO_RIGHT_BUTTON_RISING    17
#define DIO_RIGHT_BUTTON_FALLING   16

#define DIO_MIDDLE_BUTTON_RISING   15
#define DIO_MIDDLE_BUTTON_FALLING  14

#define DIO_LEFT_BUTTON_RISING     13
#define DIO_LEFT_BUTTON_FALLING    12

#define MASK_LEFT_BUTTON_RISING    (1 << DIO_LEFT_BUTTON_RISING)
#define MASK_LEFT_BUTTON_FALLING   (1 << DIO_LEFT_BUTTON_FALLING)
#define MASK_RIGHT_BUTTON_RISING   (1 << DIO_RIGHT_BUTTON_RISING)
#define MASK_RIGHT_BUTTON_FALLING  (1 << DIO_RIGHT_BUTTON_FALLING)

#define MASK_LEFT_BUTTON           (MASK_LEFT_BUTTON_RISING | MASK_LEFT_BUTTON_FALLING)
#define MASK_RIGHT_BUTTON          (MASK_RIGHT_BUTTON_RISING | MASK_RIGHT_BUTTON_FALLING)


#define APP_BUTTONS_NUM             (4UL)
#define APP_BUTTONS_BUTTON_SW1      (DIO_RIGHT_BUTTON_RISING)
#define APP_BUTTONS_BUTTON_SW2      (DIO_RIGHT_BUTTON_FALLING)
#define APP_BUTTONS_BUTTON_SW3      (DIO_LEFT_BUTTON_RISING)
#define APP_BUTTONS_BUTTON_SW4      (DIO_LEFT_BUTTON_FALLING)

#define APP_BUTTONS_DIO_MASK_RISING (MASK_LEFT_BUTTON_RISING | MASK_RIGHT_BUTTON_RISING)
#define APP_BUTTONS_DIO_MASK_FALLING (MASK_LEFT_BUTTON_FALLING | MASK_RIGHT_BUTTON_FALLING)

#define APP_BUTTONS_DIO_MASK        (APP_BUTTONS_DIO_MASK_RISING | APP_BUTTONS_DIO_MASK_FALLING)
#define APP_BUTTONS_DIO_MASK_FOR_DEEP_SLEEP        APP_BUTTONS_DIO_MASK




typedef enum {
    E_INTERRUPT_UNKNOWN,
    E_INTERRUPT_BUTTON,
    E_INTERRUPT_WAKE_TIMER_EXPIRY
} teInterruptType;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC void APP_bButtonInitialise(void);
PUBLIC void vActionOnButtonActivationAfterDeepSleep(void);
PUBLIC void vSaveDioStateBeforeDeepSleep(void);
PUBLIC bool_t bGetPreSleepOccupancyState(void);
PUBLIC void vISR_SystemController(void);
PUBLIC bool_t bButtonDebounceInProgress(void);

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#endif /*APP_OCCUPANCY_BUTTONS_H_*/
