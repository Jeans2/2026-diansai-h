#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdio.h>

#include "board.h"
#include "bsp_encoder.h"
#include "bsp_line_follow.h"
#include "bsp_speed_control.h"
#include "oled.h"

#define LINE_CONTROL_MS  10U
#define DISPLAY_MS       100U
#define OLED_PAGE_MS     10U
#define DEBUG_MS         500U
#define BUTTON_DEBOUNCE_MS 20U
#define BASE_SPEED_TARGET 800

/*
 * Stop distance in average quadrature encoder counts:
 * distance = (abs(motor A counts) + abs(motor B counts)) / 2.
 * Change this value to modify the required travel distance.
 */
#define ENCODER_STOP_DISTANCE_COUNTS 43400U  //43280

/*
 * Run-mode definitions.
 * MODE_FAST: instant start/stop, base=150, ~20 s per lap (current behaviour).
 * MODE_SLOW: soft start ramp + soft stop ramp, base=90, <30 s per lap.
 */
typedef enum {
    MODE_FAST = 0,
    MODE_SLOW = 1,
    MODE_COUNT
} RunMode;

/* ---- Mod2 slow-run tuning parameters ---- */
#define MODE2_BASE_SPEED              80    /* cruising encoder cnt / 50 ms   */
#define MODE2_RAMP_UP_MS            3000U   /* soft-start duration (ms)       */
#define MODE2_RAMP_UP_MIN_TARGET       5    /* initial target during ramp-up  */
#define MODE2_RAMP_DOWN_DIST        4000U   /* remaining cnt when ramp starts */
#define MODE2_RAMP_DOWN_MIN_TARGET     5    /* final target before hard stop  */

#define MODE_BUTTON_PIN      BUTTONS_SPEED_DOWN_PIN  /* PA12 cycles Mod1 <-> Mod2 */

static volatile uint32_t gMilliseconds;
static volatile bool gRunning;
static volatile bool gControlReady;
static volatile uint8_t gSpeedTickMs;
static bool gDistanceStopLatched;
static int32_t gDistanceStartA;
static int32_t gDistanceStartB;
static uint32_t gTravelCounts;
static uint32_t gRunStartMs;
static uint32_t gRunElapsedMs;
static RunMode gRunMode = MODE_FAST;
static bool gSoftStartActive;
static uint32_t gSoftStartBeginMs;

typedef struct {
    uint32_t pin;
    uint32_t changedAtMs;
    bool rawPressed;
    bool stablePressed;
} ButtonState;

void SysTick_Handler(void)
{
    gMilliseconds++;

    if (gControlReady && gRunning) {
        gSpeedTickMs++;
        if (gSpeedTickMs >= SPEED_CONTROL_MS) {
            gSpeedTickMs = 0;
            SpeedControl_Update(SPEED_CONTROL_MS);
        }
    } else {
        gSpeedTickMs = 0;
    }
}

static uint32_t Millis(void)
{
    return gMilliseconds;
}

static bool Button_IsPressed(uint32_t pin)
{
    return (DL_GPIO_readPins(BUTTONS_PORT, pin) & pin) == 0U;
}

static void Button_Init(
    ButtonState *button, uint32_t pin, uint32_t now)
{
    button->pin = pin;
    button->changedAtMs = now;
    button->rawPressed = Button_IsPressed(pin);
    button->stablePressed = button->rawPressed;
}

static bool Button_PressedEvent(ButtonState *button, uint32_t now)
{
    bool pressed = Button_IsPressed(button->pin);

    if (pressed != button->rawPressed) {
        button->rawPressed = pressed;
        button->changedAtMs = now;
    }

    if ((pressed != button->stablePressed) &&
        ((uint32_t) (now - button->changedAtMs) >= BUTTON_DEBOUNCE_MS)) {
        button->stablePressed = pressed;
        return pressed;
    }

    return false;
}

static uint32_t Encoder_AbsoluteDifference(int32_t current, int32_t start)
{
    int64_t difference = (int64_t) current - (int64_t) start;

    if (difference < 0) {
        difference = -difference;
    }
    if (difference > UINT32_MAX) {
        difference = UINT32_MAX;
    }
    return (uint32_t) difference;
}

static void Distance_Start(void)
{
    gDistanceStartA = Encoder_GetMotorA();
    gDistanceStartB = Encoder_GetMotorB();
    gTravelCounts = 0U;
}

static void Distance_Update(void)
{
    uint32_t distanceA = Encoder_AbsoluteDifference(
        Encoder_GetMotorA(), gDistanceStartA);
    uint32_t distanceB = Encoder_AbsoluteDifference(
        Encoder_GetMotorB(), gDistanceStartB);

    gTravelCounts = (uint32_t)
        (((uint64_t) distanceA + (uint64_t) distanceB) / 2U);
}

static void RunTimer_Start(uint32_t now)
{
    gRunStartMs = now;
    gRunElapsedMs = 0U;
}

static void RunTimer_Update(uint32_t now)
{
    gRunElapsedMs = (uint32_t) (now - gRunStartMs);
}

static void DisplayState(const LineFollowState *line, bool running)
{
    const SpeedControlState *speed = SpeedControl_GetState();
    char text[24];
    uint8_t channel;
    char runState;
    uint32_t minutes;
    uint32_t seconds;
    uint32_t milliseconds;

    runState = gDistanceStopLatched ? 'L' : (running ? 'R' : 'S');
    (void) snprintf(text, sizeof(text), "%c%03d D%05lu",
        runState, LineFollow_GetBaseSpeed(),
        (unsigned long) gTravelCounts);
    OLED_ShowString(0, 0, (u8 *) text, 16, 1);

    text[0] = 'S';
    text[1] = ':';
    for (channel = 0; channel < GRAYSCALE_CHANNELS; channel++) {
        text[2U + channel] =
            (line->mask & (uint8_t) (1U << channel)) ? '1' : '0';
    }
    (void) snprintf(&text[10], sizeof(text) - 10U, " E%+4d",
        line->error);
    OLED_ShowString(0, 16, (u8 *) text, 16, 1);

    (void) snprintf(text, sizeof(text), "TA:%+4d TB:%+4d ",
        speed->targetA, speed->targetB);
    OLED_ShowString(0, 32, (u8 *) text, 16, 1);

    minutes = gRunElapsedMs / 60000U;
    seconds = (gRunElapsedMs / 1000U) % 60U;
    milliseconds = gRunElapsedMs % 1000U;
    (void) snprintf(text, sizeof(text), "M%u %02lu:%02lu.%03lu",
        (unsigned int) (gRunMode + 1U),
        (unsigned long) minutes,
        (unsigned long) seconds,
        (unsigned long) milliseconds);
    OLED_ShowString(0, 48, (u8 *) text, 16, 1);
}

int main(void)
{
    LineFollowState line = {0};
    const SpeedControlState *speed;
    uint32_t now;
    uint32_t lastLineMs;
    uint32_t lastDisplayMs;
    uint32_t lastOledPageMs;
    uint32_t lastDebugMs;
    uint8_t oledPage = 0;
    ButtonState startStopButton;
    ButtonState modeButton;

    SYSCFG_DL_init();
    (void) SysTick_Config(CPUCLK_FREQ / 1000U);

    Encoder_Init();
    LineFollow_Init();
    LineFollow_SetBaseSpeed(BASE_SPEED_TARGET);
    SpeedControl_Init();

    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    LineFollow_ReadAndProcess(&line);
    LineFollow_ClearTargets(&line);
    DisplayState(&line, false);
    gRunning = false;
    gSpeedTickMs = 0;
    gDistanceStopLatched = false;
    gTravelCounts = 0U;
    gRunStartMs = 0U;
    gRunElapsedMs = 0U;
    gControlReady = true;

    now = Millis();
    Button_Init(&startStopButton, BUTTONS_START_STOP_PIN, now);
    Button_Init(&modeButton, MODE_BUTTON_PIN, now);
    lastLineMs = now;
    lastDisplayMs = now;
    lastOledPageMs = now - OLED_PAGE_MS;
    lastDebugMs = now;

    lc_printf("\r\nEncoder closed-loop line follower ready.\r\n");
    lc_printf("Mode M1 (fast, base=%d) / M2 (slow, base=%d).\r\n",
        BASE_SPEED_TARGET, MODE2_BASE_SPEED);
    lc_printf("PA25 run/stop. PA12 cycle mode. Direction PD = 10 ms, speed PI = 5 ms.\r\n");
    lc_printf("Distance stop = %lu average encoder counts.\r\n",
        (unsigned long) ENCODER_STOP_DISTANCE_COUNTS);

    while (1)
    {
        now = Millis();

        if (Button_PressedEvent(&startStopButton, now) &&
            !gDistanceStopLatched) {
            if (gRunning) {
                RunTimer_Update(now);
                gRunning = false;
                gSoftStartActive = false;
                if (gRunMode == MODE_FAST) {
                    LineFollow_SetBaseSpeed(BASE_SPEED_TARGET);
                } else {
                    LineFollow_SetBaseSpeed(MODE2_BASE_SPEED);
                }
                SpeedControl_Stop();
                LineFollow_ClearTargets(&line);
                SpeedControl_SetTargets(0, 0);
            } else {
                LineFollow_Init();
                SpeedControl_Init();
                Distance_Start();
                RunTimer_Start(now);
                LineFollow_ReadAndProcess(&line);
                if (gRunMode == MODE_SLOW) {
                    gSoftStartActive = true;
                    gSoftStartBeginMs = now;
                    LineFollow_SetBaseSpeed(MODE2_RAMP_UP_MIN_TARGET);
                }
                LineFollow_ComputeTargets(&line);
                SpeedControl_SetTargets(line.targetA, line.targetB);
                gSpeedTickMs = 0;
                gRunning = true;
            }
            lastDisplayMs = Millis() - DISPLAY_MS;
        }

        now = Millis();

        /* ---- Mode button (PA12, only when stopped) ---- */
        if (Button_PressedEvent(&modeButton, now) &&
            !gRunning && !gDistanceStopLatched) {
            gRunMode = (RunMode) (((unsigned int) gRunMode + 1U) %
                (unsigned int) MODE_COUNT);
            if (gRunMode == MODE_FAST) {
                LineFollow_SetBaseSpeed(BASE_SPEED_TARGET);
            } else {
                LineFollow_SetBaseSpeed(MODE2_BASE_SPEED);
            }
            lastDisplayMs = now - DISPLAY_MS;
        }

        if (gRunning) {
            RunTimer_Update(now);
            Distance_Update();
            if (gTravelCounts >= ENCODER_STOP_DISTANCE_COUNTS) {
                gDistanceStopLatched = true;
                gRunning = false;
                gSoftStartActive = false;
                if (gRunMode == MODE_FAST) {
                    LineFollow_SetBaseSpeed(BASE_SPEED_TARGET);
                } else {
                    LineFollow_SetBaseSpeed(MODE2_BASE_SPEED);
                }
                LineFollow_ClearTargets(&line);
                SpeedControl_SetTargets(0, 0);
                SpeedControl_Stop();
                DL_GPIO_clearPins(LED_PORT, LED_PIN_22_PIN);
                lastDisplayMs = now - DISPLAY_MS;
                lc_printf(
                    "DISTANCE STOP: %lu / %lu counts. Output locked at 0.\r\n",
                    (unsigned long) gTravelCounts,
                    (unsigned long) ENCODER_STOP_DISTANCE_COUNTS);
            }
        }

        if ((uint32_t) (now - lastLineMs) >= LINE_CONTROL_MS) {
            lastLineMs = now;
            LineFollow_ReadAndProcess(&line);

            if (gRunning) {
                /*
                 * Mod2 soft-start ramp: linearly increase base speed
                 * from RAMP_UP_MIN_TARGET to MODE2_BASE_SPEED over
                 * MODE2_RAMP_UP_MS milliseconds.
                 */
                if (gRunMode == MODE_SLOW && gSoftStartActive) {
                    uint32_t rampElapsed = now - gSoftStartBeginMs;

                    if (rampElapsed >= MODE2_RAMP_UP_MS) {
                        gSoftStartActive = false;
                        LineFollow_SetBaseSpeed(MODE2_BASE_SPEED);
                    } else {
                        int16_t rampTarget =
                            MODE2_RAMP_UP_MIN_TARGET +
                            (int16_t)((int32_t)(MODE2_BASE_SPEED -
                                MODE2_RAMP_UP_MIN_TARGET) *
                                (int32_t) rampElapsed /
                                (int32_t) MODE2_RAMP_UP_MS);
                        LineFollow_SetBaseSpeed(rampTarget);
                    }
                }

                /*
                 * Mod2 soft-stop ramp: when remaining distance falls
                 * below MODE2_RAMP_DOWN_DIST, linearly reduce base
                 * speed to MODE2_RAMP_DOWN_MIN_TARGET at count zero.
                 */
                if (gRunMode == MODE_SLOW && !gSoftStartActive) {
                    uint32_t remaining;

                    if (gTravelCounts < ENCODER_STOP_DISTANCE_COUNTS) {
                        remaining =
                            ENCODER_STOP_DISTANCE_COUNTS - gTravelCounts;
                    } else {
                        remaining = 0U;
                    }

                    if (remaining < MODE2_RAMP_DOWN_DIST) {
                        int16_t stopTarget =
                            MODE2_RAMP_DOWN_MIN_TARGET +
                            (int16_t)((int32_t)(MODE2_BASE_SPEED -
                                MODE2_RAMP_DOWN_MIN_TARGET) *
                                (int32_t) remaining /
                                (int32_t) MODE2_RAMP_DOWN_DIST);
                        LineFollow_SetBaseSpeed(stopTarget);
                    }
                }

                LineFollow_ComputeTargets(&line);
                SpeedControl_SetTargets(line.targetA, line.targetB);
            } else {
                LineFollow_ClearTargets(&line);
                SpeedControl_SetTargets(0, 0);
            }
        }

        if ((uint32_t) (now - lastDisplayMs) >= DISPLAY_MS) {
            lastDisplayMs = now;
            if (gRunning) {
                DL_GPIO_togglePins(LED_PORT, LED_PIN_22_PIN);
            }
            DisplayState(&line, gRunning);
        }

        if ((uint32_t) (now - lastOledPageMs) >= OLED_PAGE_MS) {
            lastOledPageMs = now;
            OLED_RefreshPage(oledPage);
            oledPage = (uint8_t) ((oledPage + 1U) & 0x07U);
        }

        if ((uint32_t) (now - lastDebugMs) >= DEBUG_MS) {
            lastDebugMs = now;
            speed = SpeedControl_GetState();
            lc_printf(
                "MOD=%u RUN=%u T_A=%d M_A=%d PWM_A=%d "
                "T_B=%d M_B=%d PWM_B=%d E=%d C=%u MASK=0x%02X\r\n",
                (unsigned int) gRunMode,
                gRunning ? 1U : 0U,
                speed->targetA, speed->measuredA, speed->pwmA,
                speed->targetB, speed->measuredB, speed->pwmB,
                line.error, (unsigned int) line.contrast,
                (unsigned int) line.mask);
            lc_printf("MOD=%u DIST=%lu/%lu LOCK=%u\r\n",
                (unsigned int) gRunMode,
                (unsigned long) gTravelCounts,
                (unsigned long) ENCODER_STOP_DISTANCE_COUNTS,
                gDistanceStopLatched ? 1U : 0U);
            lc_printf(
                "GRAY=%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                (unsigned int) line.raw[0],
                (unsigned int) line.raw[1],
                (unsigned int) line.raw[2],
                (unsigned int) line.raw[3],
                (unsigned int) line.raw[4],
                (unsigned int) line.raw[5],
                (unsigned int) line.raw[6],
                (unsigned int) line.raw[7]);
        }

        __WFI();
    }
}
