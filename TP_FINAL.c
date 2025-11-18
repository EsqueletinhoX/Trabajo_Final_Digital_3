#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>

#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_dac.h"
#include <stdint.h>

/* ==== LCD en P1.x ==== */
#define LCD_RS_PORT   LPC_GPIO1
#define LCD_E_PORT    LPC_GPIO1
#define LCD_D4_PORT   LPC_GPIO1
#define LCD_D5_PORT   LPC_GPIO1
#define LCD_D6_PORT   LPC_GPIO1
#define LCD_D7_PORT   LPC_GPIO1

#define LCD_RS_PIN    (21)
#define LCD_E_PIN     (22)
#define LCD_D4_PIN    (23)
#define LCD_D5_PIN    (24)
#define LCD_D6_PIN    (25)
#define LCD_D7_PIN    (26)

/* ==== Macros de E/S ==== */
#define PIN_OUT(port,pin)   ((port)->FIODIR |=  (1u << (pin)))
#define PIN_SET(port,pin)   ((port)->FIOSET  = (1u << (pin)))
#define PIN_CLR(port,pin)   ((port)->FIOCLR  = (1u << (pin)))

/* =================== P I N E S =================== */
/* Motor IZQ: IN1=P2.0, IN2=P2.1
   Motor DER: IN1=P2.2, IN2=P2.3  */
#define MOT_PORT      (2)
#define L_IN1         (1u << 0)
#define L_IN2         (1u << 1)
#define R_IN1         (1u << 2)
#define R_IN2         (1u << 3)
#define MOT_MASK      (L_IN1 | L_IN2 | R_IN1 | R_IN2)

/* UART2: TXD2=P0.10, RXD2=P0.11 (Func 1) */
#define UARTx         LPC_UART2

/* Defines de puertos/pines para GPIO */
#define OUTPUT (uint8_t) 1
#define INPUT  (uint8_t) 0
#define PORT_0 (uint8_t) 0
#define PORT_1 (uint8_t) 1
#define PORT_2 (uint8_t) 2
#define PORT_3 (uint8_t) 3
#define PIN_22 (uint32_t) (1<<22)
#define PIN_23 (uint32_t) (1<<23)
#define PIN_18 (uint32_t) (1<<18)
#define PIN_19 (uint32_t) (1<<19)
#define PIN_25 (uint32_t) (1<<25)
#define PIN_26 (uint32_t) (1<<26)

/* =================== ESTADO COMANDOS =================== */
typedef enum {
    CMD_NONE = 0,
    CMD_A,   /* Avanzar   */
    CMD_S,   /* Stop      */
    CMD_D,   /* Derecha   */
    CMD_I,   /* Izquierda */
    CMD_R    /* Retroceder*/
} cmd_t;

/* =================== ESTADO SIRENA / AUDIO (DAC) =================== */
#define FS_HZ        20000u   /* Frec. muestreo DAC (20 kHz) */
#define LUT_SIZE     64u      /* Tabla seno */
#define F_MIN_HZ     400.0f
#define F_MAX_HZ     1200.0f
#define DF_PER_MS    2.0f     /* variación de frecuencia por ms */

#define DAC_OFFSET   512u     /* centro (10 bits) */
#define DAC_AMPL     450u     /* amplitud */

/* Tabla seno */
static const uint16_t sineLUT[LUT_SIZE] = {
    512,562,611,660,707,752,795,836,873,907,936,962,984,1000,1013,1020,
    1023,1020,1013,1000,984,962,936,907,873,836,795,752,707,660,611,562,
    512,462,413,364,317,272,229,188,151,117,88,62,40,24,11,4,
    1,4,11,24,40,62,88,117,151,188,229,272,317,364,413,462
};

/* ---- Estado de sirena/bocina ---- */
static volatile uint8_t  siren_enabled  = 0;
static volatile uint8_t  melody_active  = 0;

static volatile uint32_t phase_q16 = 0;
static volatile uint32_t step_q16  = 0;
static volatile float    f_curr    = F_MIN_HZ;
static volatile float    df_ms     = DF_PER_MS;

/* ---- Notas para la melodía ---- */
#define REST      0
#define NOTE_B4   494
#define NOTE_D5   587
#define NOTE_E5   659
#define NOTE_FS5  740
#define NOTE_GS5  831
#define NOTE_A5   880
#define NOTE_B5   988

#define TEMPO        280
#define WHOLENOTE_MS ((60000U * 4U) / (TEMPO))

/* Melodía "Take on Me" simplificada: pares (nota, divisor) */
static const int melody[] = {
    NOTE_FS5,8, NOTE_FS5,8, NOTE_D5,8, NOTE_B4,8, REST,8, NOTE_B4,8, REST,8, NOTE_E5,8,
    REST,8, NOTE_E5,8, REST,8, NOTE_E5,8, NOTE_GS5,8, NOTE_GS5,8, NOTE_A5,8, NOTE_B5,8,
    NOTE_A5,8, NOTE_A5,8, NOTE_A5,8, NOTE_E5,8, REST,8, NOTE_D5,8, REST,8, NOTE_FS5,8,
    REST,8, NOTE_FS5,8, REST,8, NOTE_FS5,8, NOTE_E5,8, NOTE_E5,8, NOTE_FS5,8, NOTE_E5,8,
};

#define MELODY_LEN (sizeof(melody)/sizeof(melody[0]))

static volatile int melody_pos        = 0;   /* índice actual en melody[] */
static volatile int note_remaining_ms = 0;   /* ms restantes de la nota */

/* =================== VARIABLES GLOBALES =================== */
static volatile cmd_t cmd_pending     = CMD_NONE;
volatile cmd_t       cmd_current     = CMD_S;    // comando actual

volatile uint32_t DeltaTicks=0, TickInicial=0, TickFinal=0;
volatile uint16_t ADC0Value=0, DistanciaINT=0;

/* OJO: TickTime depende del prescaler de TIM1 */
const float TickTime = 0.0001f;                  // 100 us (prescale=100 us)
volatile float DeltaTime=0, DistanciaCM=0;
uint32_t MatchVal= 499;                          // ~50 ms base para MAT0.1

volatile uint32_t flagObstaculo = 0;

/* =================== PROTOTIPOS =================== */
void cfgGPIO(void);
void cfgTimer0(uint32_t match_val);
void cfgTimer1(void);
void cfgADC(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
int  mapFloatToInt(float x);

static void cfgUART2_9600(void);
static void motors_stop(void);
static void motors_forward(void);
static void motors_reverse(void);
static void motors_turn_left(void);
static void motors_turn_right(void);

void lcd_init(void);
void lcd_gotoxy(uint8_t row, uint8_t col);
void lcd_print(const char *s);
static void lcd_data(uint8_t data);
static void lcd_cmd(uint8_t cmd);
static void lcd_send(uint8_t value, uint8_t rs);
static void lcd_write4(uint8_t nibble);
static void lcd_pulse_enable(void);

/* DAC / Audio */
static void cfgPinsDAC(void);
static void cfgDAC(void);
static void cfgTIM3_Siren(void);
static void cfgTimer2_MelodyTick(void);
static void set_frequency(float f);
static void start_melody(void);
static void stop_melody(void);
static void load_current_note(void);

/* ===================== MAIN ===================== */
int main(void) {

    SystemInit();
    SystemCoreClockUpdate();

    cfgGPIO();
    cfgTimer0(MatchVal);
    cfgTimer1();
    cfgADC();
    cfgUART2_9600();

    /* Configuración DAC + Timer3 para sirena/melodía */
    cfgPinsDAC();
    cfgDAC();
    cfgTIM3_Siren();
    cfgTimer2_MelodyTick();   // Timer2 para duración de notas
    set_frequency(F_MIN_HZ);
    siren_enabled = 0;
    melody_active = 0;

    motors_stop();   // deja los motores apagados al inicio

    lcd_init();
    lcd_gotoxy(1,1);
    lcd_print("Hola LPC1769");
    lcd_gotoxy(2,1);
    lcd_print("LCD 1602A OK");

    while(1)
    {
        __WFI();
    }

    return 0 ;
}

/* =================== CONFIGURACIONES =================== */
void cfgGPIO(void){

    PINSEL_CFG_Type p;

    /* UART2 pins */
    p.Portnum   = PINSEL_PORT_0;
    p.Pinnum    = PINSEL_PIN_10;
    p.Funcnum   = PINSEL_FUNC_1;
    p.Pinmode   = PINSEL_PINMODE_TRISTATE;
    p.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&p);

    p.Portnum   = PINSEL_PORT_0;
    p.Pinnum    = PINSEL_PIN_11;
    p.Funcnum   = PINSEL_FUNC_1;
    p.Pinmode   = PINSEL_PINMODE_TRISTATE;
    p.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&p);

    /* Motores como GPIO */
    GPIO_SetDir(MOT_PORT, MOT_MASK, 1);
    GPIO_ClearValue(MOT_PORT, MOT_MASK);

    /* Pines para ADC, sensor ultrasonido y MATx */
    PINSEL_CFG_Type cfgPin0_23;  // ADC0.0 - P0.23
    PINSEL_CFG_Type cfgPin1_18;  // Trigger HC-SR04 - P1.18 (GPIO out)
    PINSEL_CFG_Type cfgPin1_19;  // Echo HC-SR04 - P1.19 (CAP1.1)
    PINSEL_CFG_Type cfgPin1_29;  // MAT0.1 - P1.29

    // P0.23 -> ADC0.0
    cfgPin0_23.Portnum   = PINSEL_PORT_0;
    cfgPin0_23.Pinnum    = PINSEL_PIN_23;
    cfgPin0_23.Funcnum   = PINSEL_FUNC_1;
    cfgPin0_23.Pinmode   = PINSEL_PINMODE_TRISTATE;
    cfgPin0_23.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&cfgPin0_23);

    // P1.18 -> GPIO out (Trigger HC-SR04)
    cfgPin1_18.Portnum   = PINSEL_PORT_1;
    cfgPin1_18.Pinnum    = PINSEL_PIN_18;
    cfgPin1_18.Funcnum   = PINSEL_FUNC_0;
    cfgPin1_18.Pinmode   = PINSEL_PINMODE_TRISTATE;
    cfgPin1_18.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&cfgPin1_18);

    GPIO_SetDir(PORT_1, PIN_18, OUTPUT);
    GPIO_ClearValue(PORT_1, PIN_18);

    // P1.19 -> CAP1.1 (echo)
    cfgPin1_19.Portnum   = PINSEL_PORT_1;
    cfgPin1_19.Pinnum    = PINSEL_PIN_19;
    cfgPin1_19.Funcnum   = PINSEL_FUNC_3;
    cfgPin1_19.Pinmode   = PINSEL_PINMODE_TRISTATE;
    cfgPin1_19.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&cfgPin1_19);

    // P1.29 -> MAT0.1 (trigger ADC)
    cfgPin1_29.Portnum   = PINSEL_PORT_1;
    cfgPin1_29.Pinnum    = PINSEL_PIN_29;
    cfgPin1_29.Funcnum   = PINSEL_FUNC_3;
    cfgPin1_29.Pinmode   = PINSEL_PINMODE_TRISTATE;
    cfgPin1_29.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&cfgPin1_29);

    // Inicializar LEDs del RGB de la placa
    GPIO_SetDir(PORT_0, PIN_22, OUTPUT);
    GPIO_SetDir(PORT_3, PIN_25, OUTPUT);
    GPIO_SetDir(PORT_3, PIN_26, OUTPUT);

    GPIO_SetValue(PORT_0, PIN_22);
    GPIO_SetValue(PORT_3, PIN_25);
    GPIO_SetValue(PORT_3, PIN_26);
}

/* Timer0 para MAT0.1 -> trigger del ADC */
void cfgTimer0(uint32_t match_val){
    TIM_TIMERCFG_Type cfgTim0;
    TIM_MATCHCFG_Type cfgMatch1;

    cfgTim0.PrescaleOption = TIM_PRESCALE_USVAL;
    cfgTim0.PrescaleValue  = 1000;           // base 1 ms

    cfgMatch1.MatchChannel       = 1;
    cfgMatch1.ResetOnMatch       = ENABLE;
    cfgMatch1.StopOnMatch        = DISABLE;
    cfgMatch1.IntOnMatch         = DISABLE;
    cfgMatch1.ExtMatchOutputType = TIM_EXTMATCH_TOGGLE;
    cfgMatch1.MatchValue         = match_val;

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &cfgTim0);
    TIM_ConfigMatch(LPC_TIM0, &cfgMatch1);
    TIM_Cmd(LPC_TIM0, ENABLE);
}

void cfgTimer1(void){
    TIM_TIMERCFG_Type   cfgTim1;
    TIM_CAPTURECFG_Type cfgCAP1;

    cfgTim1.PrescaleOption = TIM_PRESCALE_USVAL;
    cfgTim1.PrescaleValue  = 100;       // base 100 us

    cfgCAP1.CaptureChannel = 1;         // CAP1.1
    cfgCAP1.RisingEdge     = ENABLE;
    cfgCAP1.FallingEdge    = ENABLE;
    cfgCAP1.IntOnCaption   = ENABLE;

    TIM_Init(LPC_TIM1, TIM_TIMER_MODE, &cfgTim1);
    TIM_ConfigCapture(LPC_TIM1, &cfgCAP1);
    TIM_Cmd(LPC_TIM1, ENABLE);

    NVIC_SetPriority(TIMER1_IRQn, 0);
    NVIC_EnableIRQ(TIMER1_IRQn);
}

void cfgADC(void){
    ADC_Init(LPC_ADC, 200000);                             // 200 kHz
    ADC_BurstCmd(LPC_ADC, DISABLE);
    ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);             // trigger MAT0.1
    ADC_EdgeStartConfig(LPC_ADC, ADC_START_ON_FALLING);
    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE);

    NVIC_SetPriority(ADC_IRQn, 1);
    NVIC_EnableIRQ(ADC_IRQn);
}

/* UART2 a 9600 baudios */
static void cfgUART2_9600(void)
{
    UART_CFG_Type ucfg;
    UART_FIFO_CFG_Type fcfg;

    UART_ConfigStructInit(&ucfg);
    ucfg.Baud_rate = 9600;
    UART_Init(UARTx, &ucfg);

    UART_FIFOConfigStructInit(&fcfg);
    UART_FIFOConfig(UARTx, &fcfg);

    UART_TxCmd(UARTx, ENABLE);
    UART_IntConfig(UARTx, UART_INTCFG_RBR, ENABLE);
    UART_IntConfig(UARTx, UART_INTCFG_RLS, ENABLE);
    NVIC_SetPriority(UART2_IRQn, 0);
    NVIC_EnableIRQ(UART2_IRQn);
}

/* ===================== DAC / SIRENA / MELODÍA ===================== */

static void cfgPinsDAC(void)
{
    PINSEL_CFG_Type p;
    p.Portnum   = PINSEL_PORT_0;
    p.Pinnum    = PINSEL_PIN_26;        /* P0.26 = AOUT */
    p.Funcnum   = PINSEL_FUNC_2;        /* Función 2 -> DAC */
    p.Pinmode   = PINSEL_PINMODE_TRISTATE;
    p.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&p);
}

static void cfgDAC(void)
{
    DAC_Init(LPC_DAC);
    DAC_SetBias(LPC_DAC, 0);
    DAC_UpdateValue(LPC_DAC, DAC_OFFSET);
}

/* Timer3 -> ISR de muestreo DAC a 20 kHz */
static void cfgTIM3_Siren(void)
{
    TIM_TIMERCFG_Type tcfg;
    TIM_MATCHCFG_Type mcfg;

    tcfg.PrescaleOption = TIM_PRESCALE_USVAL;
    tcfg.PrescaleValue  = 1;            // 1 tick = 1 us
    TIM_Init(LPC_TIM3, TIM_TIMER_MODE, &tcfg);

    mcfg.MatchChannel       = 0;
    mcfg.IntOnMatch         = ENABLE;
    mcfg.ResetOnMatch       = ENABLE;
    mcfg.StopOnMatch        = DISABLE;
    mcfg.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    mcfg.MatchValue         = 50;       // 50 us -> 20 kHz
    TIM_ConfigMatch(LPC_TIM3, &mcfg);

    TIM_Cmd(LPC_TIM3, ENABLE);
    NVIC_EnableIRQ(TIMER3_IRQn);
}

/* Timer2 -> tick de 1 ms para controlar la duración de cada nota */
static void cfgTimer2_MelodyTick(void)
{
    TIM_TIMERCFG_Type tcfg;
    TIM_MATCHCFG_Type mcfg;

    tcfg.PrescaleOption = TIM_PRESCALE_USVAL;
    tcfg.PrescaleValue  = 1000;    // 1 tick = 1 ms
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &tcfg);

    mcfg.MatchChannel       = 0;
    mcfg.IntOnMatch         = ENABLE;
    mcfg.ResetOnMatch       = ENABLE;
    mcfg.StopOnMatch        = DISABLE;
    mcfg.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    mcfg.MatchValue         = 1;   // 1 ms

    TIM_ConfigMatch(LPC_TIM2, &mcfg);
    TIM_Cmd(LPC_TIM2, ENABLE);
    NVIC_EnableIRQ(TIMER2_IRQn);
}

static void set_frequency(float f)
{
    if (f <= 0.0f) {
        /* "Frecuencia cero": dejamos step_q16 en 0 => silencio en DAC */
        step_q16 = 0;
        return;
    }

    if (f < F_MIN_HZ) f = F_MIN_HZ;
    if (f > F_MAX_HZ) f = F_MAX_HZ;
    f_curr = f;

    /* step_q16 = (f * LUT_SIZE / FS_HZ) * 2^16 */
    float step = (f * (float)LUT_SIZE) / (float)FS_HZ;
    uint32_t s  = (uint32_t)(step * 65536.0f + 0.5f);
    if (s == 0u) s = 1u;
    step_q16 = s;
}

/* --------- Melodía "Take on Me" --------- */

static void start_melody(void)
{
    melody_pos       = 0;
    note_remaining_ms = 0;
    melody_active    = 1;

    load_current_note();
}

static void stop_melody(void)
{
    melody_active    = 0;
    melody_pos       = 0;
    note_remaining_ms = 0;
    if (!siren_enabled) {
        step_q16 = 0;
        DAC_UpdateValue(LPC_DAC, DAC_OFFSET);
    }
}

/* Cargar nota actual desde melody[melody_pos] */
static void load_current_note(void)
{
    if (!melody_active) {
        stop_melody();
        return;
    }

    if (melody_pos < 0 || melody_pos >= (int)MELODY_LEN) {
        /* Fin de la canción */
        stop_melody();
        return;
    }

    int pitch   = melody[melody_pos];
    int divider = melody[melody_pos + 1];

    uint32_t duration = 0;
    if (divider > 0) {
        duration = WHOLENOTE_MS / (uint32_t)divider;
    } else {
        duration = WHOLENOTE_MS;  // fallback
    }
    if (duration == 0) duration = 1;

    note_remaining_ms = (int)duration;

    if (pitch == REST) {
        set_frequency(0.0f);
    } else {
        set_frequency((float)pitch);
    }
}

/* ===================================== Delays SIN SysTick ======================================= */

void delay_us(uint32_t us)
{
    /* Delay por lazo ocupado basado en SystemCoreClock.
       No depende de interrupciones. */
    uint32_t cycles_per_us = SystemCoreClock / 4000000u;
    if (cycles_per_us == 0u) cycles_per_us = 1u;

    uint32_t total = cycles_per_us * us;
    while (total--)
    {
        __NOP();
    }
}

void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        delay_us(1000u);
    }
}

/* ========================================= Mapeo ====================================== */
int mapFloatToInt(float x) {
    float mapped = (x - 0.0f) * (0xFFF - 0x0) / (200.0f - 0.0f) + 0x0;
    if (mapped < 0.0f)    mapped = 0.0f;
    if (mapped > 4095.0f) mapped = 4095.0f;
    return (uint16_t)mapped;
}

/* ========================================= MOTORES ====================================== */

static void motors_stop(void)
{
    lcd_cmd(1);
    lcd_gotoxy(1,1);
    lcd_print("Comando S:");
    lcd_gotoxy(2,1);
    lcd_print("STOP");
    GPIO_ClearValue(MOT_PORT, MOT_MASK);
}

static void motors_forward(void)
{
    lcd_cmd(1);
    lcd_gotoxy(1,1);
    lcd_print("Comando A:");
    lcd_gotoxy(2,1);
    lcd_print("AVANZAR");
    GPIO_ClearValue(MOT_PORT, MOT_MASK);

    uint32_t v = 0;
    v |= L_IN1; /* izq adelante */
    v |= R_IN1; /* der adelante */
    GPIO_ClearValue(MOT_PORT, MOT_MASK);
    GPIO_SetValue(MOT_PORT, v);
}

static void motors_reverse(void)
{
    lcd_cmd(1);
    lcd_gotoxy(1,1);
    lcd_print("Comando R:");
    lcd_gotoxy(2,1);
    lcd_print("REVERSA");
    GPIO_ClearValue(MOT_PORT, MOT_MASK);

    uint32_t v = 0;
    v |= L_IN2; /* izq atrás */
    v |= R_IN2; /* der atrás */
    GPIO_ClearValue(MOT_PORT, MOT_MASK);
    GPIO_SetValue(MOT_PORT, v);
}

static void motors_turn_left(void)
{
    lcd_cmd(1);
    lcd_gotoxy(1,1);
    lcd_print("Comando I:");
    lcd_gotoxy(2,1);
    lcd_print("IZQUIERDA");
    GPIO_ClearValue(MOT_PORT, MOT_MASK);

    uint32_t v = 0;
    v |= R_IN1; /* der adelante, izq stop */
    GPIO_ClearValue(MOT_PORT, MOT_MASK);
    GPIO_SetValue(MOT_PORT, v);
}

static void motors_turn_right(void)
{
    lcd_cmd(1);
    lcd_gotoxy(1,1);
    lcd_print("Comando D:");
    lcd_gotoxy(2,1);
    lcd_print("DERECHA");
    GPIO_ClearValue(MOT_PORT, MOT_MASK);

    uint32_t v = 0;
    v |= L_IN1; /* izq adelante, der stop */
    GPIO_ClearValue(MOT_PORT, MOT_MASK);
    GPIO_SetValue(MOT_PORT, v);
}

/* ========================================= PANTALLA LCD ====================================== */

static void lcd_pulse_enable(void) {
    PIN_SET (LCD_E_PORT, LCD_E_PIN);
    delay_us(1);            // >= 450 ns
    PIN_CLR (LCD_E_PORT, LCD_E_PIN);
    delay_us(50);           // t_instr
}

static void lcd_write4(uint8_t nibble) {
    PIN_CLR(LCD_D4_PORT, LCD_D4_PIN);
    PIN_CLR(LCD_D5_PORT, LCD_D5_PIN);
    PIN_CLR(LCD_D6_PORT, LCD_D6_PIN);
    PIN_CLR(LCD_D7_PORT, LCD_D7_PIN);

    if(nibble & 0x01) PIN_SET(LCD_D4_PORT, LCD_D4_PIN);
    if(nibble & 0x02) PIN_SET(LCD_D5_PORT, LCD_D5_PIN);
    if(nibble & 0x04) PIN_SET(LCD_D6_PORT, LCD_D6_PIN);
    if(nibble & 0x08) PIN_SET(LCD_D7_PORT, LCD_D7_PIN);

    lcd_pulse_enable();
}

static void lcd_send(uint8_t value, uint8_t rs) {
    if(rs) PIN_SET(LCD_RS_PORT, LCD_RS_PIN);
    else   PIN_CLR(LCD_RS_PORT, LCD_RS_PIN);

    lcd_write4((value >> 4) & 0x0F);
    lcd_write4( value        & 0x0F);
}

static void lcd_cmd(uint8_t cmd) {
    lcd_send(cmd, 0);
    if(cmd == 0x01 || cmd == 0x02) { // Clear / Home
        delay_ms(2);
    }
}

static void lcd_data(uint8_t data) {
    lcd_send(data, 1);
}

void lcd_gotoxy(uint8_t row, uint8_t col) {
    static const uint8_t baseAddr[2] = {0x00, 0x40};
    if(row < 1) row = 1;
    if(row > 2) row = 2;
    if(col < 1) col = 1;
    if(col > 16) col = 16;
    lcd_cmd(0x80 | (baseAddr[row-1] + (col-1)));
}

void lcd_print(const char *s) {
    while(*s) {
        lcd_data((uint8_t)*s++);
    }
}

void lcd_init(void) {
    PIN_OUT(LCD_RS_PORT, LCD_RS_PIN);
    PIN_OUT(LCD_E_PORT,  LCD_E_PIN);
    PIN_OUT(LCD_D4_PORT, LCD_D4_PIN);
    PIN_OUT(LCD_D5_PORT, LCD_D5_PIN);
    PIN_OUT(LCD_D6_PORT, LCD_D6_PIN);
    PIN_OUT(LCD_D7_PORT, LCD_D7_PIN);

    PIN_CLR(LCD_RS_PORT, LCD_RS_PIN);
    PIN_CLR(LCD_E_PORT,  LCD_E_PIN);

    delay_ms(40);

    lcd_write4(0x03); delay_ms(5);
    lcd_write4(0x03); delay_us(150);
    lcd_write4(0x03); delay_us(150);
    lcd_write4(0x02); // 4-bit

    lcd_cmd(0x28);    // 4-bit, 2 líneas, 5x8
    lcd_cmd(0x08);    // display OFF
    lcd_cmd(0x01);    // clear
    delay_ms(2);
    lcd_cmd(0x06);    // entry mode
    lcd_cmd(0x0C);    // display ON, cursor OFF
}

/* ===================================== IRQs =================== */

void ADC_IRQHandler(void){

    ADC0Value = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0); // valor actual del ADC

    // Pulso de ~10us para triggear el HC-SR04 SIN usar delay_us (para no bloquear demasiado)
    GPIO_SetValue(PORT_1, PIN_18);
    for (volatile uint32_t i = 0; i < 200; i++) {
        __NOP();
    }
    GPIO_ClearValue(PORT_1, PIN_18);
}

void TIMER1_IRQHandler(void){

    if(GPIO_ReadValue(PORT_1) & PIN_19){
        // Flanco ascendente: inicio del pulso ECHO
        TickInicial = TIM_GetCaptureValue(LPC_TIM1, TIM_COUNTER_INCAP1);
    }
    else{
        // Flanco descendente: fin del pulso ECHO
        TickFinal   = TIM_GetCaptureValue(LPC_TIM1, TIM_COUNTER_INCAP1);
        DeltaTicks  = (TickFinal - TickInicial);
        DeltaTime   = DeltaTicks * TickTime;         // [s]
        DistanciaCM = (DeltaTime * 34300.0f) / 2.0f; // [cm]

        if(DistanciaCM > 200.0f){
            DistanciaCM = 200.0f;
        }

        DistanciaINT = mapFloatToInt(DistanciaCM);

        if(DistanciaINT < ADC0Value){
            // Obstáculo: LED rojo
            GPIO_ClearValue(PORT_0, PIN_22);
            GPIO_SetValue  (PORT_3, PIN_25);
            GPIO_SetValue  (PORT_3, PIN_26);
            if(flagObstaculo == 0){
                flagObstaculo = 1;
                motors_stop();
            }
        }
        else{
            // Distancia segura: LED verde
            GPIO_SetValue  (PORT_0, PIN_22);
            GPIO_ClearValue(PORT_3, PIN_25);
            GPIO_SetValue  (PORT_3, PIN_26);
            flagObstaculo = 0;
        }
    }

    TIM_ClearIntPending(LPC_TIM1, TIM_CR1_INT);
}

/* Timer2: 1 ms -> controla duración de notas de la melodía */
void TIMER2_IRQHandler(void)
{
    TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);

    if (!melody_active)
        return;

    if (note_remaining_ms > 0) {
        note_remaining_ms--;
        return;
    }

    /* Pasar a la siguiente nota (cada nota ocupa 2 enteros) */
    melody_pos += 2;
    if (melody_pos >= (int)MELODY_LEN) {
        stop_melody();
        return;
    }

    load_current_note();
}

/* Sirena / melodía: ISR de Timer3 -> genera la onda en el DAC */
void TIMER3_IRQHandler(void)
{
    static uint32_t ms_cnt = 0;

    /* Si no hay audio activo, dejar el DAC en valor medio y salir */
    if (!siren_enabled && !melody_active) {
        DAC_UpdateValue(LPC_DAC, DAC_OFFSET);
        TIM_ClearIntPending(LPC_TIM3, TIM_MR0_INT);
        return;
    }

    /* 1) Actualizar fase y obtener índice en la LUT */
    phase_q16 += step_q16;
    uint32_t idx = (phase_q16 >> 16) % LUT_SIZE;
    uint32_t val = sineLUT[idx];

    /* 2) Re-escalar por amplitud y centrar */
    uint32_t out = DAC_OFFSET + ( (int32_t)val - 512 ) * DAC_AMPL / 511;
    if (out > 1023u) out = 1023u;
    DAC_UpdateValue(LPC_DAC, out);

    /* 3) Solo si es sirena (no melodía), hacer barrido de frecuencia */
    if (siren_enabled && !melody_active)
    {
        if (++ms_cnt >= 20u)  // cada 20 muestras (1 ms a 20 kHz)
        {
            ms_cnt = 0;
            float f = f_curr + df_ms;  // barrido triangular
            if (f >= F_MAX_HZ) { f = F_MAX_HZ; df_ms = -DF_PER_MS; }
            else if (f <= F_MIN_HZ) { f = F_MIN_HZ; df_ms = DF_PER_MS; }
            set_frequency(f);
        }
    }
    else
    {
        ms_cnt = 0;  // reseteo el contador cuando no es sirena
    }

    TIM_ClearIntPending(LPC_TIM3, TIM_MR0_INT);
}

void UART2_IRQHandler(void)
{
    uint32_t iid = UART_GetIntId(UARTx);

    if ((iid & UART_IIR_INTID_RDA) || (iid & UART_IIR_INTID_CTI))
    {
        uint8_t d;
        while (UART_Receive(UARTx, &d, 1, NONE_BLOCKING))
        {
            switch (d)
            {
                case 'A': cmd_pending = CMD_A; break;
                case 'S': cmd_pending = CMD_S; break;
                case 'D': cmd_pending = CMD_D; break;
                case 'I': cmd_pending = CMD_I; break;
                case 'R': cmd_pending = CMD_R; break;

                case 'X': /* toggle sirena barrido */
                    if (siren_enabled) {
                        siren_enabled = 0;
                        if (!melody_active) {
                            set_frequency(0.0f);
                        }
                    } else {
                        siren_enabled = 1;
                        melody_active = 0;  // la melodía se apaga
                        f_curr = F_MIN_HZ;
                        df_ms  = DF_PER_MS;
                        set_frequency(F_MIN_HZ);
                    }
                    break;

                case 'B': /* toggle melodía */
                    if (melody_active) {
                        stop_melody();
                    } else {
                        /* apagamos sirena y arrancamos melodía */
                        siren_enabled = 0;
                        start_melody();
                    }
                    break;

                default: break;
            }
        }
    }

    cmd_t c = cmd_pending;
    if (c != CMD_NONE)
    {
        switch (c)
        {
            case CMD_A: motors_forward();    break;
            case CMD_S: motors_stop();       break;
            case CMD_D: motors_turn_right(); break;
            case CMD_I: motors_turn_left();  break;
            case CMD_R: motors_reverse();    break;
            default: break;
        }

        cmd_current     = c;    // guardo cuál comando quedó activo
        cmd_pending     = CMD_NONE;
    }
}
