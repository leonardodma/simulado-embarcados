#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define LED_PIO_ID_1 ID_PIOA
#define LED_PIO_1 PIOA
#define LED_IDX_1 0
#define LED_IDX_MASK_1 (1 << LED_IDX_1)

#define LED_PIO_2       PIOC
#define LED_PIO_ID_2    ID_PIOC
#define LED_IDX_2       30
#define LED_IDX_MASK_2  (1u << LED_IDX_2)

#define LED_PIO_ID_3      ID_PIOB
#define LED_PIO_3         PIOB
#define LED_IDX_3		2
#define LED_IDX_MASK_3    (1<<LED_IDX_3)

#define BUT_PIO_1 PIOD
#define BUT_PIO_ID_1 ID_PIOD
#define BUT_IDX_1 28
#define BUT_IDX_MASK_1 (1 << BUT_IDX_1)

#define BUT_PIO_2 PIOC
#define BUT_PIO_ID_2 ID_PIOC
#define BUT_IDX_2 31
#define BUT_IDX_MASK_2 (1 << BUT_IDX_2)

#define BUT_PIO_3 PIOA
#define BUT_PIO_ID_3 ID_PIOA
#define BUT_IDX_3 19
#define BUT_IDX_MASK_3 (1 << BUT_IDX_3)

volatile char flag_rtc = 0;
volatile char flag_tick_rtc = 0;
volatile char flag_tc = 0;
void Button1_Handler();
void Button2_Handler();
void Button3_Handler();

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;
void pisca_led_3(int n, int t){
	for (int i=0;i<n;i++){
		pio_clear(LED_PIO_3, LED_IDX_MASK_3);
		delay_ms(t);
		pio_set(LED_PIO_3, LED_IDX_MASK_3);
		delay_ms(t);
	}
}
//inits:
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}
void LED_init(int estado){
	pmc_enable_periph_clk(LED_PIO_ID_1);
	pio_set_output(LED_PIO_1, LED_IDX_MASK_1, estado, 0, 0); //3o argumento fala como incializar o led, 0 eh ligado!!!!!!!!!!!
	pmc_enable_periph_clk(LED_PIO_ID_2);
	pio_set_output(LED_PIO_2, LED_IDX_MASK_2, estado, 0, 0);
	pmc_enable_periph_clk(LED_PIO_ID_3);
	pio_set_output(LED_PIO_3, LED_IDX_MASK_3, estado, 0, 0);
};

void BUT_init(void)
{

	// Inicializa clock do perif?rico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT_PIO_ID_1);
	pmc_enable_periph_clk(BUT_PIO_ID_2);
	pmc_enable_periph_clk(BUT_PIO_ID_3);

	// Configura PIO para lidar com o pino do bot?o como entrada
	// com pull-up
	pio_configure(BUT_PIO_1, PIO_INPUT, BUT_IDX_MASK_1, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_PIO_2, PIO_INPUT, BUT_IDX_MASK_2, PIO_PULLUP| PIO_DEBOUNCE);
	pio_configure(BUT_PIO_3, PIO_INPUT, BUT_IDX_MASK_3, PIO_PULLUP| PIO_DEBOUNCE);


	// Configura interrup??o no pino referente ao botao e associa
	// fun??o de callback caso uma interrup??o for gerada
	// a fun??o de callback ? a: but_callback()
	pio_handler_set(BUT_PIO_1,
	BUT_PIO_ID_1,
	BUT_IDX_MASK_1,
	PIO_IT_FALL_EDGE,
	Button1_Handler);
	
	pio_handler_set(BUT_PIO_2,
	BUT_PIO_ID_2,
	BUT_IDX_MASK_2,
	PIO_IT_FALL_EDGE,
	Button2_Handler);
	
	pio_handler_set(BUT_PIO_3,
	BUT_PIO_ID_3,
	BUT_IDX_MASK_3,
	PIO_IT_FALL_EDGE,
	Button3_Handler);

	// Ativa interrup??o
	pio_enable_interrupt(BUT_PIO_1, BUT_IDX_MASK_1);
	pio_enable_interrupt(BUT_PIO_2, BUT_IDX_MASK_2);
	pio_enable_interrupt(BUT_PIO_3, BUT_IDX_MASK_3);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr?ximo de 0 maior)
	NVIC_EnableIRQ(BUT_PIO_ID_1);
	NVIC_SetPriority(BUT_PIO_ID_1, 4); // Prioridade 4
	NVIC_EnableIRQ(BUT_PIO_ID_2);
	NVIC_SetPriority(BUT_PIO_ID_2, 4);
	NVIC_EnableIRQ(BUT_PIO_ID_3);
	NVIC_SetPriority(BUT_PIO_ID_3, 4);
}

//HANDLERS:
volatile int but_flag_1=0;
volatile int but_flag_2=0;
volatile int but_flag_3=0;
void Button1_Handler(void){
	but_flag_1 = 1;
}
void Button2_Handler(void){
	but_flag_2 = 1;
}
void Button3_Handler(void){
	but_flag_3 = 1;
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/* seccond tick	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		flag_tick_rtc=1;
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		flag_rtc = 1;
	}
	
	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}


int main (void)
{
	board_init();
	sysclk_init();
	delay_init();

	// Init OLED
	gfx_mono_ssd1306_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	/* Configura Leds */
	LED_init(1);

	/* Configura os bot?es */
	BUT_init();
	
	// Escreve na tela um circulo e um texto
	//gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
	//gfx_mono_draw_string("mundo", 50,16, &sysfont);
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);
	int sec_cro=0;
	int min_cro=0;
	int contagem=0;
	but_flag_1=0;
	but_flag_2=0;
	but_flag_3=0;
	/* Insert application code here, after the board has been initialized. */
	while(1) {
		if (flag_tick_rtc==1){
			uint32_t hr;
			uint32_t min;
			uint32_t sec;
			rtc_get_time(RTC, &hr, &min, &sec);
			char buffer1[32];
			char buffer2[32];
			sprintf(buffer1, "%.2d : %.2d : %.2d", hr, min, sec);
			sprintf(buffer2, "%.2d : %.2d", min_cro, sec_cro);
			gfx_mono_draw_string(buffer1, 0,0, &sysfont);
			gfx_mono_draw_string(buffer2, 0,15, &sysfont);
			flag_tick_rtc=0;
		}
		
		
		if(contagem==1){
			pio_clear(LED_PIO_1,LED_IDX_MASK_1);
			}else{
			pio_set(LED_PIO_1,LED_IDX_MASK_1);
		}
		
		if (but_flag_1==1){
			min_cro+=1;
			but_flag_1=0;
		}
		if(but_flag_2==1){
			sec_cro+=1;
			but_flag_2=0;
		}
		if (but_flag_3==1){
			contagem=1;
			but_flag_3=0;
			int hr;
			int min;
			int sec;
			rtc_get_time(RTC, &hr, &min, &sec);
			
			if (sec_cro+sec<=59){
				// configura alarme do RTC
				rtc_set_date_alarm(RTC, 1, rtc_initial.month, 1, rtc_initial.day);
				rtc_set_time_alarm(RTC, 1, hr, 1, min+min_cro, 1, sec + sec_cro);
				}else{
				rtc_set_date_alarm(RTC, 1, rtc_initial.month, 1, rtc_initial.day);
			rtc_set_time_alarm(RTC, 1, hr, 1, min+1+min_cro, 1, sec+sec_cro-60);}
			
			flag_rtc=0;
			
		}
		
		if(flag_rtc==1){
			pio_set(LED_PIO_1,LED_IDX_MASK_1);
			contagem=0;
			sec_cro=0;
			min_cro=0;
			flag_rtc=0;
			pisca_led_3(10,300);
		}
		//pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		

	}
}
