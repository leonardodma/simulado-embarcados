#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

// LED 1
#define LED1_PIO      PIOA
#define LED1_PIO_ID   ID_PIOC
#define LED1_IDX      0
#define LED1_IDX_MASK (1 << LED1_IDX)

// LED 2
#define LED2_PIO      PIOC
#define LED2_PIO_ID   ID_PIOC
#define LED2_IDX      30
#define LED2_IDX_MASK (1 << LED2_IDX)

// LED 3
#define LED3_PIO      PIOB
#define LED3_PIO_ID   ID_PIOB
#define LED3_IDX      2
#define LED3_IDX_MASK (1 << LED3_IDX)

// Botao1
#define BUT1_PIO      PIOD
#define BUT1_PIO_ID   ID_PIOD
#define BUT1_IDX  28
#define BUT1_IDX_MASK (1 << BUT1_IDX)

// Botao2
#define BUT2_PIO      PIOC
#define BUT2_PIO_ID   ID_PIOC
#define BUT2_IDX  31
#define BUT2_IDX_MASK (1 << BUT2_IDX)

// Botao3
#define BUT3_PIO      PIOA
#define BUT3_PIO_ID   ID_PIOA
#define BUT3_IDX  19
#define BUT3_IDX_MASK (1 << BUT3_IDX)

#define HEIGHT        5
#define MAX_WIDTH     130

/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */
typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} calendar;


/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/
volatile char flag_tc1 = 0;
volatile char flag_tc2 = 0;
volatile char flag_tc3 = 0;

volatile Bool f_rtt_alarme = false;
volatile char acende = 1;


volatile char flag_button1 = 0;
volatile char flag_button2 = 0;
volatile char flag_button3 = 0;

volatile char flag_rtc = 0;

char buffer[60];

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

void pisca_led(int n, int t, Pio * p_pio , uint32_t mask){
  for (int i=0;i<n;i++){
    pio_clear(p_pio, mask);
    delay_ms(t);
    pio_set(p_pio, mask);
    delay_ms(t);
  }
}

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void LED_init(int estado, Pio *p_pio, uint32_t id, uint32_t mask);
void BUT_init( Pio *p_pio ,uint32_t id, uint32_t mask, void *p_handler, uint32_t p_edge);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void pisca_led(int n, int t, Pio *  p_pio , uint32_t mask);


/************************************************************************/
/* INITS                                                              */
/************************************************************************/

void LED_init(int estado, Pio *p_pio ,uint32_t id, uint32_t mask ){
	pmc_enable_periph_clk(id);
	pio_set_output(p_pio, mask, estado, 0, 0);
};

//inicializa botoes
void BUT_init( Pio *p_pio ,uint32_t id, uint32_t mask, void *p_handler, uint32_t p_edge){
	// Inicializa clock do perif?rico PIO responsavel pelo botao
	pmc_enable_periph_clk(id);

	// Configura PIO para lidar com o pino do bot?o como entrada
	// com pull-up
	pio_configure(p_pio, PIO_INPUT, mask, PIO_PULLUP);
	pio_configure(p_pio, PIO_INPUT, mask, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(p_pio, mask, 60);

	// Configura interrup??o no pino referente ao botao e associa
	// fun??o de callback caso uma interrup??o for gerada
	// a fun??o de callback ? a: but_callback()
	pio_handler_set(p_pio,
	id,
	mask,
	p_edge,
	p_handler);

	// Ativa interrup??o
	pio_enable_interrupt(p_pio, mask);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr?ximo de 0 maior)
	NVIC_EnableIRQ(id);
	NVIC_SetPriority(id, 4); // Prioridade 4
}

/**
* Configura TimerCounter (TC) para gerar uma interrupcao no canal (ID_TC e TC_CHANNEL)
* na taxa de especificada em freq.
*/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	/* O TimerCounter � meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrup�c�o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrup�c�o no TC canal 0 */
	/* Interrup��o no C */
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN | RTT_MR_RTTINCIEN);
}

/**
* Configura o RTC para funcionar com interrupcao de alarme
*/
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

/************************************************************************/
/* HANDLERS                                                             */
/************************************************************************/

static void BUT1_Handler(void)
{
	if (flag_button1){
		flag_button1 = 0;
	}else{
		flag_button1 = 1;
	}
}

static void BUT2_Handler(void)
{
	if (flag_button2){
		flag_button2 = 0;
		}else{
		flag_button2 = 1;
	}
}

static void BUT3_Handler(void)
{
	if (flag_button3){
		flag_button3 = 0;
		}else{
		flag_button3 = 1;
	}
}

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc1 = 1;
}

void TC3_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC1, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc2 = 1;
}

void TC6_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC2, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc3 = 1;
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		f_rtt_alarme = false;
		
		
	}

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		if (acende){
			acende = 0;
		}else{
			acende = 1;	
		}
		f_rtt_alarme = true;                  // flag RTT alarme
	}
}

/**
* \brief Interrupt handler for the RTC. Refresh the display.
*/
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/* seccond tick	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		
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


/************************************************************************/
/* MAIN                                                              */
/************************************************************************/
int main (void)
{
	board_init();
	sysclk_init();
	delay_init();

  	// Init OLED
	gfx_mono_ssd1306_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	/* Inicializa Leds */
	LED_init(0, LED1_PIO, LED1_PIO_ID, LED1_IDX_MASK);
	LED_init(1, LED2_PIO, LED2_PIO_ID, LED2_IDX_MASK);
	LED_init(0, LED3_PIO, LED3_PIO_ID, LED3_IDX_MASK);

	//Inicializa botoes
	BUT_init(BUT1_PIO, BUT1_PIO_ID, BUT1_IDX_MASK, BUT1_Handler, PIO_IT_RISE_EDGE);
	BUT_init(BUT2_PIO, BUT2_PIO_ID, BUT2_IDX_MASK, BUT2_Handler, PIO_IT_FALL_EDGE);
	BUT_init(BUT3_PIO, BUT3_PIO_ID, BUT3_IDX_MASK, BUT3_Handler, PIO_IT_RISE_EDGE);
  
	// Inicializa os TCs
	TC_init(TC0, ID_TC1, 1, 5); //Configura timer TC0, canal 1, freq 5 
	TC_init(TC1, ID_TC3, 0, 10); //Configura timer TC1, canal 0, freq 10 
	TC_init(TC2, ID_TC6, 0, 1); //Configura timer TC2, canal 0, freq 1 
	
	// Inicializa RTT com IRQ no alarme.
	f_rtt_alarme = true;
	
	/** Configura RTC */
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN);

	// Escreve na tela um circulo e um texto
	gfx_mono_draw_string("5", 1,15, &sysfont);
	gfx_mono_draw_string("10", 60,15, &sysfont);
	gfx_mono_draw_string("1", 120,15, &sysfont);
	
	int tempo = 0;
	int width = 0;
	while(1) {
		if (f_rtt_alarme){
      
		  /*
		   * IRQ (interrupção ocorre) apos 4s => 4 pulsos por sengundo (0,25s) -> 16 pulsos são necessários para dar 4s
		   * tempo[s] = 0,25 * 16 = 4s
		   */
		  uint16_t pllPreScale = (int) (((float) 32768) / 5.0);
		  uint32_t irqRTTvalue = 25;
      
		  // reinicia RTT para gerar um novo IRQ
		  RTT_init(pllPreScale, irqRTTvalue);  
		       
		  tempo = 0;
		  width = 0;
		
		  gfx_mono_draw_filled_rect(0, 30, MAX_WIDTH, HEIGHT, GFX_PIXEL_CLR); // apaga retângulo

		  f_rtt_alarme = false;
		}
		
		if (acende){
			if (flag_button1){
				if(flag_tc1){
					pisca_led(1,10, LED1_PIO, LED1_IDX_MASK);
					flag_tc1 = 0;
				}
			}
			if (flag_button2){
				if(flag_tc2){
					pisca_led(1,10, LED2_PIO, LED2_IDX_MASK);
					flag_tc2 = 0;
				}
			}
			if (flag_button3){
				if(flag_tc3){
					pisca_led(1,10, LED3_PIO, LED3_IDX_MASK);
					flag_tc3 = 0;
					tempo+=1;
					width=tempo*(MAX_WIDTH/5);
				}
			}
		}
		else{
			if(flag_tc3){
				delay_ms(20);
				flag_tc3 = 0;
				tempo+=1;
				width=tempo*(MAX_WIDTH/5);
			}
		}
		//sprintf(buffer, "%d", tempo);
		//gfx_mono_draw_string(buffer, 50,20, &sysfont);
		gfx_mono_draw_filled_rect(0, 30, width, HEIGHT, GFX_PIXEL_SET);
		
		uint32_t h, m, s;
		rtc_get_time(RTC,&h,&m,&s);
		sprintf(buffer, "%lu:%lu:%lu", h, m, s);
		gfx_mono_draw_string(buffer, 35,2, &sysfont);
		
        pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
		
	}
}
