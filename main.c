#include <avr/io.h>
#define F_CPU 32000000UL
#include <util/delay.h>

void Osc32MHz(void) {
    OSC.CTRL     =    OSC_RC32MEN_bm;       // włączenie oscylatora 32MHz
    while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // czekanie na ustabilizowanie się generatora
    CPU_CCP      =    CCP_IOREG_gc;         // odblokowanie zmiany źródła sygnału
    CLK.CTRL     =    CLK_SCLKSEL_RC32M_gc; // zmiana źródła sygnału na RC 32MHz
}


//int main() {
//    Osc32MHz();
//
//
//    PORTC.DIR = 0xFF;
//    TCC0.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm;
//    TCC0.PER = 2;
//    TCC0.CCA = 1;
//    TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
//
//    TWI_MASTER_t twiMaster;
//
//    //TC_CLKSEL_DIV2_gc
//    while(1);
//}

#include <ssd1306.h>
#include <udc.h>
#include <udi_cdc.h>
#include "conf_usb.h"
#include "uart.h"

static volatile bool main_b_cdc_enable = false;

int main(void)
{
    //Osc32MHz();
    PORTC.DIR = 0xFF;
    PORTB.DIR = 0;

	//! the page address to write to
	uint8_t page_address;
	//! the column address, or the X pixel.
	uint8_t column_address;
	//! store the LCD controller start draw line
	uint8_t start_line_address = 0;

	//board_init();
	sysclk_init();

	// Initialize SPI and SSD1306 controller
	ssd1306_init();

	// set addresses at beginning of display
	ssd1306_set_page_address(0);
	ssd1306_set_column_address(0);

	// fill display with lines
	for (page_address = 0; page_address <= 7; page_address++) {
		ssd1306_set_page_address(page_address);
		for (column_address = 0; column_address < 128; column_address++) {
			ssd1306_set_column_address(column_address);
			/* fill every other pixel in the display. This will produce
			horizontal lines on the display. */
			ssd1306_write_data(0xFF);
		}
	}

    // _delay_ms(500);
	// for (page_address = 0; page_address <= 7; page_address++) {
	// 	ssd1306_set_page_address(page_address);
	// 	for (column_address = 0; column_address < 128; column_address++) {
	// 		ssd1306_set_column_address(column_address);
	// 		/* fill every other pixel in the display. This will produce
	// 		horizontal lines on the display. */
	// 		ssd1306_write_data(0x00);
	// 	}
	// }
    //

  //Włączamy wszystkie poziomy przerwań (potrzebujemy tylko najniższy)
  PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm |
              PMIC_HILVLEN_bm;
  sei();

  udc_start();

  uint8_t data[8*128] = { 0 };
  for( int i = 0; i < 8*128; ++i )
      data[i] = i%2;

  ssd1306_display( data );

  _delay_ms( 1000 );

    // wejścia enkodera
    PORTCFG.MPCMASK    =    0b00000011;                       // wybór pinów 0 i 1 do konfiguracji
    PORTE.PIN0CTRL     =    PORT_ISC_LEVEL_gc |               // reagowanie na poziom niski
                            PORT_OPC_PULLUP_gc;               // podciągnięcie do zasilania
    PORTE.PIN1CTRL     =    PORT_ISC_LEVEL_gc |               // reagowanie na poziom niski
                            PORT_OPC_PULLUP_gc;               // podciągnięcie do zasilania

    // konfiguracja systemu zdarzeń
    EVSYS.CH0MUX       =    EVSYS_CHMUX_PORTE_PIN0_gc;        // pin C0 wywołuje zdarzenie
    EVSYS.CH0CTRL      =    EVSYS_QDEN_bm; //|                    // włączenie dekodera w systemie zdarzeń
                            //EVSYS_DIGFILT_8SAMPLES_gc;        // filtr cyfrowy

    // konfiguracja timera
    TCE0.CTRLA         =    TC_CLKSEL_EVCH0_gc;               // taktowanie systemem zdarzeń
    TCE0.CTRLD         =    TC_EVACT_QDEC_gc |                // włączenie dekodera kwadraturowego
                            TC_EVSEL_CH0_gc;                  // dekoder zlicza impulsy z kanału 0


    for(int i = 1; 1; ++i)  {
        ssd1306_set_column_address(0);
        ssd1306_set_page_address(0);
        ssd1306_write_text("val: ");
        ssd1306_write_num( TCE0.CNT );
        ssd1306_write_text(" -> ok ");
        ssd1306_write_num( i );
        ssd1306_write_text(" -> ");
        ssd1306_write_num( TCC0.CNT );
    }



  //while (1) {
  //  if( i == 128 ) {
  //      ssd1306_set_column_address(0);
  //      i = 0;
  //      ssd1306_set_page_address(r++);
  //      udi_cdc_write_buf( data, 128 );
  //  }
  //  if( r == 8 ) {
  //      ssd1306_set_page_address(0);
  //      r = 0;
  //  }

  //  if (udi_cdc_is_rx_ready()) {
  //      data[i] = udi_cdc_getc();
  //      ssd1306_write_data(data[i++]);
  //  }

  //}

  //      ssd1306_set_column_address(0);

// - - - konfiguracja timera
  sysclk_enable_module(SYSCLK_PORT_C, SYSCLK_TC0);
  sysclk_enable_module(SYSCLK_PORT_C, SYSCLK_HIRES);

  PORTC.DIR |= 1;
  TCC0.CCABUF = 1; // period * 0.5
  TCC0.PER = 2;
  TCC0.CTRLA = (TCC0.CTRLA & ~TC0_CLKSEL_gm) | TC_CLKSEL_DIV1_gc;
  TCC0.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm;

  while (1) { };
}



void main_suspend_action(void)
{
}

void main_resume_action(void)
{
}

void main_sof_action(void)
{
	if (!main_b_cdc_enable)
		return;
}

bool main_cdc_enable(uint8_t port)
{
	return true;
}

void main_cdc_disable(uint8_t port)
{
}

void main_cdc_set_dtr(uint8_t port, bool b_enable)
{
}

