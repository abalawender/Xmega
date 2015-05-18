#include <avr/io.h>
#include <avr/interrupt.h>
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
#include "twim.h"

#define TWI_MASTER       TWIC
#define TWI_MASTER_PORT  PORTC
#define TWI_SPEED        50000
#define TWI_MASTER_ADDR  0x50

unsigned char readOV7670( unsigned char _addr ) {
        twi_package_t packet = {
            .addr_length = 0,
            .chip        = 0x21,
            .buffer      = (void *)&_addr,
            .length      = 1,
            .no_wait     = false
        };

        twi_master_write(&TWI_MASTER, &packet);
        twi_master_read(&TWI_MASTER, &packet);
        return *(unsigned int *)packet.buffer;
}
void writeOV7670( unsigned char _addr, unsigned char _data ) {
        unsigned char c[2] = {_addr, _data};
        twi_package_t packet = {
            .addr_length = 0,
            .chip        = 0x21,
            .buffer      = (void *)c,
            .length      = 2,
            .no_wait     = false
        };

        twi_master_write(&TWI_MASTER, &packet);
}

static volatile bool main_b_cdc_enable = false;

int main(void)
{
    //Osc32MHz();
    PORTC.DIR = 0xFF;
    PORTB.DIR = 0x0;
    PORTF.DIR = 0x0;
    PORTE.DIR = 0x0;

    PORTB.PIN0CTRL = PORT_OPC_PULLDOWN_gc;
    PORTB.PIN1CTRL = PORT_OPC_PULLDOWN_gc;
    PORTB.PIN2CTRL = PORT_OPC_PULLDOWN_gc;
    PORTB.PIN3CTRL = PORT_OPC_PULLDOWN_gc;
    PORTB.PIN4CTRL = PORT_OPC_PULLDOWN_gc;
    PORTB.PIN5CTRL = PORT_OPC_PULLDOWN_gc;
    PORTB.PIN6CTRL = PORT_OPC_PULLDOWN_gc;
    PORTB.PIN7CTRL = PORT_OPC_PULLDOWN_gc;

    PORTF.PIN0CTRL = PORT_OPC_PULLDOWN_gc;
    PORTF.PIN1CTRL = PORT_OPC_PULLDOWN_gc;
    PORTF.PIN2CTRL = PORT_OPC_PULLDOWN_gc;
    PORTF.PIN3CTRL = PORT_OPC_PULLDOWN_gc;
    PORTF.PIN4CTRL = PORT_OPC_PULLDOWN_gc;
    PORTF.PIN5CTRL = PORT_OPC_PULLDOWN_gc;
    PORTF.PIN6CTRL = PORT_OPC_PULLDOWN_gc;
    PORTF.PIN7CTRL = PORT_OPC_PULLDOWN_gc;

    //! the page address to write to
    uint8_t page_address;
    //! the column address, or the X pixel.
    uint8_t column_address;
    //! store the LCD controller start draw line
    uint8_t start_line_address = 0;

    //board_init();
    sysclk_init(); //modified, disabled disabling

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
            ssd1306_write_data(0x0);
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
    //PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm |
    //    PMIC_HILVLEN_bm;
    //sei();

    //udc_start();

    //uint8_t data[8*128] = { 0 };
    //for( int i = 0; i < 8*128; ++i )
    //    data[i] = i%4;

    //ssd1306_display( data );

    //_delay_ms( 100 );



    // - - - konfiguracja timera
    //  sysclk_enable_module(SYSCLK_PORT_C, SYSCLK_TC0);
    //  sysclk_enable_module(SYSCLK_PORT_C, SYSCLK_HIRES);
    //
    //  PORTC.DIR |= 1;
    //  TCC0.CCABUF = 1; // period * 0.5
    //  TCC0.PER = 2;
    //  TCC0.CTRLA = (TCC0.CTRLA & ~TC0_CLKSEL_gm) | TC_CLKSEL_DIV1_gc;
    //  TCC0.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm;
    //
    //    for(int i = 1; 1; ++i)  {
    //        ssd1306_set_column_address(0);
    //        ssd1306_set_page_address(0);
    //        ssd1306_write_text("val: ");
    //        ssd1306_write_num( TCE0.CNT+i );
    //        ssd1306_write_text(" -> ok ");
    //        ssd1306_write_num( i );
    //        ssd1306_write_text(" -> ");
    //        ssd1306_write_num( TCC0.CNT );
    //    }
    //
    // - - - konfiguracja timera
//    sysclk_enable_module(SYSCLK_PORT_D, SYSCLK_TC0);
//    sysclk_enable_module(SYSCLK_PORT_D, SYSCLK_HIRES);
//
    PORTD.DIR |= 7; //0b111
    PORTD.DIR |= 7; //0b111
//    TCD0.CCABUF = 256; // period * 0.5
//    TCD0.PER = 512;
//    TCD0.CTRLA = (TCD0.CTRLA & ~TC0_CLKSEL_gm) | TC_CLKSEL_DIV1024_gc;
//    TCD0.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm;

    twi_options_t m_options = {
        .speed     = TWI_SPEED,
        .chip      = TWI_MASTER_ADDR,
        .speed_reg = TWI_BAUD(sysclk_get_cpu_hz(), TWI_SPEED)
    };
    TWI_MASTER_PORT.PIN0CTRL = PORT_OPC_WIREDANDPULL_gc;
    TWI_MASTER_PORT.PIN1CTRL = PORT_OPC_WIREDANDPULL_gc;
    //irq_initialize_vectors();
    sysclk_enable_peripheral_clock(&TWI_MASTER);
    twi_master_init(&TWI_MASTER, &m_options);
    twi_master_enable(&TWI_MASTER);

    //unsigned char addr = 0x0C;
    //ssd1306_set_page_address(3);
    //ssd1306_set_column_address(0);
    //ssd1306_write_num( addr );
    //ssd1306_write_text(" -> ");
    //ssd1306_write_num( readOV7670(addr) );
    //ssd1306_write_text(" | ");
    //writeOV7670( addr, readOV7670( addr ) | 0b00001000 );
    //ssd1306_write_num( addr );
    //ssd1306_write_text(" -> ");
    //ssd1306_write_num( readOV7670(addr) );
    //    _delay_ms( 500 );

    //addr = 0x12;
    //ssd1306_set_page_address(4);
    //ssd1306_set_column_address(0);
    //ssd1306_write_num( addr );
    //ssd1306_write_text(" -> ");
    //ssd1306_write_num( readOV7670(addr) );
    //ssd1306_write_text(" | ");
    //writeOV7670( addr, readOV7670( addr ) | 0b00001000 );
    //ssd1306_write_num( addr );
    //ssd1306_write_text(" -> ");
    //ssd1306_write_num( readOV7670(addr) );

    //addr = 0x11;
    //ssd1306_set_page_address(5);
    //ssd1306_set_column_address(0);
    //ssd1306_write_num( addr );
    //ssd1306_write_text(" -> ");
    //ssd1306_write_num( readOV7670(addr) );
    //ssd1306_write_text(" | ");
    //writeOV7670( addr, ( readOV7670( addr ) | 0x2 ) & 0b10111111 );
    //ssd1306_write_num( addr );
    //ssd1306_write_text(" -> ");
    //ssd1306_write_num( readOV7670(addr) );

uint8_t regs[] = {
0x11, 0x01,    // CLKRC
0x12, 0x8, //0x00,    // COM7, 0x8=qcif
0x0C, 0x0C,    // COM3
0x3E, 0x1a, //0x12,    // COM14
0x6B, 0x3A,    // DBLV
0x70, 0x3A,    // SCALING_XSC
0x71, 0x35,    // SCALING_YSC
0x72, 0x22,    // SCALING_DCWCTR
0x73, 0xF2,    // SCALING_PCLK_DIV
0xA2, 0x2A,    // SCALING_PCLK_DELAY
0 };

    for( int i = 0; regs[i]; i += 2 ) {
        writeOV7670( regs[i], regs[i+1] );
        ssd1306_set_page_address(5);
        ssd1306_set_column_address(0);
        ssd1306_write_num( i );
        ssd1306_write_text( "   ");
        //ssd1306_write_num( readOV7670(addr) );
    }

    PORTE.DIRCLR    =    PIN0_bm;                     // pin E0 jako wejście (VSYNC)
    PORTE.PIN0CTRL  =    PORT_OPC_PULLDOWN_gc|          // podciągnięcie do masy
                         PORT_ISC_FALLING_gc;         // zdarzenie mam wywoływać zbocze malejące
    // konfiguracja systemu zdarzeń
    EVSYS.CH0MUX    =    EVSYS_CHMUX_PORTE_PIN0_gc;   // pinE0 wywołuje zdarzenie
    EVSYS.CH0CTRL   =    EVSYS_DIGFILT_2SAMPLES_gc;   // filtr cyfrowy
    // konfiguracja timera
    TCD0.CTRLB      =    TC_WGMODE_NORMAL_gc;         // tryb normalny
    TCD0.CTRLA      =    TC_CLKSEL_EVCH0_gc;          // ustawienie źródła sygnały na kanał 0

    //sysclk_enable_module(SYSCLK_PORT_C, SYSCLK_TC0);
    //sysclk_enable_module(SYSCLK_PORT_C, SYSCLK_HIRES);
    //sysclk_enable_module(SYSCLK_PORT_C, SYSCLK_EVSYS);
    //sysclk_enable_module(SYSCLK_PORT_E, SYSCLK_EVSYS);
	//uint8_t *reg = (uint8_t *)&PR.PRGEN;
	//for (int i = 0; i <= SYSCLK_PORT_F; i++) {
	//	*(reg++) = 0;
	//}

    ssd1306_set_page_address(6);
    ssd1306_set_column_address(0);
    uint8_t data[8*128] = { 0 };

    // reset device
    PORTD.OUTSET = 7;
    PORTD.OUTCLR = 7;

    int counter = 0, i =0, cnt2;
    while( 1 ) {
        PORTD.OUTSET = 6;
        // RST WRITE + READ
        PORTD.OUTCLR = 6;
        _delay_ms(1);

        cnt2 = 0;
        for( i = 0; i < 176; ++i ) {
            data[i] = PORTF.IN;
            PORTD.OUTTGL = 1;
            _delay_us(1);
            if( data[i] ) cnt2++;
        }
        ssd1306_set_page_address(1);
        ssd1306_set_column_address(0);
        for( i= 0; i < 128; ++i )
            ssd1306_write_data( data[i] );
            
        //
        //ssd1306_write_num( TCC0.CNT );
        ssd1306_set_page_address(7);
        ssd1306_write_num( counter );
        ssd1306_write_text( " | " );
        ssd1306_write_num( TCD0.CNT );
        ssd1306_write_text( " | " );
        ssd1306_write_num( cnt2 );
        ssd1306_write_text( "   " );

        //for( int i = 0; i < sizeof(data); ++i ) {
        //    PORTD.OUTTGL = 1;
        //    data[i] = PORTF.IN;
        //}
        //ssd1306_display( data );
        //_delay_ms( 10 );
        //ssd1306_write_data( PORTF.IN );
        //_delay_us(500);
        //PORTD.OUTTGL = 1;
        //_delay_us(500);
        //PORTD.OUTTGL = 1;

        _delay_ms(1000);
        ++counter;
    }
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

