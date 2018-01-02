

#include <stdio.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include <onewire0.h>

#define DELAY 200

// 74HC595 pins
#define PIN_SER   2
#define PIN_RCLK  1
#define PIN_SRCLK 0

// Button
#define PIN_BTN   3

// PB4 used for onewire

#define LCD_RS_BIT  0
#define LCD_RW_BIT  1
#define LCD_E_BIT   2
#define LCD_DB7_BIT 3
#define LCD_DB6_BIT 4
#define LCD_DB5_BIT 5
#define LCD_DB4_BIT 6
#define LCD_BL_BIT  7

#define LCD_CLEAR_DISPLAY 0b00000001
#define LCD_RETURN_HOME   0b00000010

uint8_t do_read_temp = 1;
uint8_t bl_counter = 2;

void setup_pwr() {

    // Disable ADC
    ADCSRA &= ~_BV(ADEN);

    // Disable Analog comparator
    ACSR |= _BV(ACD);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

void configure_pcint() {

    PCMSK |= _BV(PCINT3);
    GIMSK |= _BV(PCIE);
}

void configure_watchdog() {

    // ~8 second
    WDTCR |= _BV(WDP3) | _BV(WDP0);
}

inline void enable_watchdog() {

    WDTCR |= _BV(WDIE) | _BV(WDCE) | _BV(WDE);
}

inline void enable_power_down() {

    cli();
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
}

void write(uint8_t output) {

    if (bl_counter) {
        output |= _BV(LCD_BL_BIT);
    } else {
        output &= ~_BV(LCD_BL_BIT);
    }

    for(int i = 7; i >= 0; i--) {
        if (output & (1 << i)) {
            PORTB |= (1<<PIN_SER);
        } else {
            PORTB &= ~(1<<PIN_SER);
        }
        PORTB |= (1<<PIN_SRCLK);
        PORTB &= ~(1<<PIN_SRCLK);
    }

    PORTB |= (1<<PIN_RCLK);
    PORTB &= ~(1<<PIN_RCLK);
}

void write_4bits(uint8_t command, bool rs) {

    uint8_t output = 0;

    if (rs) {
        output |= (1 <<LCD_RS_BIT);
    }

    // output &= ~(1<<LCD_RS_BIT);
    // output &= ~(1<<LCD_RW_BIT);
    // output &= ~(1<<LCD_E_BIT);

    output |= (1<<LCD_E_BIT);

    if (command & (1<<7)) {
        output |= (1<<LCD_DB7_BIT);
    }
    if (command & (1<<6)) {
        output |= (1<<LCD_DB6_BIT);
    }
    if (command & (1<<5)) {
        output |= (1<<LCD_DB5_BIT);
    }
    if (command & (1<<4)) {
        output |= (1<<LCD_DB4_BIT);
    }

    write(output);
    _delay_us(1);

    output &= ~(1<<LCD_E_BIT);

    write(output);
    _delay_us(1);

    output |= (1<<LCD_E_BIT);

    write(output);
    _delay_us(100);
}

void write_8bits(uint8_t command, bool rs) {

    write_4bits(command, rs);
    write_4bits(command << 4, rs);
}

void clear_display() {
    write_8bits(LCD_CLEAR_DISPLAY, false);
    _delay_ms(2);
}

void return_home() {
    write_8bits(LCD_RETURN_HOME, false);
    _delay_ms(2);
}

void write_char(unsigned char c) {
    write_8bits(c, true);
    _delay_us(40);
}

uint8_t bytes;
char arr[8];

int main(void)
{
    // Pins as output
    DDRB |= (1 << PIN_SER) | (1 << PIN_RCLK) | (1 << PIN_SRCLK);

    cli();
    onewire0_init();
    sei();

    // Input with pull-up
    DDRB &= ~(1 << PIN_BTN);
    PORTB |= (1 << PIN_BTN);

    setup_pwr();
    configure_pcint();
    configure_watchdog();

    // _delay_ms(2000);

    _delay_ms(50);
    write(0x00);

    write_4bits(0b00110000, false);
    _delay_ms(5);

    write_4bits(0b00110000, false);
    _delay_us(200);

    write_4bits(0b00110000, false);
    _delay_us(200);

    write_4bits(0b00100000, false);
    _delay_ms(5);

    // Function set
    write_4bits(0b00100000, false);
    write_4bits(0b10000000, false);
    _delay_ms(1);

    // Display off
    write_4bits(0b00000000, false);
    write_4bits(0b10000000, false);
    _delay_ms(1);

    clear_display();

    // Entry set
    write_4bits(0b00000000, false);
    write_4bits(0b01100000, false);
    _delay_ms(1);

    // Display on, cursor off, cursor blink
    write_4bits(0b00000000, false);
    write_4bits(0b11000000, false);
    _delay_ms(1);

    clear_display();
    return_home();

    while (true) {

        if (do_read_temp) {

            // Clear flag
            do_read_temp = 0;

            // Decrement backlight counter
            if (bl_counter > 0) {
                --bl_counter;
            }

            onewire0_reset();
            onewire0_skiprom();
            onewire0_convert();

            onewire0_convertdelay();
            while (! onewire0_isidle()) {}

            onewire0_reset();
            onewire0_skiprom();
            onewire0_readscratchpad();

            uint8_t lsb = onewire0_readbyte();
            uint8_t msb = onewire0_readbyte();

            int8_t e = (msb << 4) | (lsb >> 4 & 0xF);
            uint16_t f = 0;

            if (lsb & 0x08)
                f = f + 50;
            if (lsb & 0x04)
                f = f + 25;
            if (lsb & 0x02)
                f = f + 12;
            if (lsb & 0x01)
                f = f + 6;

            clear_display();

            sprintf(arr, "%u,%02u", e, f);

            char *p = arr;
            while(*p) {
                write_char(*p);
                p++;
            }

            write_char(223);
            write_char('C');
        }

        write(_BV(LCD_E_BIT));

        enable_watchdog();
        enable_power_down();
    }

    return 0;
}

ISR(PCINT0_vect) {

    bl_counter = 2;
}

ISR(WDT_vect) {

    do_read_temp = 1;
}
