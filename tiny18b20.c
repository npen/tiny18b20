

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
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
uint8_t lcd_out = _BV(LCD_BL_BIT);
int16_t min_temp = 99;

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

void write_lcd() {

    for(int i = 7; i >= 0; i--) {
        if (lcd_out & (1 << i)) {
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

void write_4bits(uint8_t command) {

    // Write output on shift register for each pin change. Changing to many
    // pins at the same time can lead to badly interpreted command on the LCD.

    if (command & (1<<7)) {
        lcd_out |= _BV(LCD_DB7_BIT);
    } else {
        lcd_out &= ~_BV(LCD_DB7_BIT);
    }
    write_lcd();

    if (command & (1<<6)) {
        lcd_out |= _BV(LCD_DB6_BIT);
    } else {
        lcd_out &= ~_BV(LCD_DB6_BIT);
    }
    write_lcd();

    if (command & (1<<5)) {
        lcd_out |= _BV(LCD_DB5_BIT);
    } else {
        lcd_out &= ~_BV(LCD_DB5_BIT);
    }
    write_lcd();

    if (command & (1<<4)) {
        lcd_out |= _BV(LCD_DB4_BIT);
    } else {
        lcd_out &= ~_BV(LCD_DB4_BIT);
    }
    write_lcd();

    lcd_out |= _BV(LCD_E_BIT);
    write_lcd();
    _delay_us(1);

    lcd_out &= ~_BV(LCD_E_BIT);
    write_lcd();
    _delay_us(1);
}

void write_8bits(uint8_t command, bool rs) {

    lcd_out &= ~_BV(LCD_RW_BIT);
    if (rs) {
        lcd_out |= _BV(LCD_RS_BIT);
    } else {
        lcd_out &= ~_BV(LCD_RS_BIT);
    }
    write_lcd();

    write_4bits(command);
    write_4bits(command << 4);
}

void lcd_clear_out() {

    // LCD power consumption is reduced by enabling all outputs except E.

    lcd_out |= _BV(LCD_RS_BIT);
    write_lcd();

    lcd_out |= _BV(LCD_RW_BIT);
    write_lcd();

    lcd_out &= ~_BV(LCD_E_BIT);
    write_lcd();

    lcd_out |= _BV(LCD_DB7_BIT);
    write_lcd();

    lcd_out |= _BV(LCD_DB6_BIT);
    write_lcd();

    lcd_out |= _BV(LCD_DB5_BIT);
    write_lcd();

    lcd_out |= _BV(LCD_DB4_BIT);
    write_lcd();
}

void lcd_set_bl(bool enable) {

    if (enable) {
        lcd_out |= _BV(LCD_BL_BIT);
    } else {
        lcd_out &= ~_BV(LCD_BL_BIT);
    }
    write_lcd();
}

void clear_display() {
    write_8bits(LCD_CLEAR_DISPLAY, false);
    _delay_ms(2);
}

void return_home() {
    write_8bits(LCD_RETURN_HOME, false);
    _delay_ms(2);
}

void move_to(uint8_t addr) {
    write_8bits(0b0010000000 | addr, false);
    _delay_ms(2);
}

void write_char(unsigned char c) {
    write_8bits(c, true);
    _delay_us(40);
}

char arr[8];

void
write_degrees(int16_t value) {

    sprintf(arr, "%03d", value);

    uint8_t len = strlen(arr);

    for(uint8_t i = 0; i < (5 - len); i++) {
        write_char(' ');
    }

    for(uint8_t i = 0; i < len; i++) {
        if (i == len - 2) {
            write_char(',');
        }
        write_char(arr[i]);
    }

    write_char(223);
    write_char('C');
}

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
    write_lcd();

    write_4bits(0b00110000);
    _delay_ms(5);

    write_4bits(0b00110000);
    _delay_us(200);

    write_4bits(0b00110000);
    _delay_us(200);

    write_4bits(0b00100000);
    _delay_ms(5);

    // Function set
    write_4bits(0b00100000);
    write_4bits(0b10000000);
    _delay_ms(1);

    // Display off
    write_4bits(0b00000000);
    write_4bits(0b10000000);
    _delay_ms(1);

    clear_display();

    // Entry set
    write_4bits(0b00000000);
    write_4bits(0b01100000);
    _delay_ms(1);

    // Display on, cursor off, cursor blink
    write_4bits(0b00000000);
    write_4bits(0b11000000);
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


            int16_t value = (msb << 8) | (lsb);
            value = (int32_t)value * 100 / 16;

            if (min_temp > value) {
                min_temp = value;
            }

            clear_display();

            write_degrees(value);

            move_to(0x40);
            write_degrees(min_temp);

            lcd_clear_out();
        }

        lcd_set_bl(bl_counter > 0);

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
