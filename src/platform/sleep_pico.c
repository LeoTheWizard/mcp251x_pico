/**
 * @file sleep.c
 * @brief Sleep functions for the raspberry pico platform.
 */

#include "mcp251x/platform/sleep.h"

#include "pico/time.h"

void mcp251x_sleep_ms(unsigned int milliseconds)
{
    sleep_ms(milliseconds);
}

void mcp251x_sleep_us(unsigned int microseconds)
{
    sleep_us(microseconds);
}