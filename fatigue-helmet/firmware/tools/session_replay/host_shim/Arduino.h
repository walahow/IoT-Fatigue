// Host stand-in for what FuzzyFatigue.h takes from <Arduino.h> (uint8_t and
// constrain), so the firmware's fuzzy model compiles unchanged on the PC.
#pragma once
#include <stdint.h>
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
