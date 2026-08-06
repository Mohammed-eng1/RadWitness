/*
 * Arduino.h -- minimal stand-in so protocol.h compiles on a host PC
 * ==================================================================
 * NOT part of the firmware and never compiled into it. Its existence
 * lets the protocol be tested without an ESP32 board, keeping
 * protocol.h a SINGLE source instead of a second copy that drifts.
 *
 * ASCII only in this file -- see the BiDi note in controller_display.ino.
 */
#pragma once
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>
