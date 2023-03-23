#pragma once

typedef enum {
  KeyUnknown=-1,
  KeyEsc=0,
  KeyArrowUp=1,
  KeyArrowDown=2,
  KeyArrowRight=3,
  KeyArrowLeft=4,
  KeyS=5,
  KeyR=6,
  KeyD=7,
  KeyJ=8,
  KeyX=9,
  KeyI=10,
  KeyH=11
} KeyCode;

void setConioTerminalMode(void);

void resetTerminalMode(void);

KeyCode getKey();
