#include <Arduino.h>
#include <MapleFreeRTOS900.h>
#include "button.h"

BUTTON::BUTTON(Button_t* pButton)
{
  pinMode(pButton->pin, INPUT_PULLUP);
}

bool BUTTON::Debounce(Button_t* pButton)
{
  pButton->state = digitalRead(pButton->pin);
  if(pButton->state == BUTTON_PRESSED)
  {
    vTaskDelay(pdMS_TO_TICKS(10));
    if(pButton->state == BUTTON_PRESSED)
    {
      return true;
    }
  }
  return false;
}

void BUTTON::Poll(Button_t* pButton)
{
  pButton->isDebounced = BUTTON::Debounce(pButton);
}
