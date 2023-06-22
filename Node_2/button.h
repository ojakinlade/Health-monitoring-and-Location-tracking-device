#pragma once

#define BUTTON_PRESSED        0
#define BUTTON_NOT_PRESSED    1

typedef struct
{
  uint8_t pin;
  bool state;
  bool prevPressed;
  bool isDebounced;
}Button_t;

class BUTTON
{
  private:
    bool Debounce(Button_t* pButton);

  public:
    BUTTON(Button_t* pButton);
    void Poll(Button_t* pButton);   
};
