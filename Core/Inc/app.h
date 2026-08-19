
#ifndef APP_H
#define APP_H

void App_Init(void);

/* Control loop step: read CAN status -> decide -> send commands -> LED.
   Run by ControlTask (fast, high priority). */
void App_ControlStep(void);

/* Diagnostics step: print the CAN trace to the console.
   Run by DiagTask (slow, low priority). */
void App_DiagStep(void);

#endif /* APP_H */
