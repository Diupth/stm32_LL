#include "ComMgr.h"
#include "Receiver.h"
#include "SyncSignalApp.h"
#include "SystemClock.h"
#include "Transmitter.h"
#include "stm32h5xx.h"

void Error_Handler(void);

int main(void) {
  HAL_Init();
  SystemClock_Config();
  ComMgr_Init();

  // Initialize applications
  Transmitter_Init();
  Receiver_Init();
  SyncSignalApp_Init(); // Initialize timer last to start conversions

  while (1) {
    // 1. Process USB tasks (cdc flush, tud task)
    ComMgr_Process();

    // 2. Receive a physical frame and forward it over USB
    Receiver_Process();

    // 3. Send periodic synchronization telemetry
    SyncSignalApp_Process();

#ifdef SIMULATION_MODE
    // 4. Update simulation noise periodically
    Transmitter_UpdateNoise();
#endif
  }
}

// Ngắt SysTick dùng để tăng biến đếm thời gian hệ thống của HAL (uwTick)
void SysTick_Handler(void) { HAL_IncTick(); }

void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}
