#ifndef SYNCSIGNALAPP_H
#define SYNCSIGNALAPP_H

#include <stdbool.h>

void SyncSignalApp_Init(void);
void SyncSignalApp_Process(void);
bool SyncSignalApp_HasFrames(void);
bool SyncSignalApp_WaitForFrames(void);

#endif /* SYNCSIGNALAPP_H */