#ifndef _FILEFUNCTION
#define _FILEFUNCTION
#include "Global.h"

void WriteOffsetToFile(void);
void LoadOffsetFromFile(void);
void AddStorageParameter(char section[], char keyWord[], float *currentValue);
short LoadParameterFromFile(void);
short WriteParameterToFile(void);
#endif