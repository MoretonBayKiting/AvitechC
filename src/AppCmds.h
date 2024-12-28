#ifndef APPCMDS_H
#define APPCMDS_H

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Function prototypes
void Cmd1(void);
void Cmd2(void);
void Cmd3(void);
void Cmd4(void);
void Cmd5(void);
void Cmd6(void);
void Cmd7(void);
void Cmd8(void);
void Cmd9(void);
void Cmd10(void);
void Cmd11(void);
void Cmd12(void);
void Cmd13(void);
void Cmd14(void);
// void Cmd15(void);
void Cmd16(void);
void Cmd17(void);
void Cmd18(void);
void Cmd19(void);
void Cmd20(void);
void Cmd21(void);
void Cmd22(void);
void Cmd23(void);
void Cmd24(void);
void Cmd25(void);
void Cmd26(void);
void Cmd27(void);
void Cmd28(void);
void Cmd29(void);
void Cmd30(void);
void Cmd31(void);
void Cmd32(void);
void Cmd33(void);
void Cmd34(void);
void Cmd35(void);
void Cmd36(void);
void Cmd37(void);
void Cmd38(void);
void Cmd39(void);

void Cmd45(void);
void Cmd46(void);
void Cmd47(void);
void Cmd48(void);
void Cmd49(void);
void Cmd50(void);
void Cmd51(void);
void Cmd52(void);
void Cmd54(void);
#ifdef NEW_APP
void CmdStorePts(bool test);
void GoToMapIndex();
void ReportVertices();
#endif
// void Cmd52(void);

void DecodeCommsData(void);
#endif // APPCMDS_H