#ifndef _RM_CRC_H_
#define _RM_CRC_H_

#ifdef __cplusplus
extern "C" {
#endif
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned char Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
unsigned short Get_CRC16_Check_Sum(unsigned char *pchMessage,unsigned dwLength,unsigned short wCRC);
unsigned char Verify_CRC16_Check_Sum(unsigned char *pchMessage, unsigned dwLength);
void Append_CRC16_Check_Sum(unsigned char * pchMessage,unsigned dwLength);
#ifdef __cplusplus
}
#endif

#endif
