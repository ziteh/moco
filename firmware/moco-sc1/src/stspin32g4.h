/**
 * @file   stspin32g4.h
 * @brief  STSPIN32G4 define.
 * @author ZiTe (honmonoh@gmail.com)
 */

#ifndef STSPIN32G4_H_
#define STSPIN32G4_H_

#define SPING4_I2C_ADDRESS (0x47) /* The 7-bit address. */

/* Registers. */
#define SPING4_REG_POWMNG (0x01) /* Power manager. */
#define SPING4_REG_LOGIC (0x02)  /* Driving logic. */
#define SPING4_REG_READY (0x07)  /* READY output. */
#define SPING4_REG_NFAULT (0x08) /* nFAULT output. */
#define SPING4_REG_CLEAR (0x09)  /* FAULT clear. */
#define SPING4_REG_STBY (0x0A)   /* Standby. */
#define SPING4_REG_LOCK (0x0B)   /* LOCK. */
#define SPING4_REG_RESET (0x0C)  /* RESET command. */
#define SPING4_REG_STATUS (0x80) /* Device STATUS. */

#endif /* STSPIN32G4_H_ */
