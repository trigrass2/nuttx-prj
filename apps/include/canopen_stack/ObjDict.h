
/* File generated by gen_cfile.py. Should not be modified. */

#ifndef _OBJDICT_H
#define _OBJDICT_H

#include "canopen_stack/data.h"

/* Prototypes of function provided by object dictionnary */
UNS32 ObjDict_valueRangeTest (UNS8 typeValue, void * value);
const indextable * ObjDict_scanIndexOD (UNS16 wIndex, UNS32 * errorCode, ODCallback_t **callbacks);
extern int Canopen_Read_Enable;
/* Master node data struct */
extern CO_Data ObjDict_Data;
extern UNS16 ControlWordAxis1;		            /* Mapped at index 0x6040, subindex 0x00 */
extern UNS16 StatusWordAxis1;                    /* Mapped at index 0x6041, subindex 0x00 */
extern INTEGER32 PositionDemandValueAxis1;       /* Mapped at index 0x6062, subindex 0x00 */
extern INTEGER32 InterpolationDataAxis1;		    /* Mapped at index 0x60c1, subindex 0x01 */

extern UNS16 ControlWordAxis2;		            /* Mapped at index 0x6040, subindex 0x01 */
extern UNS16 StatusWordAxis2;                    /* Mapped at index 0x6041, subindex 0x01 */
extern INTEGER32 PositionDemandValueAxis2;       /* Mapped at index 0x6062, subindex 0x01 */
extern INTEGER32 InterpolationDataAxis2;		    /* Mapped at index 0x60c1, subindex 0x03 */
#endif // ObjDict_H
