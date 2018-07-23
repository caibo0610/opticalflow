#ifndef ADSPMSGD_H
#define ADSPMSGD_H
/*==============================================================================
  Copyright (c) 2013 Qualcomm Technologies, Inc.
  All rights reserved. Qualcomm Proprietary and Confidential.
==============================================================================*/

#include "AEEStdDef.h"

#ifdef __cplusplus
extern "C" {
#endif

int adspmsgd_start(int heapid, uint32 flags, int buf_size);
void adspmsgd_stop(void);

#ifdef __cplusplus
}
#endif

#endif // ADSPMSGD_H

