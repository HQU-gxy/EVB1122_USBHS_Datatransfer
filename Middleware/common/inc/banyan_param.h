//
// Created by Kurosu Chan on 2024/5/17.
//

#ifndef BANYAN_PARAM_H
#define BANYAN_PARAM_H
#include "banyan.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const RADAR_REG_T InitRegList[] __attribute__((aligned (4)));
extern const RADAR_REG_T InitRegList_ABD[] __attribute__((aligned (4)));
extern const RADAR_REG_T InitRegList_MTT[] __attribute__((aligned (4)));
extern RADAR_REG_T InitChipRegListConfig0[MAX_REG_NUM] __attribute__((aligned (4)));
extern RADAR_REG_T InitChipRegListStart0[MAX_REG_FIX_NUM] __attribute__((aligned (4)));

#ifdef __cplusplus
}
#endif
#endif //BANYAN_PARAM_H
