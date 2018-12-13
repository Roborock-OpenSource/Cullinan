
#ifndef BMI160_H
#define BMI160_H

#include "BMI160_Reg.h"
#include "BMI160_Defs.h"

extern struct _BMI160_REG BMI160_REG;

int32_t BMI160_Init(void);
int32_t BMI160_SoftReset(void);


int32_t BMI160_SetAccelNormal(void);


int32_t BMI160_SetGyroNormal(void);



extern const float FS_2G_LSB_PER_G;
extern const float FS_4G_LSB_PER_G;
extern const float FS_8G_LSB_PER_G;
extern const float FS_16G_LSB_PER_G;

extern const float FS_2000_LSB_PER_DPS;
extern const float FS_1000_LSB_PER_DPS;
extern const float FS_500_LSB_PER_DPS;
extern const float FS_250_LSB_PER_DPS;
extern const float FS_125_LSB_PER_DPS;






#endif /* BMI160_H */
/* end of BMI160.h */
