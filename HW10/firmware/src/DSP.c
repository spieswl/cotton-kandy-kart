#include <xc.h>
#include "DSP.h"

void clear_data_array(signed short * data_array, unsigned char length)
{
    unsigned char index;
    
    for(index = 0; index < length; ++index)
    {
        data_array[index] = 0;
    }
}

signed short movAvgFilter(signed short * data, unsigned char sampCount)
{
    unsigned char index;
    signed short result = 0;
    
    for(index = 0; index < sampCount; ++index)
    {
        result += data[index];
    }
    
    return (result / sampCount);
}

signed short finImpRespFilter(signed short * data, unsigned char sampCount)
{
    unsigned char index;
    float weights[10] = {0.0022, 0.0174, 0.0737, 0.1662, 0.2405, 0.2405, 0.1662, 0.0737, 0.0174, 0.0022};
    float result = 0;
    
    for(index = 0; index < sampCount; ++index)
    {
        result += (weights[index] * (float) data[index]);
    }
    
    return ((signed short) result);
}

signed short infImpRespFilter(signed short * data, signed short priorValue, float weightOld, float weightNew)
{
    float result = 0.0;
    
    result = ((weightOld * (float) priorValue) + (weightNew * (float) data[0]));
    
    return ((signed short) result);
}
