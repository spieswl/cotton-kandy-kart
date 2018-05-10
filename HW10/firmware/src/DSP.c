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
    
    return result;
}

signed short finImpRespFilter(signed short * data, unsigned char sampCount)
{
    unsigned char index;
    signed short weights[8] = {0,0,0,0,0,0,0,0};
    signed short result = 0;
    
    for(index = 0; index < sampCount; ++index)
    {
        result += (weights[index] * data[index]);
    }
    
    return result;
}

signed short infImpRespFilter(signed short * data, signed short priorValue, signed short weightOld, signed short weightNew)
{
    signed short result = 0;
    
    result = ((weightOld * priorValue) + (weightNew * data[0])) / 100;
    
    return result;
}
