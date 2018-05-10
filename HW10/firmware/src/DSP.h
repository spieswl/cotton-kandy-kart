#ifndef _DSP_H
#define _DSP_H

void clear_data_array(signed short * data_array, unsigned char length);

signed short movAvgFilter(signed short * data, unsigned char sampCount);
signed short finImpRespFilter(signed short * data, unsigned char sampCount);
signed short infImpRespFilter(signed short * data, signed short priorValue, float weightOld, float weightNew);

#endif /* _DSP_H */