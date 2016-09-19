#ifndef __Algorithm_filter_H
#define	__Algorithm_filter_H

float IIR_I_Filter(float InData, float *x, float *y, float *b, short nb, float *a, short na);
void LPF_1st(float *oldData, float *newData, float lpf_factor);
#endif /* __Algorithm_filter_H */
