#include "sdc.h"

// Define a dummy value for inactive or unimplemented entries
static const uint8_t zero = 0;

// Initialize array with live pointers or &zero placeholders
const uint8_t *sdcStatusParams[SDC_NUM_BREAKPOINTS] = 
{
    &zero,              // &sdcStatus1.data,
    &zero,              // &sdcStatus2.data,
    &sdcStatus3.data,
    &sdcStatus4.data,
    &zero,              // &sdcStatus5.data,
    &zero,              // &sdcStatus6.data,
    &zero,              // &sdcStatus7.data,
    &sdcStatus8.data,
    &sdcStatus9.data,
    &zero,              // &sdcStatus10.data,
    &zero,              // &sdcStatus11.data,
    &zero,              // &sdcStatus12.data,
    &zero,              // &sdcStatus13.data,
    &zero,              // &sdcStatus14.data,
    &zero,              // &sdcStatus15.data,
    &zero,              // &sdcStatus16.data,
    &sdcStatus17.data,
    &sdcStatus18.data,
    &sdcStatus19.data
};
