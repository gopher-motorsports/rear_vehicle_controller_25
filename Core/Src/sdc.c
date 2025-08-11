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
    &sdcStatus5.data,              // &sdcStatus5.data,
    &sdcStatus6.data,              // &sdcStatus6.data,
    &sdcStatus7.data,              // &sdcStatus7.data,
    &sdcStatus8.data,
    &sdcStatus9.data,
    &zero,              // &sdcStatus10.data,
    &bmsShutdown6_state.data,              // &sdcStatus11.data,
    &bmsShutdown5_state.data,              // &sdcStatus12.data,
    &bmsShutdown4_state.data,              // &sdcStatus13.data,
    &bmsShutdown3_state.data,              // &sdcStatus14.data,
    &bmsShutdown2_state.data,              // &sdcStatus15.data,
    &bmsShutdown1_state.data,              // &sdcStatus16.data,
    &sdcStatus17.data,
    &sdcStatus18.data,
    &sdcStatus19.data
};
