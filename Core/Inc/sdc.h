/*
 * sdc.h
 *
 *  Created on: July 30, 2025
 *      Author: Juliana
 */

#ifndef INC_SDC_H_
#define INC_SDC_H_

#include <stdint.h>
#include "GopherCAN.h"

#define SDC_NUM_BREAKPOINTS 19

extern const uint8_t *sdcStatusParams[SDC_NUM_BREAKPOINTS];

/* FOR REFERENCE */
// 2. PLM.9 -> Inertial Switch.1 ~ PLM
// 3. Inertial Switch.2 -> BOT.1 ~ FVC.7
// 4. BOT.1 -> Dash.2 ~ FVC.8
// 5. Dash.3 -> Estop_R.1 ~ PLM.20
// 6. Estop_R.2 -> Estop_L.1 ~ PLM.21
// 7. Estop_L.2 -> Inverter_Relay.3 ~ PLM.22
// 8. Inverter_Relay.4 -> RVC.6 (BSPD) ~ RVC internal sensing
// 9. RVC.7 -> BMS_LV.9 ~ RVC internal/BMS internal
// 10. BMS_LV.20 -> IO_Panel.1 (TSMS) ~ BMS Internal
// 11. IO_Panel.2 (or Charger_SDC_Relay.4) -> TSMP(interlock).2 ~ BMS.9
// 12. TMSP(interlock).1 -> EMeter(interlock).6 ~ BMS.10
// 13. Emeter(interlock).5 -> TVIM_Interlock.2  ~ BMS.11
// 14. TVIM_Interlock.1 -> MSD_Interlock.1 ~ BMS.12
// 15. MSD_Interlock.2 -> HV_Interlock.2 ~ BMS.13
// 16. HV_Interlock.1 -> TVIM_LV.1 (and AIR_Neg_CTRL.1) ~ BMS.14
// 17. *IO_Panel.1 (TSMS) -> *SDC_Switch.3 - only when charger board connected
// 18.  SDC_Switch.2 -> (charger)Estop.1 - only when charger board connected
// 19. (charger)Estop.2 -> Charger_SDC_Relay.3 - only when charger board connected


#endif /* INC_SDC_H_ */