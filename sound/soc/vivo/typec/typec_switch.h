
#ifndef TYPEC_ACCDET__H_
#define TYPEC_ACCDET__H_

//#include "tcpm.h"
#include "max20328-i2c.h"


extern void max20328_usbc_set_switch_mode(int mode);
void accdet_register_typec_notifier_call(void);
extern void vivo_set_typec_power_role(int power_role);

extern int accdet_enable_power(bool enable);
extern int max20328_is_ready;
extern int mmax_is_unuseirq;
static int cable_byte_check_flag;


#endif /*end of TYPEC_ACCDET__H_*/

