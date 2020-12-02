/*
 * usbpd_preference.h
 *
 *  Created on: Nov 30, 2020
 *      Author: dave
 */

#ifndef INC_USBPD_PREFERENCE_H_
#define INC_USBPD_PREFERENCE_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
  //XXX: Compare the following to the supplied PDOs the user setup within the CUBE
  uint32_t min_v_millis;
  uint32_t max_v_millis;  //even if the user would like a higher max voltage, if it is not in the range of snk should I allow it??
  //XXX: Add preferences for min/max voltage
  //XXX: Add preferences for min/max current
  uint32_t min_c_millis;
  uint32_t max_c_millis;
  //XXX: Add a prefernce for max voltage/max power
  bool prefer_max_v;
  bool prefer_max_p;
  //uint8_t snk_pdo_index_preference;
} usbpd_user_config_t;

void usbpd_pref_init(usbpd_user_config_t *config);

void usbpd_pref_set(usbpd_user_config_t *config, uint32_t min_v_millis, uint32_t max_v_millis, uint32_t min_c_millis, uint32_t max_c_millis);

void usbpd_pref_prefer_max_v(usbpd_user_config_t *config, bool enable);
void usbpd_pref_prefer_max_p(usbpd_user_config_t *config, bool enable);

uint32_t usbpd_pref_get_min_v_millis(usbpd_user_config_t *config);
uint32_t usbpd_pref_get_max_v_millis(usbpd_user_config_t *config);
uint32_t usbpd_pref_get_min_c_millis(usbpd_user_config_t *config);
uint32_t usbpd_pref_get_max_c_millis(usbpd_user_config_t *config);
bool usbpd_pref_is_prefer_max_v(usbpd_user_config_t *config);
bool usbpd_pref_is_prefer_max_p(usbpd_user_config_t *config);




#endif /* INC_USBPD_PREFERENCE_H_ */
