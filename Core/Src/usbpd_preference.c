/*
 * usbpd_preference.c
 *
 *  Created on: Nov 30, 2020
 *      Author: dave
 */

#include "usbpd_preference.h"

void usbpd_pref_init(usbpd_user_config_t *config)
{
  config->min_v_millis = 0;
  config->max_v_millis = 0;
  config->min_c_millis = 0;
  config->max_c_millis = 0;
  config->prefer_max_v = false;
  config->prefer_max_p = false;
}

void usbpd_pref_set(usbpd_user_config_t *config, uint32_t min_v_millis, uint32_t max_v_millis, uint32_t min_c_millis, uint32_t max_c_millis)
{
  config->min_v_millis = min_v_millis;
  config->max_v_millis = max_v_millis;
  config->min_c_millis = min_c_millis;
  config->max_c_millis = max_c_millis;
  config->prefer_max_v = false;
  config->prefer_max_p = false;

}

void usbpd_pref_prefer_max_v(usbpd_user_config_t *config, bool enable)
{
  config->prefer_max_v = enable;
  config->prefer_max_p = !enable;
}
void usbpd_pref_prefer_max_p(usbpd_user_config_t *config, bool enable)
{
  config->prefer_max_p = enable;
  config->prefer_max_v = !enable;
}

uint32_t usbpd_pref_get_min_v_millis(usbpd_user_config_t *config)
{
  return config->min_v_millis;
}
uint32_t usbpd_pref_get_max_v_millis(usbpd_user_config_t *config)
{
  return config->max_v_millis;
}
uint32_t usbpd_pref_get_min_c_millis(usbpd_user_config_t *config)
{
  return config->min_c_millis;
}
uint32_t usbpd_pref_get_max_c_millis(usbpd_user_config_t *config)
{
  return config->max_c_millis;
}
bool usbpd_pref_is_prefer_max_v(usbpd_user_config_t *config)
{
  return config->prefer_max_v;
}
bool usbpd_pref_is_prefer_max_p(usbpd_user_config_t *config)
{
  return config->prefer_max_p;
}


