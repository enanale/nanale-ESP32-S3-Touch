#ifndef WEATHER_ICONS_H
#define WEATHER_ICONS_H

#include "lvgl.h"

class WeatherIcon {
public:
  WeatherIcon();
  void create(lv_obj_t *parent);
  void setWeather(int code);
  void update();

private:
  lv_obj_t *_container;
  lv_obj_t *_main_shape;
  lv_obj_t *_extra_shape;
  int _current_code;

  void _clear_shapes();
  void _create_sun();
  void _create_clouds();
  void _create_rain();
  void _create_thunder();
};

#endif
