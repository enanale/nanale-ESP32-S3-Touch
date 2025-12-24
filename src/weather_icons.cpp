#include "weather_icons.h"

WeatherIcon::WeatherIcon()
    : _container(NULL), _main_shape(NULL), _extra_shape(NULL),
      _current_code(-1) {}

void WeatherIcon::create(lv_obj_t *parent) {
  _container = lv_obj_create(parent);
  lv_obj_set_size(_container, 100, 100);
  lv_obj_set_style_bg_opa(_container, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(_container, 0, 0);
  lv_obj_set_style_pad_all(_container, 0, 0);
  lv_obj_clear_flag(_container, LV_OBJ_FLAG_SCROLLABLE);
}

void WeatherIcon::_clear_shapes() {
  if (_main_shape) {
    lv_obj_delete(_main_shape);
    _main_shape = NULL;
  }
  if (_extra_shape) {
    lv_obj_delete(_extra_shape);
    _extra_shape = NULL;
  }
}

void WeatherIcon::setWeather(int code) {
  if (_current_code == code)
    return;
  _current_code = code;
  _clear_shapes();

  if (code == 0 || code == 1) { // Clear / Mainly Clear
    _create_sun();
  } else if (code == 2 || code == 3) { // Partly Cloudy / Overcast
    _create_clouds();
  } else if (code >= 51 && code <= 67) { // Rain / Drizzle
    _create_rain();
  } else if (code >= 95) { // Thunderstorm
    _create_thunder();
  } else {
    _create_sun(); // Default
  }
}

void WeatherIcon::_create_sun() {
  _main_shape = lv_obj_create(_container);
  lv_obj_set_size(_main_shape, 40, 40);
  lv_obj_align(_main_shape, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(_main_shape, lv_palette_main(LV_PALETTE_YELLOW), 0);
  lv_obj_set_style_radius(_main_shape, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(_main_shape, 0, 0);

  // Subtle breathing animation
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, _main_shape);
  lv_anim_set_values(&a, 40, 45);
  lv_anim_set_time(&a, 2000);
  lv_anim_set_playback_time(&a, 2000);
  lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
  lv_anim_set_exec_cb(
      &a, [](void *var, int32_t v) { lv_obj_set_size((lv_obj_t *)var, v, v); });
  lv_anim_start(&a);
}

void WeatherIcon::_create_clouds() {
  _main_shape = lv_obj_create(_container);
  lv_obj_set_size(_main_shape, 50, 30);
  lv_obj_align(_main_shape, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(_main_shape, lv_color_white(), 0);
  lv_obj_set_style_radius(_main_shape, 15, 0);
  lv_obj_set_style_border_width(_main_shape, 0, 0);

  // Drifting animation
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, _main_shape);
  lv_anim_set_values(&a, -10, 10);
  lv_anim_set_time(&a, 4000);
  lv_anim_set_playback_time(&a, 4000);
  lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
  lv_anim_set_exec_cb(
      &a, [](void *var, int32_t v) { lv_obj_set_x((lv_obj_t *)var, v); });
  lv_anim_start(&a);
}

void WeatherIcon::_create_rain() {
  _create_clouds(); // Rain comes from clouds

  _extra_shape = lv_obj_create(_container);
  lv_obj_set_size(_extra_shape, 4, 15);
  lv_obj_align(_extra_shape, LV_ALIGN_CENTER, 0, 20);
  lv_obj_set_style_bg_color(_extra_shape, lv_palette_main(LV_PALETTE_BLUE), 0);
  lv_obj_set_style_border_width(_extra_shape, 0, 0);

  // Falling animation
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, _extra_shape);
  lv_anim_set_values(&a, 10, 40);
  lv_anim_set_time(&a, 800);
  lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
  lv_anim_set_exec_cb(&a, [](void *var, int32_t v) {
    lv_obj_set_y((lv_obj_t *)var, v);
    lv_obj_set_style_opa((lv_obj_t *)var, 255 - (v * 4), 0);
  });
  lv_anim_start(&a);
}

void WeatherIcon::_create_thunder() {
  _create_clouds();

  _extra_shape = lv_obj_create(_container);
  lv_obj_set_size(_extra_shape, 10, 20);
  lv_obj_align(_extra_shape, LV_ALIGN_CENTER, 0, 25);
  lv_obj_set_style_bg_color(_extra_shape, lv_palette_main(LV_PALETTE_YELLOW),
                            0);
  lv_obj_set_style_border_width(_extra_shape, 0, 0);

  // Flickering animation
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, _extra_shape);
  lv_anim_set_values(&a, 0, 255);
  lv_anim_set_time(&a, 100);
  lv_anim_set_playback_time(&a, 100);
  lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
  lv_anim_set_exec_cb(&a, [](void *var, int32_t v) {
    if (v > 200)
      lv_obj_set_style_opa((lv_obj_t *)var, 255, 0);
    else
      lv_obj_set_style_opa((lv_obj_t *)var, 0, 0);
  });
  lv_anim_start(&a);
}

void WeatherIcon::update() {
  // Basic maintenance if needed
}
