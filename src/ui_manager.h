#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include "battery_manager.h"
#include "lvgl.h"
#include "power_manager.h"
#include "time_manager.h"
#include "weather_icons.h"
#include "weather_manager.h"

class UIManager {
public:
  UIManager(WeatherManager &wm, BatteryManager &bm, TimeManager &tm,
            PowerManager &pm, WeatherIcon &wi);

  void build();
  void updateStatusBar();
  void updateWeather();
  void updateVoiceMemoStatus();

  int getCurrentCityIndex() const { return _currentCityIndex; }
  void setCurrentCityIndex(int index);

private:
  static void gesture_event_cb(lv_event_t *e);
  static void voice_memo_event_cb(lv_event_t *e);

  WeatherManager &_weatherMgr;
  BatteryManager &_batMgr;
  TimeManager &_timeMgr;
  PowerManager &_pwrMgr;
  WeatherIcon &_weatherIcon;

  int _currentCityIndex;

  lv_obj_t *_ui_status_bar;
  lv_obj_t *_ui_status_label_time;
  lv_obj_t *_ui_status_label_bat;

  lv_obj_t *_ui_content_area;
  lv_obj_t *_ui_city_label;
  lv_obj_t *_ui_temp_label;
  lv_obj_t *_ui_cond_label;

  // Voice Memo UI
  lv_obj_t *_ui_memo_cont;
  lv_obj_t *_ui_record_btn;
  lv_obj_t *_ui_play_btn;
  lv_obj_t *_ui_memo_label;
};

#endif
