#include "ui_manager.h"
#include "audio_manager.h"
#include "user_config.h"
#include <Arduino.h>

UIManager::UIManager(WeatherManager &wm, BatteryManager &bm, TimeManager &tm,
                     PowerManager &pm, WeatherIcon &wi)
    : _weatherMgr(wm), _batMgr(bm), _timeMgr(tm), _pwrMgr(pm), _weatherIcon(wi),
      _currentCityIndex(0), _ui_status_bar(NULL), _ui_status_label_time(NULL),
      _ui_status_label_bat(NULL), _ui_content_area(NULL), _ui_city_label(NULL),
      _ui_temp_label(NULL), _ui_cond_label(NULL) {}

void UIManager::setCurrentCityIndex(int index) { _currentCityIndex = index; }

void UIManager::build() {
  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

  // 1. Status Bar (Bottom, Purple)
  _ui_status_bar = lv_obj_create(scr);
  lv_obj_set_size(_ui_status_bar, LV_PCT(100), 30);
  lv_obj_set_align(_ui_status_bar, LV_ALIGN_BOTTOM_MID);
  lv_obj_set_style_bg_color(_ui_status_bar,
                            lv_palette_main(LV_PALETTE_DEEP_PURPLE), 0);
  lv_obj_set_style_radius(_ui_status_bar, 0, 0);
  lv_obj_set_style_border_width(_ui_status_bar, 0, 0);

  _ui_status_label_time = lv_label_create(_ui_status_bar);
  lv_label_set_text(_ui_status_label_time, "--:--");
  lv_obj_set_style_text_color(_ui_status_label_time, lv_color_white(), 0);
  lv_obj_align(_ui_status_label_time, LV_ALIGN_LEFT_MID, 5, 0);

  _ui_status_label_bat = lv_label_create(_ui_status_bar);
  lv_label_set_text(_ui_status_label_bat, "Bat: --%");
  lv_obj_set_style_text_color(_ui_status_label_bat, lv_color_white(), 0);
  lv_obj_align(_ui_status_label_bat, LV_ALIGN_RIGHT_MID, -5, 0);

  // 2. Main Content Area (Weather - Left Side)
  _ui_content_area = lv_obj_create(scr);
  lv_obj_set_size(_ui_content_area, 640 - 60, 172 - 30);
  lv_obj_align(_ui_content_area, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_bg_color(_ui_content_area, lv_color_black(), 0);
  lv_obj_set_style_border_width(_ui_content_area, 0, 0);
  lv_obj_set_style_pad_all(_ui_content_area, 0, 0);

  // City Label
  _ui_city_label = lv_label_create(_ui_content_area);
  lv_obj_set_style_text_font(_ui_city_label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(_ui_city_label, lv_color_white(), 0);
  lv_obj_align(_ui_city_label, LV_ALIGN_TOP_MID, 0, 10);
  lv_label_set_text(_ui_city_label, "City");

  // Weather Icon
  _weatherIcon.create(_ui_content_area);
  lv_obj_set_align(lv_obj_get_child(_ui_content_area, -1), LV_ALIGN_CENTER);
  lv_obj_set_x(lv_obj_get_child(_ui_content_area, -1), -60);

  // Temp Label
  _ui_temp_label = lv_label_create(_ui_content_area);
  lv_obj_set_style_text_font(_ui_temp_label, &lv_font_montserrat_40, 0);
  lv_obj_set_style_text_color(_ui_temp_label,
                              lv_palette_main(LV_PALETTE_YELLOW), 0);
  lv_obj_align(_ui_temp_label, LV_ALIGN_CENTER, 40, 0);
  lv_label_set_text(_ui_temp_label, "--°F");

  // Condition Label
  _ui_cond_label = lv_label_create(_ui_content_area);
  lv_obj_set_style_text_font(_ui_cond_label, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(_ui_cond_label,
                              lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
  lv_obj_align(_ui_cond_label, LV_ALIGN_BOTTOM_MID, 0, -5);
  lv_label_set_text(_ui_cond_label, "---");
  lv_obj_set_size(_ui_content_area, 640 - 100, 172 - 30);
  lv_obj_align(_ui_content_area, LV_ALIGN_TOP_LEFT, 0, 0);

  // 2. Right Side: Voice Memo Area (100px wide)
  _ui_memo_cont = lv_obj_create(scr);
  lv_obj_set_size(_ui_memo_cont, 100, 172 - 30);
  lv_obj_align(_ui_memo_cont, LV_ALIGN_TOP_RIGHT, 0, 0);
  lv_obj_set_style_bg_color(_ui_memo_cont, lv_palette_main(LV_PALETTE_GREY), 0);
  lv_obj_set_style_bg_opa(_ui_memo_cont, 40, 0);
  lv_obj_set_style_border_width(_ui_memo_cont, 0, 0);
  lv_obj_set_style_pad_all(_ui_memo_cont, 5, 0);
  lv_obj_set_flex_flow(_ui_memo_cont, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(_ui_memo_cont, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_gap(_ui_memo_cont, 10, 0);
  lv_obj_set_style_pad_all(_ui_memo_cont, 0, 0);

  _ui_record_btn = lv_button_create(_ui_memo_cont);
  lv_obj_set_size(_ui_record_btn, 70, 45); // Wider/shorter to fit better
  lv_obj_set_style_bg_color(_ui_record_btn, lv_palette_main(LV_PALETTE_RED), 0);
  lv_obj_set_style_radius(_ui_record_btn, 10, 0);
  lv_obj_t *rec_label = lv_label_create(_ui_record_btn);
  lv_label_set_text(rec_label, LV_SYMBOL_VIDEO);
  lv_obj_center(rec_label);

  _ui_play_btn = lv_button_create(_ui_memo_cont);
  lv_obj_set_size(_ui_play_btn, 70, 45);
  lv_obj_set_style_bg_color(_ui_play_btn, lv_palette_main(LV_PALETTE_BLUE), 0);
  lv_obj_set_style_radius(_ui_play_btn, 10, 0);
  lv_obj_t *play_label = lv_label_create(_ui_play_btn);
  lv_label_set_text(play_label, LV_SYMBOL_PLAY);
  lv_obj_center(play_label);

  _ui_memo_label =
      lv_label_create(scr); // Move label to a better place or remove?
  lv_obj_set_size(_ui_memo_label, 120, 20);
  lv_label_set_text(_ui_memo_label, "");
  lv_obj_set_style_text_font(_ui_memo_label, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(_ui_memo_label, lv_color_white(), 0);
  lv_obj_align(_ui_memo_label, LV_ALIGN_TOP_RIGHT, -65, 5);

  lv_obj_add_event_cb(_ui_record_btn, voice_memo_event_cb, LV_EVENT_CLICKED,
                      this);
  lv_obj_add_event_cb(_ui_play_btn, voice_memo_event_cb, LV_EVENT_CLICKED,
                      this);

  // 4. Gestures
  lv_obj_add_event_cb(_ui_content_area, gesture_event_cb, LV_EVENT_GESTURE,
                      this);

  updateWeather();
  updateVoiceMemoStatus();
}

void UIManager::updateStatusBar() {
  if (!_ui_status_label_time || !_ui_status_label_bat)
    return;

  lv_label_set_text(_ui_status_label_time, _timeMgr.getTimeString().c_str());

  float voltage = _batMgr.getVoltage();
  int pct = _batMgr.getPercentage();

  const char *icon = LV_SYMBOL_BATTERY_FULL;
  if (pct < 10)
    icon = LV_SYMBOL_BATTERY_EMPTY;
  else if (pct < 30)
    icon = LV_SYMBOL_BATTERY_1;
  else if (pct < 60)
    icon = LV_SYMBOL_BATTERY_2;
  else if (pct < 90)
    icon = LV_SYMBOL_BATTERY_3;

  char buf[64];
  snprintf(buf, sizeof(buf), "%s %d%% (%.2fV)", icon, pct, voltage);
  lv_label_set_text(_ui_status_label_bat, buf);

  // High frequency update for voice memo animations if recording
  updateVoiceMemoStatus();
}

void UIManager::updateWeather() {
  if (!_ui_city_label || !_ui_temp_label)
    return;

  WeatherData data = _weatherMgr.getWeather(_currentCityIndex);
  lv_label_set_text(_ui_city_label, data.city.c_str());

  char tempBuf[16];
  snprintf(tempBuf, sizeof(tempBuf), "%.1f°F", data.tempF);
  lv_label_set_text(_ui_temp_label, tempBuf);

  lv_label_set_text(_ui_cond_label, data.condition.c_str());
  _weatherIcon.setWeather(data.weather_code);
}

void UIManager::updateVoiceMemoStatus() {
  extern AudioManager audioMgr;
  if (audioMgr.isRecording()) {
    lv_label_set_text(_ui_memo_label, "#ff0000 REC#");
    lv_label_set_recolor(_ui_memo_label, true);
    lv_obj_set_style_bg_color(_ui_record_btn, lv_palette_main(LV_PALETTE_GREY),
                              0);
  } else if (audioMgr.isMemoPlaying()) {
    lv_label_set_text(_ui_memo_label, "#0000ff PLAY#");
    lv_label_set_recolor(_ui_memo_label, true);
  } else {
    if (audioMgr.hasRecording()) {
      lv_label_set_text(_ui_memo_label, "READY");
      lv_obj_remove_state(_ui_play_btn, LV_STATE_DISABLED);
    } else {
      lv_label_set_text(_ui_memo_label, "");
      lv_obj_add_state(_ui_play_btn, LV_STATE_DISABLED);
    }
    lv_label_set_recolor(_ui_memo_label, false);
    lv_obj_set_style_bg_color(_ui_record_btn, lv_palette_main(LV_PALETTE_RED),
                              0);
  }
}

void UIManager::gesture_event_cb(lv_event_t *e) {
  UIManager *ui = (UIManager *)lv_event_get_user_data(e);
  lv_indev_t *indev = lv_indev_active();
  if (!indev)
    return;

  lv_dir_t dir = lv_indev_get_gesture_dir(indev);
  if (dir == LV_DIR_LEFT) {
    ui->_currentCityIndex++;
    if (ui->_currentCityIndex >= 3)
      ui->_currentCityIndex = 0;
    ui->updateWeather();
    ui->_pwrMgr.resetTimer();
    Serial.println("[UI] Swipe LEFT -> Next City");
  } else if (dir == LV_DIR_RIGHT) {
    ui->_currentCityIndex--;
    if (ui->_currentCityIndex < 0)
      ui->_currentCityIndex = 2;
    ui->updateWeather();
    ui->_pwrMgr.resetTimer();
    Serial.println("[UI] Swipe RIGHT -> Prev City");
  }
}

void UIManager::voice_memo_event_cb(lv_event_t *e) {
  lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
  UIManager *ui = (UIManager *)lv_event_get_user_data(e);

  lv_event_code_t code = lv_event_get_code(e);
  Serial.printf("[UI] Memo Event: %d on %p\n", (int)code, obj);

  extern AudioManager audioMgr;
  audioMgr.playClick(); // Haptic feedback

  if (obj == ui->_ui_record_btn) {
    if (audioMgr.isRecording()) {
      audioMgr.stopRecording();
    } else {
      audioMgr.startRecording();
    }
  } else if (obj == ui->_ui_play_btn) {
    audioMgr.startMemoPlayback();
  }
  ui->_pwrMgr.resetTimer();
}
