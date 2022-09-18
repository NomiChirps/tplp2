#include "tplp/ui/screen_paper.h"

#include "absl/strings/str_cat.h"
#include "picolog/status.h"
#include "tplp/ui/globals.h"
#include "tplp/ui/popups.h"

namespace tplp {
namespace {

static constexpr int kUpdatePeriodMs = 100;

static lv_obj_t* state_text;
static lv_obj_t* load_text;

static lv_timer_t* update_timer = nullptr;
static void update() {
  lv_label_set_text(state_text, global_tplp_->GetPaperState().c_str());
  lv_label_set_text(load_text,
                    absl::StrCat(global_tplp_->GetLoadCellValue()).c_str());
}

static void clicked(int button_id) {
  util::Status status;
  switch (button_id) {
    case 0:  // Tension
      status = global_tplp_->TensionPaper();
      break;
    case 1:  // Feed
      status = global_tplp_->StartFeed();
      break;
    case 2:  // Stop
      status = global_tplp_->StopFeed();
      break;
    case 3:  // Release
      status = global_tplp_->ReleasePaper();
      break;
  }
  if (!status.ok()) show_popup_error(std::move(status));
}
}  // namespace

void ui_screen_paper_on_unload_cb() { lv_timer_pause(update_timer); }

lv_obj_t* ui_screen_paper_create(lv_obj_t* parent) {
  lv_obj_t* container = lv_obj_create(parent);
  lv_obj_remove_style_all(container);
  lv_obj_set_layout(container, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN_WRAP);
  lv_obj_set_size(container, LV_PCT(100), LV_PCT(100));

  state_text = lv_label_create(container);
  load_text = lv_label_create(container);

  lv_obj_t* buttons = lv_btnmatrix_create(container);
  static const char* buttons_map[] = {"Tension", "Feed", "Pause", "Release",
                                      nullptr};
  lv_btnmatrix_set_map(buttons, buttons_map);
  lv_obj_add_event_cb(
      buttons,
      [](lv_event_t* e) {
        clicked(lv_btnmatrix_get_selected_btn(lv_event_get_target(e)));
      },
      LV_EVENT_CLICKED, nullptr);

  if (!update_timer) {
    update_timer = lv_timer_create([](lv_timer_t*) { update(); },
                                   kUpdatePeriodMs, nullptr);
  }
  lv_timer_resume(update_timer);
  lv_obj_add_event_cb(
      container, [](lv_event_t*) { lv_timer_pause(update_timer); },
      LV_EVENT_DELETE, nullptr);

  return container;
}

}  // namespace tplp