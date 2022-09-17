#include "tplp/ui/screen_params.h"

#include <algorithm>

#include "absl/strings/str_cat.h"
#include "lvgl/lvgl.h"
#include "picolog/picolog.h"
#include "tplp/config/params.h"
#include "tplp/ui/globals.h"
#include "tplp/ui/popups.h"

using tplp::config::AllParameters;
using tplp::config::ParameterBase;

// Only 4 fit above the keyboard for now.
// TODO: shrink row height or reduce font size or scroll to active param on edit
static constexpr int kParamsPerPage = 4;
static int page_number = 0;
struct row_t {
  // fixed
  lv_obj_t* label;
  lv_obj_t* textarea;
  lv_obj_t* edit_button;

  // mutable
  ParameterBase* param;
};
static row_t rows[kParamsPerPage];

static constexpr int kRowHeight = 30;
// TODO: we're gonna need a smaller font...
static constexpr int kMaxValueLength = 32;

// add 1 for the trailing null
static char tmp_value_buf[kMaxValueLength + 1];

static lv_obj_t* prev_page_button;
static lv_obj_t* next_page_button;

static void save_clicked(lv_event_t* e) {
  util::Status status = global_tplp_->SaveAllParameters();
  if (status.ok()) {
    // TODO: show a brief confirmation
    // instead for now we're using this modal
    static const char* btn_txts[] = {nullptr};
    lv_obj_t* msgbox = lv_msgbox_create(
        nullptr, "", LV_SYMBOL_OK " Parameters Saved!", btn_txts,
        /*add_close_btn=*/true);
    lv_obj_center(msgbox);
  } else {
    show_popup_error(std::move(status));
  }
}

static void reset_value(lv_obj_t* textarea, lv_obj_t* edit_button) {
  auto* userdata = static_cast<row_t*>(lv_obj_get_user_data(textarea));
  CHECK_NOTNULL(userdata);
  auto n = userdata->param->Serialize(tmp_value_buf, sizeof(tmp_value_buf));
  if (!n.ok()) {
    tmp_value_buf[n.status().ToString().copy(tmp_value_buf,
                                             sizeof(tmp_value_buf) - 1)] = '\0';
    lv_obj_add_state(edit_button, LV_STATE_DISABLED);
  } else {
    if (n.value() == kMaxValueLength) {
      strncpy(tmp_value_buf, absl::StrCat("<", n.value(), " bytes>").c_str(),
              sizeof(tmp_value_buf) - 1);
      lv_obj_add_state(edit_button, LV_STATE_DISABLED);
    } else {
      lv_obj_clear_state(edit_button, LV_STATE_DISABLED);
    }
    tmp_value_buf[n.value()] = '\0';
  }
  lv_textarea_set_text(textarea, tmp_value_buf);
}

static void go_to_page(int page_number) {
  ::page_number = page_number;
  int begin_param_index = kParamsPerPage * page_number;
  int end_param_index = kParamsPerPage * (page_number + 1);
  int param_index = -1;
  int row_index = -1;
  bool more_params = false;
  for (ParameterBase* param : AllParameters()) {
    ++param_index;
    if (param_index < begin_param_index) {
      continue;
    }
    if (param_index >= end_param_index) {
      more_params = true;
      break;
    }
    row_index = param_index % kParamsPerPage;
    rows[row_index].param = param;
    lv_label_set_text(rows[row_index].label, param->name());
    reset_value(rows[row_index].textarea, rows[row_index].edit_button);
    lv_obj_clear_flag(rows[row_index].label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(rows[row_index].textarea, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(rows[row_index].edit_button, LV_OBJ_FLAG_HIDDEN);
  }
  // disable any remaining empty rows
  for (++row_index; row_index < kParamsPerPage; ++row_index) {
    lv_obj_add_flag(rows[row_index].label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(rows[row_index].textarea, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(rows[row_index].edit_button, LV_OBJ_FLAG_HIDDEN);
  }
  // enable appropriate next/prev buttons
  if (page_number > 0) {
    lv_obj_clear_state(prev_page_button, LV_STATE_DISABLED);
  } else {
    lv_obj_add_state(prev_page_button, LV_STATE_DISABLED);
  }
  if (more_params) {
    lv_obj_clear_state(next_page_button, LV_STATE_DISABLED);
  } else {
    lv_obj_add_state(next_page_button, LV_STATE_DISABLED);
  }
}

static void prev_page_clicked(lv_event_t* e) { go_to_page(page_number - 1); }
static void next_page_clicked(lv_event_t* e) { go_to_page(page_number + 1); }

static void edit_clicked(lv_event_t* e) {
  lv_obj_t* textarea = static_cast<lv_obj_t*>(lv_event_get_user_data(e));
  CHECK_NOTNULL(textarea);
  show_popup_numpad(textarea);
}

static void textarea_cb(lv_event_t* e) {
  bool ok;
  switch (lv_event_get_code(e)) {
    case LV_EVENT_READY:
      ok = true;
      break;
    case LV_EVENT_CANCEL:
      ok = false;
      break;
    default:
      // uninteresting event
      return;
  }

  lv_obj_t* textarea = static_cast<lv_obj_t*>(lv_event_get_user_data(e));
  CHECK_NOTNULL(textarea);
  auto userdata = static_cast<row_t*>(lv_obj_get_user_data(textarea));
  CHECK_NOTNULL(userdata);

  if (ok) {
    util::Status status =
        userdata->param->Parse(lv_textarea_get_text(textarea));
    if (status.ok()) {
      // TODO: show a brief confirmation
    } else {
      show_popup_error(std::move(status));
    }
  }
  // Regardless of success or failure, refresh the textarea
  // with the param's value. On success the purpose of this
  // is to canonicalize the string representation.
  reset_value(textarea, userdata->edit_button);
}

lv_obj_t* ui_screen_params_create(lv_obj_t* parent) {
  lv_obj_t* grid = lv_obj_create(parent);
  lv_obj_remove_style_all(grid);
  lv_obj_set_style_pad_ver(grid, 10, 0);
  lv_obj_set_style_pad_gap(grid, 10, 0);
  lv_obj_add_flag(grid, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

  // Columns: name, textarea, edit button
  // Rows: params..., global buttons
  static lv_coord_t col_dsc[] = {LV_GRID_CONTENT, LV_GRID_FR(1),
                                 LV_GRID_CONTENT, LV_GRID_TEMPLATE_LAST};
  // +1 for the bottom buttons, +1 for the array sentinel
  static lv_coord_t row_dsc[tplp::config::kMaxNumParams + 2];
  for (int row_index = 0; row_index < kParamsPerPage; ++row_index) {
    row_dsc[row_index] = kRowHeight;
  }
  row_dsc[kParamsPerPage + 0] = LV_GRID_CONTENT;
  row_dsc[kParamsPerPage + 1] = LV_GRID_TEMPLATE_LAST;
  lv_obj_set_style_grid_row_dsc_array(grid, row_dsc, 0);
  lv_obj_set_style_grid_column_dsc_array(grid, col_dsc, 0);
  lv_obj_set_size(grid, LV_PCT(100), LV_SIZE_CONTENT);
  lv_obj_set_layout(grid, LV_LAYOUT_GRID);

  for (int i = 0; i < kParamsPerPage; ++i) {
    lv_obj_t* label = lv_label_create(grid);
    lv_obj_set_grid_cell(label, LV_GRID_ALIGN_START, 0, 1, LV_GRID_ALIGN_CENTER,
                         i, 1);

    lv_obj_t* textarea = lv_textarea_create(grid);
    lv_obj_remove_style_all(textarea);
    lv_obj_set_style_border_side(textarea, LV_BORDER_SIDE_LEFT,
                                 LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_obj_set_style_border_width(textarea, 1,
                                  LV_PART_CURSOR | LV_STATE_FOCUSED);
    // leave room for the cursor on the left side
    lv_obj_set_style_pad_left(textarea, 1, 0);
    lv_obj_set_style_border_color(textarea, lv_color_black(),
                                  LV_PART_CURSOR | LV_STATE_FOCUSED);
    lv_obj_set_style_anim_time(textarea, 400, LV_PART_CURSOR);
    lv_obj_clear_flag(textarea, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_textarea_set_one_line(textarea, true);
    lv_textarea_set_max_length(textarea, kMaxValueLength);

    lv_obj_add_event_cb(textarea, textarea_cb, LV_EVENT_ALL,
                        /*userdata=*/textarea);
    lv_obj_set_user_data(textarea, &rows[i]);
    lv_obj_set_grid_cell(textarea, LV_GRID_ALIGN_STRETCH, 1, 1,
                         LV_GRID_ALIGN_CENTER, i, 1);

    lv_obj_t* edit_button = lv_btn_create(grid);
    lv_obj_t* edit_button_label = lv_label_create(edit_button);
    lv_label_set_text(edit_button_label, LV_SYMBOL_EDIT);
    lv_obj_add_event_cb(edit_button, edit_clicked, LV_EVENT_CLICKED, textarea);
    lv_obj_set_grid_cell(edit_button, LV_GRID_ALIGN_START, 2, 1,
                         LV_GRID_ALIGN_CENTER, i, 1);

    rows[i].label = label;
    rows[i].textarea = textarea;
    rows[i].edit_button = edit_button;
  }

  lv_obj_t* nextprev_container = lv_obj_create(grid);
  lv_obj_remove_style_all(nextprev_container);
  lv_obj_add_flag(nextprev_container, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
  lv_obj_set_style_pad_gap(nextprev_container, 10, 0);
  lv_obj_set_layout(nextprev_container, LV_LAYOUT_FLEX);
  lv_obj_set_size(nextprev_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);

  prev_page_button = lv_btn_create(nextprev_container);
  lv_obj_t* prev_page_button_label = lv_label_create(prev_page_button);
  lv_label_set_text(prev_page_button_label, LV_SYMBOL_PREV);
  lv_obj_add_event_cb(prev_page_button, prev_page_clicked, LV_EVENT_CLICKED,
                      nullptr);
  next_page_button = lv_btn_create(nextprev_container);
  lv_obj_t* next_page_button_label = lv_label_create(next_page_button);
  lv_label_set_text(next_page_button_label, LV_SYMBOL_NEXT);
  lv_obj_add_event_cb(next_page_button, next_page_clicked, LV_EVENT_CLICKED,
                      nullptr);

  lv_obj_set_grid_cell(nextprev_container, LV_GRID_ALIGN_START, /*col_pos=*/0,
                       /*col_span=*/3, LV_GRID_ALIGN_START,
                       /*row_pos=*/kParamsPerPage, 1);

  // TODO: factory reset button? or should that go on a separate page?

  lv_obj_t* save_button = lv_btn_create(grid);
  lv_obj_t* img = lv_img_create(save_button);
  lv_img_set_src(img, LV_SYMBOL_SAVE);
  lv_obj_add_event_cb(save_button, save_clicked, LV_EVENT_CLICKED, NULL);
  lv_obj_set_grid_cell(save_button, LV_GRID_ALIGN_END, /*col_pos=*/0,
                       /*col_span=*/3, LV_GRID_ALIGN_START,
                       /*row_pos=*/kParamsPerPage, 1);

  // initialize from the page we were on, if coming back to this screen.
  go_to_page(page_number);

  return grid;
}
