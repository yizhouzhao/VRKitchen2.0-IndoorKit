# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
__all__ = ["julia_modeler_style"]

from omni.ui import color as cl
from omni.ui import constant as fl
from omni.ui import url
import omni.kit.app
import omni.ui as ui
import pathlib

EXTENSION_FOLDER_PATH = pathlib.Path(
    omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
)

ATTR_LABEL_WIDTH = 150
BLOCK_HEIGHT = 22
TAIL_WIDTH = 35
WIN_WIDTH = 400
WIN_HEIGHT = 930

# Pre-defined constants. It's possible to change them at runtime.
cl.window_bg_color = cl(0.2, 0.2, 0.2, 1.0)
cl.window_title_text = cl(.9, .9, .9, .9)
cl.collapsible_header_text = cl(.8, .8, .8, .8)
cl.collapsible_header_text_hover = cl(.95, .95, .95, 1.0)
cl.main_attr_label_text = cl(.65, .65, .65, 1.0)
cl.main_attr_label_text_hover = cl(.9, .9, .9, 1.0)
cl.multifield_label_text = cl(.65, .65, .65, 1.0)
cl.combobox_label_text = cl(.65, .65, .65, 1.0)
cl.field_bg = cl(0.18, 0.18, 0.18, 1.0)
cl.field_border = cl(1.0, 1.0, 1.0, 0.2)
cl.btn_border = cl(1.0, 1.0, 1.0, 0.4)
cl.slider_fill = cl(1.0, 1.0, 1.0, 0.3)
cl.revert_arrow_enabled = cl(.25, .5, .75, 1.0)
cl.revert_arrow_disabled = cl(.75, .75, .75, 1.0)
cl.transparent = cl(0, 0, 0, 0)

fl.main_label_attr_hspacing = 10
fl.attr_label_v_spacing = 3
fl.collapsable_group_spacing = 2
fl.outer_frame_padding = 15
fl.tail_icon_width = 15
fl.border_radius = 3
fl.border_width = 1
fl.window_title_font_size = 18
fl.field_text_font_size = 14
fl.main_label_font_size = 14
fl.multi_attr_label_font_size = 14
fl.radio_group_font_size = 14
fl.collapsable_header_font_size = 13
fl.range_text_size = 10

url.closed_arrow_icon = f"{EXTENSION_FOLDER_PATH}/icons/closed.svg"
url.open_arrow_icon = f"{EXTENSION_FOLDER_PATH}/icons/opened.svg"
url.revert_arrow_icon = f"{EXTENSION_FOLDER_PATH}/icons/revert_arrow.svg"
url.checkbox_on_icon = f"{EXTENSION_FOLDER_PATH}/icons/checkbox_on.svg"
url.checkbox_off_icon = f"{EXTENSION_FOLDER_PATH}/icons/checkbox_off.svg"
url.radio_btn_on_icon = f"{EXTENSION_FOLDER_PATH}/icons/radio_btn_on.svg"
url.radio_btn_off_icon = f"{EXTENSION_FOLDER_PATH}/icons/radio_btn_off.svg"
url.diag_bg_lines_texture = f"{EXTENSION_FOLDER_PATH}/icons/diagonal_texture_screenshot.png"

####################### Indoor Kit ###########################################
# url.start_btn_on_icon = f"{EXTENSION_FOLDER_PATH}/icons/random.svg"       
url.start_btn_on_icon = f"{EXTENSION_FOLDER_PATH}/icons/toolbar_play.svg"  
url.replay_btn_on_icon = f"{EXTENSION_FOLDER_PATH}/icons/toolbar_replay.svg" 
url.stop_btn_on_icon = f"{EXTENSION_FOLDER_PATH}/icons/toolbar_stop.svg" 
url.pause_btn_on_icon = f"{EXTENSION_FOLDER_PATH}/icons/timeline_pause.svg" 
url.pencil_btn_on_icon = f"{EXTENSION_FOLDER_PATH}/icons/pencil.svg" 
url.open_folder_btn_on_icon = f"{EXTENSION_FOLDER_PATH}/icons/open_folder.svg" 

# The main style dict
julia_modeler_style = {
    "Button::tool_button": {
        "background_color": cl.field_bg,
        "margin_height": 8,
        "margin_width": 6,
        "border_color": cl.btn_border,
        "border_width": fl.border_width,
        "font_size": fl.field_text_font_size,
    },
    "CollapsableFrame::group": {
        "margin_height": fl.collapsable_group_spacing,
        "background_color": cl.transparent,
    },
    # TODO: For some reason this ColorWidget style doesn't respond much, if at all (ie, border_radius, corner_flag)
    "ColorWidget": {
        "border_radius": fl.border_radius,
        "border_color": cl(0.0, 0.0, 0.0, 0.0),
    },
    "Field": {
        "background_color": cl.field_bg,
        "border_radius": fl.border_radius,
        "border_color": cl.field_border,
        "border_width": fl.border_width,
    },
    "Field::attr_field": {
        "corner_flag": ui.CornerFlag.RIGHT,
        "font_size": 2,  # fl.field_text_font_size,  # Hack to allow for a smaller field border until field padding works
    },
    "Field::attribute_color": {
        "font_size": fl.field_text_font_size,
    },
    "Field::multi_attr_field": {
        "padding": 4,  # TODO: Hacky until we get padding fix
        "font_size": fl.field_text_font_size,
    },
    "Field::path_field": {
        "corner_flag": ui.CornerFlag.RIGHT,
        "font_size": fl.field_text_font_size,
    },
    "HeaderLine": {"color": cl(.5, .5, .5, .5)},
    "Image::collapsable_opened": {
        "color": cl.collapsible_header_text,
        "image_url": url.open_arrow_icon,
    },
    "Image::collapsable_opened:hovered": {
        "color": cl.collapsible_header_text_hover,
        "image_url": url.open_arrow_icon,
    },
    "Image::collapsable_closed": {
        "color": cl.collapsible_header_text,
        "image_url": url.closed_arrow_icon,
    },
    "Image::collapsable_closed:hovered": {
        "color": cl.collapsible_header_text_hover,
        "image_url": url.closed_arrow_icon,
    },
    "Image::radio_on": {"image_url": url.radio_btn_on_icon},
    "Image::radio_off": {"image_url": url.radio_btn_off_icon},
    "Image::revert_arrow": {
        "image_url": url.revert_arrow_icon,
        "color": cl.revert_arrow_enabled,
    },
    "Image::revert_arrow:disabled": {
        "image_url": url.revert_arrow_icon, 
        "color": cl.revert_arrow_disabled
        },
     "Image::revert_arrow_task_type": {
        "image_url": url.revert_arrow_icon,
        "color": cl.revert_arrow_enabled,
    },
    "Image::revert_arrow_task_type:disabled": {
        "image_url": url.pencil_btn_on_icon, 
        "color": cl.revert_arrow_disabled
        },
    "Image::open_folder": {
        "image_url": url.open_folder_btn_on_icon, 
        "color": cl.revert_arrow_disabled
        },

    "Image::checked": {"image_url": url.checkbox_on_icon},
    "Image::unchecked": {"image_url": url.checkbox_off_icon},
    "Image::slider_bg_texture": {
        "image_url": url.diag_bg_lines_texture,
        "border_radius": fl.border_radius,
        "corner_flag": ui.CornerFlag.LEFT,
    },
    "Label::attribute_name": {
        "alignment": ui.Alignment.RIGHT_TOP,
        "margin_height": fl.attr_label_v_spacing,
        "margin_width": fl.main_label_attr_hspacing,
        # "color": "lightsteelblue",
        "font_size": fl.main_label_font_size,
    },
    "Label::attribute_name:hovered": {"color": cl.main_attr_label_text_hover},
    "Label::collapsable_name": {"font_size": fl.collapsable_header_font_size},
    "Label::multi_attr_label": {
        "color": cl.multifield_label_text,
        "font_size": fl.multi_attr_label_font_size,
    },
    "Label::radio_group_name": {
        "font_size": fl.radio_group_font_size,
        "alignment": ui.Alignment.CENTER,
        "color": cl.main_attr_label_text,
    },
    "Label::range_text": {
        "font_size": fl.range_text_size,
    },
    "Label::window_title": {
        "font_size": fl.window_title_font_size,
        "color": cl.window_title_text,
    },
    "ScrollingFrame::window_bg": {
        "background_color": cl.window_bg_color,
        "padding": fl.outer_frame_padding,
        "border_radius": 20  # Not obvious in a window, but more visible with only a frame
    },
    "Slider::attr_slider": {
        "draw_mode": ui.SliderDrawMode.FILLED,
        "padding": 0,
        "color": cl.transparent,
        # Meant to be transparent, but completely transparent shows opaque black instead.
        "background_color": cl(0.28, 0.28, 0.28, 0.01),
        "secondary_color": cl.slider_fill,
        "border_radius": fl.border_radius,
        "corner_flag": ui.CornerFlag.LEFT,  # TODO: Not actually working yet OM-53727
    },

    # Combobox workarounds
    "Rectangle::combobox": {  # TODO: remove when ComboBox can have a border
        "background_color": cl.field_bg,
        "border_radius": fl.border_radius,
        "border_color": cl.btn_border,
        "border_width": fl.border_width,
    },
    "ComboBox::dropdown_menu": {
        "color": "lightsteelblue",  # label color
        "padding_height": 1.25,
        "margin": 2,
        "background_color": cl.field_bg,
        "border_radius": fl.border_radius,
        "font_size": fl.field_text_font_size,
        "secondary_color": cl.transparent,  # button background color
    },
    "Rectangle::combobox_icon_cover": {"background_color": cl.field_bg},


    ################## VRKitchen Indoor Kit ###############
    "Field::choose_id": { 
        "margin": 8,
    },

    "Button::record_button": {
        "background_color": cl.field_bg,
        "border_color": cl.btn_border,
        "border_width": fl.border_width,
        "border_radius": 6,
        "margin": 4,
        "corner_flag": ui.CornerFlag.ALL,
    },

    "Button::load_button": {
        "background_color": cl.field_bg,
        "border_color": cl.btn_border,
        "border_width": fl.border_width,
        "border_radius": 10,
        "margin": 4,
        "corner_flag": ui.CornerFlag.ALL,
    },


    "Button::add_button": {
        "background_color": cl.field_bg,
        "border_color": cl.btn_border,
        "border_width": fl.border_width,
        "border_radius": 2,
        "margin": 8,
        "corner_flag": ui.CornerFlag.ALL,
    },

    "Button::control_button": {
        "background_color": cl.field_bg,
        "border_color": cl.btn_border,
        "border_width": fl.border_width,
        "border_radius": 4,
        "margin": 2,
        "corner_flag": ui.CornerFlag.ALL,
    },

    "Button::control_button_disabled": {
        "background_color": cl(0.1, 0.7, 0.3, 0.4),
        "border_color": cl.btn_border,
        "border_width": fl.border_width,
        "border_radius": 4,
        "margin": 2,
        "corner_flag": ui.CornerFlag.ALL,
    },

    "Button::control_button_pressed1": {
        "background_color": cl( 0.7, 0.1, 0.3, 0.3),
        "border_color": cl.btn_border,
        "border_width": fl.border_width,
        "border_radius": 4,
        "margin": 2,
        "corner_flag": ui.CornerFlag.ALL,
    },

    "Button::control_button_pressed2": {
        "background_color": cl(0.1, 0.3, 0.7, 0.3),
        "border_color": cl.btn_border,
        "border_width": fl.border_width,
        "border_radius": 4,
        "margin": 2,
        "corner_flag": ui.CornerFlag.ALL,
    },

    "Button::control_button_pressed3": {
        "background_color": cl(0.7, 0.3, 0.7, 0.3),
        "border_color": cl.btn_border,
        "border_width": fl.border_width,
        "border_radius": 4,
        "margin": 2,
        "corner_flag": ui.CornerFlag.ALL,
    },


    "Image::start_on": {
        "image_url": url.start_btn_on_icon,
        },

    "Image::replay_on": {
        "image_url": url.replay_btn_on_icon,
        },
    
    "Image::stop_on": {
        "image_url": url.stop_btn_on_icon,
        },
    
    "Image::pause_on": {
        "image_url": url.pause_btn_on_icon,
        },
    # "Image::radio_off": {"image_url": url.radio_btn_off_icon},
}
