# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
__all__ = ["CustomSliderWidget"]

from typing import Optional

import omni.ui as ui
from omni.ui import color as cl
from omni.ui import constant as fl

from .custom_base_widget import CustomBaseWidget

NUM_FIELD_WIDTH = 50
SLIDER_WIDTH = ui.Percent(100)
FIELD_HEIGHT = 22  # TODO: Once Field padding is fixed, this should be 18
SPACING = 4
TEXTURE_NAME = "slider_bg_texture"


class CustomSliderWidget(CustomBaseWidget):
    """A compound widget for scalar slider input, which contains a
    Slider and a Field with text input next to it.
    """

    def __init__(self,
                 model: ui.AbstractItemModel = None,
                 num_type: str = "float",
                 min=0.0,
                 max=1.0,
                 default_val=0.0,
                 display_range: bool = False,
                 **kwargs):
        self.__slider: Optional[ui.AbstractSlider] = None
        self.__numberfield: Optional[ui.AbstractField] = None
        self.__min = min
        self.__max = max
        self.__default_val = default_val
        self.__num_type = num_type
        self.__display_range = display_range

        # Call at the end, rather than start, so build_fn runs after all the init stuff
        CustomBaseWidget.__init__(self, model=model, **kwargs)

    def destroy(self):
        CustomBaseWidget.destroy()
        self.__slider = None
        self.__numberfield = None

    @property
    def model(self) -> Optional[ui.AbstractItemModel]:
        """The widget's model"""
        if self.__slider:
            return self.__slider.model

    @model.setter
    def model(self, value: ui.AbstractItemModel):
        """The widget's model"""
        self.__slider.model = value
        self.__numberfield.model = value

    def _on_value_changed(self, *args):
        """Set revert_img to correct state."""
        if self.__num_type == "float":
            index = self.model.as_float
        else:
            index = self.model.as_int
        self.revert_img.enabled = self.__default_val != index

    def _restore_default(self):
        """Restore the default value."""
        if self.revert_img.enabled:
            self.model.set_value(self.__default_val)
            self.revert_img.enabled = False

    def _build_display_range(self):
        """Builds just the tiny text range under the slider."""
        with ui.HStack():
            ui.Label(str(self.__min), alignment=ui.Alignment.LEFT, name="range_text")
            if self.__min < 0 and self.__max > 0:
                # Add middle value (always 0), but it may or may not be centered,
                # depending on the min/max values.
                total_range = self.__max - self.__min
                # subtract 25% to account for end number widths
                left = 100 * abs(0 - self.__min) / total_range - 25
                right = 100 * abs(self.__max - 0) / total_range - 25
                ui.Spacer(width=ui.Percent(left))
                ui.Label("0", alignment=ui.Alignment.CENTER, name="range_text")
                ui.Spacer(width=ui.Percent(right))
            else:
                ui.Spacer()
            ui.Label(str(self.__max), alignment=ui.Alignment.RIGHT, name="range_text")
        ui.Spacer(height=.75)

    def _build_body(self):
        """Main meat of the widget.  Draw the Slider, display range text, Field,
        and set up callbacks to keep them updated.
        """
        with ui.HStack(spacing=0):
            # the user provided a list of default values
            with ui.VStack(spacing=3, width=ui.Fraction(3)):
                with ui.ZStack():
                    # Put texture image here, with rounded corners, then make slider
                    # bg be fully transparent, and fg be gray and partially transparent
                    with ui.Frame(width=SLIDER_WIDTH, height=FIELD_HEIGHT,
                                  horizontal_clipping=True):
                        # Spacing is negative because "tileable" texture wasn't
                        # perfectly tileable, so that adds some overlap to line up better.
                        with ui.HStack(spacing=-12):
                            for i in range(50):  # tiling the texture
                                ui.Image(name=TEXTURE_NAME,
                                         fill_policy=ui.FillPolicy.PRESERVE_ASPECT_CROP,
                                         width=50,)

                    slider_cls = (
                        ui.FloatSlider if self.__num_type == "float" else ui.IntSlider
                    )
                    self.__slider = slider_cls(
                        height=FIELD_HEIGHT,
                        min=self.__min, max=self.__max, name="attr_slider"
                    )

                if self.__display_range:
                    self._build_display_range()

            with ui.VStack(width=ui.Fraction(1)):
                model = self.__slider.model
                model.set_value(self.__default_val)
                field_cls = (
                    ui.FloatField if self.__num_type == "float" else ui.IntField
                )

                # Note: This is a hack to allow for text to fill the Field space more, as there was a bug
                # with Field padding.  It is fixed, and will be available in the next release of Kit.
                with ui.ZStack():
                    # height=FIELD_HEIGHT-1 to account for the border, so the field isn't
                    # slightly taller than the slider
                    ui.Rectangle(
                        style_type_name_override="Field",
                        name="attr_field",
                        height=FIELD_HEIGHT - 1
                    )
                    with ui.HStack(height=0):
                        ui.Spacer(width=2)
                        self.__numberfield = field_cls(
                            model,
                            height=0,
                            style={
                                "background_color": cl.transparent,
                                "border_color": cl.transparent,
                                "padding": 4,
                                "font_size": fl.field_text_font_size,
                            },
                        )
                if self.__display_range:
                    ui.Spacer()

        model.add_value_changed_fn(self._on_value_changed)
