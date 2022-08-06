# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
__all__ = ["TaskTypeComboboxWidget"]

from typing import List, Optional

import omni.ui as ui

from .custom_base_widget import CustomBaseWidget
from .style import BLOCK_HEIGHT


class TaskTypeComboboxWidget(CustomBaseWidget):
    """A customized combobox widget"""

    def __init__(self,
                 model: ui.AbstractItemModel = None,
                 options: List[str] = None,
                 default_value=0,
                 **kwargs):
        self.__default_val = default_value
        self.__options = options or ["1", "2", "3"]
        self.__combobox_widget = None

        # Call at the end, rather than start, so build_fn runs after all the init stuff
        CustomBaseWidget.__init__(self, model=model, **kwargs)

    def destroy(self):
        CustomBaseWidget.destroy()
        self.__options = None
        self.__combobox_widget = None

    @property
    def model(self) -> Optional[ui.AbstractItemModel]:
        """The widget's model"""
        if self.__combobox_widget:
            return self.__combobox_widget.model

    @model.setter
    def model(self, value: ui.AbstractItemModel):
        """The widget's model"""
        self.__combobox_widget.model = value

    def _on_value_changed(self, *args):
        """Set revert_img to correct state."""
        model = self.__combobox_widget.model
        index = model.get_item_value_model().get_value_as_int()
        self.revert_img.enabled = self.__default_val != index

    def _restore_default(self):
        """Restore the default value."""
        if self.revert_img.enabled:
            self.__combobox_widget.model.get_item_value_model().set_value(
                self.__default_val)
            self.revert_img.enabled = False

    def _build_body(self):
        """Main meat of the widget.  Draw the Rectangle, Combobox, and
        set up callbacks to keep them updated.
        """
        with ui.HStack():
            with ui.ZStack():
                # TODO: Simplify when borders on ComboBoxes work in Kit!
                # and remove style rule for "combobox" Rect

                # Use the outline from the Rectangle for the Combobox
                ui.Rectangle(name="combobox",
                             height=BLOCK_HEIGHT)

                option_list = list(self.__options)
                self.__combobox_widget = ui.ComboBox(
                    0, *option_list,
                    name="dropdown_menu",
                    # Abnormal height because this "transparent" combobox
                    # has to fit inside the Rectangle behind it
                    height=10
                )

                # Swap for  different dropdown arrow image over current one
                with ui.HStack():
                    ui.Spacer()  # Keep it on the right side
                    with ui.VStack(width=0):  # Need width=0 to keep right-aligned
                        ui.Spacer(height=5)
                        with ui.ZStack():
                            ui.Rectangle(width=15, height=15, name="combobox_icon_cover")
                            ui.Image(name="collapsable_closed", width=12, height=12)
                    ui.Spacer(width=2)  # Right margin

            ui.Spacer(width=ui.Percent(30))

        self.__combobox_widget.model.add_item_changed_fn(self._on_value_changed)
