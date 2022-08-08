# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
__all__ = ["CustomBaseWidget"]

from typing import Optional

import omni.ui as ui

from .style import ATTR_LABEL_WIDTH


class CustomBaseWidget:
    """The base widget for custom widgets that follow the pattern of Head (Label),
    Body Widgets, Tail Widget"""

    def __init__(self, *args, model=None, **kwargs):
        self.existing_model: Optional[ui.AbstractItemModel] = kwargs.pop("model", None)
        self.revert_img = None
        self.__attr_label: Optional[str] = kwargs.pop("label", "")
        self.__frame = ui.Frame()
        with self.__frame:
            self._build_fn()

    def destroy(self):
        self.existing_model = None
        self.revert_img = None
        self.__attr_label = None
        self.__frame = None

    def __getattr__(self, attr):
        """Pretend it's self.__frame, so we have access to width/height and
        callbacks.
        """
        return getattr(self.__frame, attr)

    def _build_head(self):
        """Build the left-most piece of the widget line (label in this case)"""
        ui.Label(
            self.__attr_label,
            name="attribute_name",
            width=120,
        )

    def _build_body(self):
        """Build the custom part of the widget. Most custom widgets will
        override this method, as it is where the meat of the custom widget is.
        """
        ui.Spacer()

    def _build_tail(self):
        """Build the right-most piece of the widget line. In this case,
        we have a Revert Arrow button at the end of each widget line.
        """
        with ui.HStack(width=0):
            ui.Spacer(width=5)
            with ui.VStack(height=0):
                ui.Spacer(height=3)
                self.revert_img = ui.Image(
                    name="revert_arrow",
                    fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                    width=12,
                    height=13,
                    enabled=False,
                )
            ui.Spacer(width=5)

        # call back for revert_img click, to restore the default value
        self.revert_img.set_mouse_pressed_fn(
            lambda x, y, b, m: self._restore_default())

    def _build_fn(self):
        """Puts the 3 pieces together."""
        with ui.HStack():
            self._build_head()
            self._build_body()
            self._build_tail()
