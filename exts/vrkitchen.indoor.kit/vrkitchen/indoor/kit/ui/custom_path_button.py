# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
__all__ = ["CustomPathButtonWidget"]

from typing import Callable, Optional

import omni.ui as ui

from .style import ATTR_LABEL_WIDTH, BLOCK_HEIGHT


class CustomPathButtonWidget:
    """A compound widget for holding a path in a StringField, and a button
    that can perform an action.
    TODO: Get text ellision working in the path field, to start with "..."
    """
    def __init__(self,
                 label: str,
                 path: str,
                 btn_label: str,
                 btn_callback: Callable):
        self.__attr_label = label
        self.__pathfield: ui.StringField = None
        self.__path = path
        self.__btn_label = btn_label
        self.__btn = None
        self.__callback = btn_callback
        self.__frame = ui.Frame()

        with self.__frame:
            self._build_fn()

    def destroy(self):
        self.__pathfield = None
        self.__btn = None
        self.__callback = None
        self.__frame = None

    @property
    def model(self) -> Optional[ui.AbstractItem]:
        """The widget's model"""
        if self.__pathfield:
            return self.__pathfield.model

    @model.setter
    def model(self, value: ui.AbstractItem):
        """The widget's model"""
        self.__pathfield.model = value

    def get_path(self):
        return self.model.as_string

    def _build_fn(self):
        """Draw all of the widget parts and set up callbacks."""
        with ui.HStack():
            ui.Label(
                self.__attr_label,
                name="attribute_name",
                width=ATTR_LABEL_WIDTH
            )
            self.__pathfield = ui.StringField(
                name="path_field",
                height=BLOCK_HEIGHT,
                width=ui.Fraction(2),
            )
            # TODO: Add clippingType=ELLIPSIS_LEFT for long paths
            self.__pathfield.model.set_value(self.__path)

            self.__btn = ui.Button(
                self.__btn_label,
                name="tool_button",
                height=BLOCK_HEIGHT,
                width=ui.Fraction(1),
                clicked_fn=lambda path=self.get_path(): self.__callback(path),
            )
