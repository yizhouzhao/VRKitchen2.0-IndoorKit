# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
__all__ = ["CustomBoolWidget"]

import omni.ui as ui

from .custom_base_widget import CustomBaseWidget


class CustomBoolWidget(CustomBaseWidget):
    """A custom checkbox or switch widget"""

    def __init__(self,
                 model: ui.AbstractItemModel = None,
                 default_value: bool = True,
                 **kwargs):
        self.__default_val = default_value
        self.__bool_image = None

        # Call at the end, rather than start, so build_fn runs after all the init stuff
        CustomBaseWidget.__init__(self, model=model, **kwargs)

    def destroy(self):
        CustomBaseWidget.destroy()
        self.__bool_image = None

    def _restore_default(self):
        """Restore the default value."""
        if self.revert_img.enabled:
            self.__bool_image.checked = self.__default_val
            self.__bool_image.name = (
                "checked" if self.__bool_image.checked else "unchecked"
            )
            self.revert_img.enabled = False

    def _on_value_changed(self):
        """Swap checkbox images and set revert_img to correct state."""
        self.__bool_image.checked = not self.__bool_image.checked
        self.__bool_image.name = (
            "checked" if self.__bool_image.checked else "unchecked"
        )
        self.revert_img.enabled = self.__default_val != self.__bool_image.checked

    def _build_body(self):
        """Main meat of the widget.  Draw the appropriate checkbox image, and
        set up callback.
        """
        with ui.HStack():
            with ui.VStack():
                # Just shift the image down slightly (2 px) so it's aligned the way
                # all the other rows are.
                ui.Spacer(height=2)
                self.__bool_image = ui.Image(
                    name="checked" if self.__default_val else "unchecked",
                    fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                    height=16, width=16, checked=self.__default_val
                )
            # Let this spacer take up the rest of the Body space.
            ui.Spacer()

        self.__bool_image.set_mouse_pressed_fn(
            lambda x, y, b, m: self._on_value_changed())
