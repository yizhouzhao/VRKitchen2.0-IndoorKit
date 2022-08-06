# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
__all__ = ["CustomMultifieldWidget"]

from typing import List, Optional

import omni.ui as ui

from .custom_base_widget import CustomBaseWidget


class CustomMultifieldWidget(CustomBaseWidget):
    """A custom multifield widget with a variable number of fields, and
    customizable sublabels.
    """

    def __init__(self,
                 model: ui.AbstractItemModel = None,
                 sublabels: Optional[List[str]] = None,
                 default_vals: Optional[List[float]] = None,
                 **kwargs):
        self.__field_labels = sublabels or ["X", "Y", "Z"]
        self.__default_vals = default_vals or [0.0] * len(self.__field_labels)
        self.__multifields = []

        # Call at the end, rather than start, so build_fn runs after all the init stuff
        CustomBaseWidget.__init__(self, model=model, **kwargs)

    def destroy(self):
        CustomBaseWidget.destroy()
        self.__multifields = []

    @property
    def model(self, index: int = 0) -> Optional[ui.AbstractItemModel]:
        """The widget's model"""
        if self.__multifields:
            return self.__multifields[index].model

    @model.setter
    def model(self, value: ui.AbstractItemModel, index: int = 0):
        """The widget's model"""
        self.__multifields[index].model = value

    def _restore_default(self):
        """Restore the default values."""
        if self.revert_img.enabled:
            for i in range(len(self.__multifields)):
                model = self.__multifields[i].model
                model.as_float = self.__default_vals[i]
            self.revert_img.enabled = False

    def _on_value_changed(self, val_model: ui.SimpleFloatModel, index: int):
        """Set revert_img to correct state."""
        val = val_model.as_float
        self.revert_img.enabled = self.__default_vals[index] != val

    def _build_body(self):
        """Main meat of the widget.  Draw the multiple Fields with their
        respective labels, and set up callbacks to keep them updated.
        """
        with ui.HStack():
            for i, (label, val) in enumerate(zip(self.__field_labels, self.__default_vals)):
                with ui.HStack(spacing=3):
                    ui.Label(label, name="multi_attr_label", width=0)
                    model = ui.SimpleFloatModel(val)
                    # TODO: Hopefully fix height after Field padding bug is merged!
                    self.__multifields.append(
                        ui.FloatField(model=model, name="multi_attr_field"))
                if i < len(self.__default_vals) - 1:
                    # Only put space between fields and not after the last one
                    ui.Spacer(width=15)

        for i, f in enumerate(self.__multifields):
            f.model.add_value_changed_fn(lambda v: self._on_value_changed(v, i))
