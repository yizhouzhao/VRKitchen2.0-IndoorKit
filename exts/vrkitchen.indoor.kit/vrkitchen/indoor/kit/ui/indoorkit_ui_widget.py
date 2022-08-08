from typing import List, Optional

import omni
import omni.ui as ui

from .style import ATTR_LABEL_WIDTH, cl, fl
from .custom_base_widget import CustomBaseWidget
from ..robot_setup.controller import Controller

SPACING = 5 

class CustomRecordGroup:
    STYLE = {
        "Rectangle::image_button": {
            "background_color": 0x0, 
            "border_width": 1.5, 
            "border_radius": 2.0,
            "margin": 4,
            "border_color": cl.btn_border,
            "corner_flag": ui.CornerFlag.RIGHT,
            },
        "Rectangle::image_button:hovered": {
            "background_color": 0xAAB8B8B8,
            "border_width": 0,
            "border_radius": 2.0,
        },
        "Rectangle::image_button:selected": {
            "background_color": 0x0,
            "border_width": 1,
            "border_color": 0xFFC5911A,
            "border_radius": 2.0,
        },
    }

    def __init__(self, width = 60, height = 60,
        on_click_record_fn: callable = None,
        on_click_stop_fn: callable = None,
        on_click_replay_fn: callable = None,
    ):

        self.timeline = omni.timeline.get_timeline_interface()
        self.on_click_record_fn = on_click_record_fn
        self.on_click_stop_fn = on_click_stop_fn
        self.on_click_replay_fn = on_click_replay_fn

        # another ui for control
        self.control_group : CustomControlGroup = None 


        self._selected = False
        with ui.HStack():
            with ui.HStack():
                with ui.ZStack(width=0, height=0, spacing=0): # 
                    with ui.Placer(offset_x=width, offset_y=0):
                        self.play_label = ui.Label("Record", width = 60)

                    with ui.Placer(offset_x=0, offset_y=0):
                        self.rect_play = ui.Rectangle(name="image_button", width=2 * width, height=height, style=CustomRecordGroup.STYLE)
        
                    with ui.Placer(offset_x=5, offset_y=5):
                        self.image_play = ui.Image(
                        name="start_on", width=width - 10, height=height - 10, fill_policy=ui.FillPolicy.STRETCH
                        )  
                
                    self.rect_play.set_mouse_pressed_fn(lambda x, y, btn, a: self._on_mouse_pressed_play(btn))

            with ui.ZStack(width=0, height=0, spacing=0): # 
                with ui.Placer(offset_x=width, offset_y=0):
                    self.stop_label = ui.Label("Stop", width = 60)

                with ui.Placer(offset_x=0, offset_y=0):
                    self.rect_stop = ui.Rectangle(name="image_button", width=2 * width, height=height, style=CustomRecordGroup.STYLE)
    
                with ui.Placer(offset_x=5, offset_y=5):
                    self.image_stop = ui.Image(
                    name="stop_on", width=width - 10, height=height - 10, fill_policy=ui.FillPolicy.STRETCH
                    )  
            
                self.rect_stop.set_mouse_pressed_fn(lambda x, y, btn, a: self._on_mouse_pressed_stop(btn)) # 

            with ui.HStack():
                with ui.ZStack(width=0, height=0, spacing=0):

                    with ui.Placer(offset_x=width, offset_y=0):
                        self.replay_label = ui.Label("Replay", width = 60)

                    with ui.Placer(offset_x=0, offset_y=0):
                        self.rect_replay = ui.Rectangle(name="image_button", width= 2 * width, height=height, style=CustomRecordGroup.STYLE)
        
                    with ui.Placer(offset_x=10, offset_y=10):
                        self.image_replay = ui.Image(
                        name="replay_on", width=width - 20, height=height - 20, fill_policy=ui.FillPolicy.STRETCH
                        )  
                
                    self.rect_replay.set_mouse_pressed_fn(lambda x, y, btn, a: self._on_mouse_pressed_replay(btn))


    def __del__(self):
        # set ui.Image objects to None explicitly to avoid this error:
        # Client omni.ui Failed to acquire interface [omni::kit::renderer::IGpuFoundation v0.2] while unloading all plugins
        self.image_play = None


    def _on_mouse_pressed_play(self, key):
        # 0 is for mouse left button
        if key == 0:
            if self.timeline.is_stopped(): # if stopped, start recording
                self.play_label.text = "Pause"
                self.image_play.name = "pause_on"
                self.on_click_record_fn()

                if self.control_group:
                    self.control_group.enable()

            elif self.timeline.is_playing(): # if is playing, pause
                self.play_label.text = "Continue"
                self.image_play.name = "start_on"
                self.timeline.pause()
            else: # if is paused, just play
                self.play_label.text = "Pause"
                self.image_play.name = "pause_on"
                self.timeline.play()


    
    def _on_mouse_pressed_replay(self, key):
        # 0 is for mouse left button
        if key == 0:
            if self.timeline.is_stopped(): # if stopped, start recording
                self.replay_label.text = "Pause"
                self.image_replay.name = "pause_on"
                self.on_click_replay_fn()
            elif self.timeline.is_playing(): # if is playing, pause
                self.replay_label.text = "Continue"
                self.image_replay.name = "replay_on"
                self.timeline.pause()
            else: # if is paused, just play
                self.replay_label.text = "Pause"
                self.image_replay.name = "pause_on"
                self.timeline.play()

    def _on_mouse_pressed_stop(self, key):
        # print("press stop button", self.timeline.is_playing(), self.timeline.is_stopped())
        # 0 is for mouse left button
        if key == 0:
            self.play_label.text = "Record"
            self.image_play.name = "start_on"

            self.replay_label.text = "Replay"
            self.image_replay.name = "replay_on"
            
            self.on_click_stop_fn()

            if self.control_group:
                self.control_group.disable()

    @property
    def selected(self):
        return self._selected

    @selected.setter
    def selected(self, value):
        self._selected = value

class CustomControlGroup():
    def __init__(self) -> None:
        self.collapse_frame = ui.CollapsableFrame("\tRobot control")
        self.collapse_frame.collapsed = False
        self.collapse_frame.enabled = True
        
        # ui 
        with self.collapse_frame:
            with ui.VStack(height=0, spacing=0):
                with ui.HStack():
                    ui.Label("position control: ")
                    self.button_w = ui.Button("W", name = "control_button", tooltip = "move end factor forward")
                    self.button_s = ui.Button("S", name = "control_button", tooltip = "move end factor backward")
                    self.button_a = ui.Button("A", name = "control_button", tooltip = "move end factor to left")
                    self.button_d = ui.Button("D", name = "control_button", tooltip = "move end factor to right")

                    self.button_q = ui.Button("Q", name = "control_button", tooltip = "move end factor to down")
                    self.button_e = ui.Button("E", name = "control_button", tooltip = "move end factor to up")

                with ui.HStack():
                    ui.Label("rotation control: ")
                    self.button_up = ui.Button("UP", name = "control_button",  tooltip = "Rotate hand upward")
                    self.button_down = ui.Button("DOWN", name = "control_button", tooltip = "Rotate hand downard")
                    self.button_left = ui.Button("LEFT", name = "control_button", tooltip = "Rotate hand to left")
                    self.button_right = ui.Button("RIGHT", name = "control_button", tooltip = "Rotate hand to right")

                with ui.HStack():
                    ui.Label("gripper control: ")
                    self.button_control = ui.Button("LEFT CTRL", name = "control_button", tooltip = "Close/Open gripper")
            
        self.button_list = [self.button_w, self.button_s, self.button_a, self.button_d, self.button_q, self.button_e,
                            self.button_up, self.button_down, self.button_left, self.button_right,
                            ]
    
        self.button_w.set_clicked_fn(lambda : self._on_button("w"))
        self.button_s.set_clicked_fn(lambda : self._on_button("s"))
        self.button_a.set_clicked_fn(lambda : self._on_button("a"))
        self.button_d.set_clicked_fn(lambda : self._on_button("d"))
        self.button_q.set_clicked_fn(lambda : self._on_button("q"))
        self.button_e.set_clicked_fn(lambda : self._on_button("e"))

        self.button_up.set_clicked_fn(lambda : self._on_button("up", 2))
        self.button_down.set_clicked_fn(lambda : self._on_button("down", 2))
        self.button_left.set_clicked_fn(lambda : self._on_button("left", 2))
        self.button_right.set_clicked_fn(lambda : self._on_button("right", 2))

        self.button_control.set_clicked_fn(lambda: self._on_button_control())

        self.disable()
    
    def enable(self):
        """
        Enable itself by showing the robot controling buttons
        """
        self.collapse_frame.collapsed = False
        self.collapse_frame.enabled = True

        self.enable_buttons()
    
    def disable(self):
        """
        Disable itself by closing the robot controling buttons
        """
        self.collapse_frame.collapsed = True
        # self.collapse_frame.enabled = False


    def disable_buttons(self):
        for button in self.button_list:
            button.name = "control_button_disabled"
            # button.enabled = False
            Controller.reset_movement()
    
    def enable_buttons(self):
        for button in self.button_list:
            button.enabled = True
            button.name = "control_button"
            Controller.reset_movement()
    

    def _on_button(self, attr_name:str, style = 1):
        attr = getattr(Controller, attr_name)
        # print("attr", attr_name, attr)
        button = getattr(self, f"button_{attr_name}")
        if attr:
            setattr(Controller, attr_name, False)
            button.name = "control_button"
            self.enable_buttons()
        else:
            self.disable_buttons()
            setattr(Controller, attr_name, True)
            button.enabled = True
            button.name = f"control_button_pressed{style}"

    def _on_button_control(self):
        if Controller.left_control:
            Controller.left_control = False
            self.button_control.text = "LEFT CTRL"
            self.button_control.name = "control_button"
        else:
            Controller.left_control = True
            self.button_control.text = "Gripper closed"
            self.button_control.name = "control_button_pressed3"


class CustomBoolWidget(CustomBaseWidget):
    """A custom checkbox or switch widget"""

    def __init__(self,
                 model: ui.AbstractItemModel = None,
                 default_value: bool = True,
                 on_checked_fn: callable = None,
                 **kwargs):
        self.__default_val = default_value
        self.__bool_image = None
        self.on_checked_fn = on_checked_fn

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
        
        if self.on_checked_fn:
            self.on_checked_fn(self.__bool_image.checked)

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
                 num_type: str = "int",
                 min=0.0,
                 max=1.0,
                 default_val=0.0,
                 display_range: bool = False,
                 on_slide_fn: callable = None,
                 **kwargs):
        self.__slider: Optional[ui.AbstractSlider] = None
        self.__numberfield: Optional[ui.AbstractField] = None
        self.__min = min
        self.__max = max
        self.__default_val = default_val
        self.__num_type = num_type
        self.__display_range = display_range
        self.on_slide_fn = on_slide_fn

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

        if self.on_slide_fn:
            self.on_slide_fn(index)

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


class CustomSkySelectionGroup(CustomBaseWidget):
    def __init__(self,
        on_select_fn: callable = None 
    ) -> None:
        self.on_select_fn = on_select_fn
        self.sky_type = ""
        CustomBaseWidget.__init__(self, label = "Sky type:")

    def _build_body(self):
        with ui.HStack():
            self.button_clear = ui.Button("Sunny", name = "control_button")
            self.button_cloudy = ui.Button("Cloudy", name = "control_button")
            self.button_overcast = ui.Button("Overcast", name = "control_button")
            self.button_night = ui.Button("Night", name = "control_button")

        self.button_clear.set_clicked_fn(lambda : self._on_button("clear"))
        self.button_cloudy.set_clicked_fn(lambda : self._on_button("cloudy"))
        self.button_overcast.set_clicked_fn(lambda : self._on_button("overcast"))
        self.button_night.set_clicked_fn(lambda : self._on_button("night"))

        self.button_list = [self.button_clear, self.button_cloudy, self.button_overcast,  self.button_night]

    def enable_buttons(self):
        for button in self.button_list:
            button.enabled = True
            button.name = "control_button"

    def _on_button(self, sky_type:str):
        if self.on_select_fn:
            self.on_select_fn(sky_type.capitalize())
        self.enable_buttons()
        button = getattr(self, f"button_{sky_type}")
        button.name = f"control_button_pressed{2}"
        self.revert_img.enabled = True

    def _restore_default(self):
        """Restore the default value."""
        if self.revert_img.enabled:
            self.revert_img.enabled = False
            self.enable_buttons()
            self.on_select_fn("")


class CustomIdNotice():
    def __init__(self) -> None:
        self.ui = ui.HStack()
        with self.ui:
            ui.Spacer(width=4)
            self.task_ui = ui.Button("pickup_object", name = "control_button", style = {"color": "lightsteelblue", "border_color": "lightsteelblue"}, enabled = False)
            ui.Spacer(width=4)
            self.object_ui = ui.Button("object: 0", name = "control_button", style = {"color": "DarkSalmon", "border_color": "DarkSalmon"}, enabled = False)
            ui.Spacer(width=4)
            self.house_ui = ui.Button("house: 1", name = "control_button", style = {"color": "Plum", "border_color": "Plum"}, enabled = False)
        
        self.ui.visible = False


class CustomRenderTypeSelectionGroup(CustomBaseWidget):
    def __init__(self,
        on_select_fn: callable = None 
    ) -> None:
        self.on_select_fn = on_select_fn
        self.sky_type = ""
        CustomBaseWidget.__init__(self, label = "Render type:")

    def _build_body(self):
        with ui.HStack():
            self.button_gdb = ui.Button("RGB", name = "control_button")
            self.button_depth= ui.Button("Depth", name = "control_button")
            self.button_semanic = ui.Button("Semantic Map", name = "control_button")

        # self.button_clear.set_clicked_fn(lambda : self._on_button("clear"))
        # self.button_cloudy.set_clicked_fn(lambda : self._on_button("cloudy"))
        # self.button_overcast.set_clicked_fn(lambda : self._on_button("overcast"))
        # self.button_night.set_clicked_fn(lambda : self._on_button("night"))

        # self.button_list = [self.button_clear, self.button_cloudy, self.button_overcast,  self.button_night]

    def enable_buttons(self):
        for button in self.button_list:
            button.enabled = True
            button.name = "control_button"

    def _on_button(self, sky_type:str):
        if self.on_select_fn:
            self.on_select_fn(sky_type.capitalize())
        self.enable_buttons()
        button = getattr(self, f"button_{sky_type}")
        button.name = f"control_button_pressed{2}"
        self.revert_img.enabled = True

    def _restore_default(self):
        """Restore the default value."""
        if self.revert_img.enabled:
            self.revert_img.enabled = False
            self.enable_buttons()
            self.on_select_fn("")

class CustomPathButtonWidget:
    """A compound widget for holding a path in a StringField, and a button
    that can perform an action.
    TODO: Get text ellision working in the path field, to start with "..."
    """
    def __init__(self,
                 label: str,
                 path: str,
                 btn_label: str,
                 btn_callback: callable = None):
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
        with ui.VStack():
            with ui.HStack():
                ui.Label(
                    self.__attr_label,
                    name="attribute_name",
                    width=120,
                )
                self.__pathfield = ui.StringField(
                    name="path_field",
                    height=20,
                )
                
                # # TODO: Add clippingType=ELLIPSIS_LEFT for long paths
                self.__pathfield.model.set_value(self.__path)

                self.__btn = ui.Button(
                    self.__btn_label,
                    name="tool_button",
                    height=20,
                    clicked_fn=lambda path=self.get_path(): self.__callback(path),
                )

                ui.Spacer(width = 8)
