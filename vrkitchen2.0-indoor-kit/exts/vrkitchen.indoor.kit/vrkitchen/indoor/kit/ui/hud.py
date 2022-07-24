import omni.kit.viewport_widgets_manager as wm
from omni import ui

CAMERA_WIDGET_STYLING = {
    "Rectangle::background": {"background_color": 0x7F808080, "border_radius": 5}
}

class LabelWidget(wm.WidgetProvider):
    def __init__(self, text0, text1="", text2=""):
        self.text0 = text0
        self.text1 = text1
        self.text2 = text2
    
    def build_widget(self, window):
        with ui.ZStack(width=0, height=0, style=CAMERA_WIDGET_STYLING):
            ui.Rectangle(name="background")
            with ui.VStack(width=0, height=0):
                ui.Spacer(height=4)
                ui.Label(self.text1, width=0, height=0, name="")
                with ui.HStack(width=0, height=0):
                    ui.Spacer(width=4)
                    ui.Label(self.text0, width=0, height=0, name="")
                    ui.Spacer(width=4)
                ui.Spacer(height=4)
                ui.Spacer(height=4)
                ui.Label(self.text2, width=0, height=0, name="")
