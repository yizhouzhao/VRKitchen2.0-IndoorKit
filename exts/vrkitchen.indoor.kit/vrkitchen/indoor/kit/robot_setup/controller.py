# controller
import carb

class Controller():
    w = False
    s = False
    a = False
    d = False
    q = False
    e = False

    up = False
    down = False
    left = False
    right = False

    # Controller.scale = 0.1
    left_control = False

    def __init__(self) -> None:
        self.user_control = 0.25 
        self.network_control = 0.25

        Controller.reset_movement()
    
    @classmethod
    def reset_movement(cls):
        Controller.w = False
        Controller.s = False
        Controller.a = False
        Controller.d = False
        Controller.q = False
        Controller.e = False

        Controller.up = False
        Controller.down = False
        Controller.left = False
        Controller.right = False

        # Controller.left_control = False
        
        
    def handle_keyboard_event(self, event):
        if (
            event.type == carb.input.KeyboardEventType.KEY_PRESS
            or event.type == carb.input.KeyboardEventType.KEY_REPEAT
            ): 
            # print("event input", event.input)
            if event.input == carb.input.KeyboardInput.W:
                Controller.w = True
            if event.input == carb.input.KeyboardInput.S:
                Controller.s = True
            if event.input == carb.input.KeyboardInput.A:
                Controller.a = True
            if event.input == carb.input.KeyboardInput.D:
                Controller.d = True
            if event.input == carb.input.KeyboardInput.Q:
                Controller.q = True
            if event.input == carb.input.KeyboardInput.E:
                Controller.e = True
                

            if event.input == carb.input.KeyboardInput.UP:
                Controller.up = True
            if event.input == carb.input.KeyboardInput.DOWN:
                Controller.down = True
            if event.input == carb.input.KeyboardInput.LEFT:
                Controller.left = True
            if event.input == carb.input.KeyboardInput.RIGHT:
                Controller.right = True

            if event.input == carb.input.KeyboardInput.LEFT_CONTROL:
                Controller.left_control = True

        if event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # print("event release", event.input)
            if event.input == carb.input.KeyboardInput.W:
                Controller.w = False
            if event.input == carb.input.KeyboardInput.S:
                Controller.s = False
            if event.input == carb.input.KeyboardInput.A:
                Controller.a = False
            if event.input == carb.input.KeyboardInput.D:
                Controller.d = False
            if event.input == carb.input.KeyboardInput.Q:
                Controller.q = False
            if event.input == carb.input.KeyboardInput.E:
                Controller.e = False

            if event.input == carb.input.KeyboardInput.UP:
                Controller.up = False
            if event.input == carb.input.KeyboardInput.DOWN:
                Controller.down = False
            if event.input == carb.input.KeyboardInput.LEFT:
                Controller.left = False
            if event.input == carb.input.KeyboardInput.RIGHT:
                Controller.right = False

            if event.input == carb.input.KeyboardInput.LEFT_CONTROL:
                Controller.left_control = False

    def PoolUserControl(self):
        return self.user_control

    def PoolNetworkControl(self):
        return 0.1 if Controller.w else 0.25

    def QueryMove(self):
        move = [0, 0, 0]
        if Controller.w:
            move[0] += 1 
        if Controller.s:
            move[0] -= 1
        if Controller.a:
            move[1] += 1
        if Controller.d:
            move[1] -= 1
        if Controller.q:
            move[2] -= 1
        if Controller.e:
            move[2] += 1

        return move

    def QueryRotation(self):
        rotation = [0, 0]
        if Controller.up:
            rotation[0] += 1 
        if Controller.down:
            rotation[0] -= 1
        if Controller.left:
            rotation[1] += 1
        if Controller.right:
            rotation[1] -= 1

        return rotation
        
    def QueryGripper(self):
        if not Controller.left_control:
            return 1 # open
        else:
            return -1 # close