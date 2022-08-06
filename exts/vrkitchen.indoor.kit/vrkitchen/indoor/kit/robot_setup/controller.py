# controller
import carb

class Controller():
    def __init__(self) -> None:
        self.user_control = 0.25 
        self.network_control = 0.25

        self.w = False
        self.s = False
        self.a = False
        self.d = False
        self.q = False
        self.e = False

        self.up = False
        self.down = False
        self.left = False
        self.right = False

        # self.scale = 0.1
        self.left_control = False
        
        
    def handle_keyboard_event(self, event):
        if (
            event.type == carb.input.KeyboardEventType.KEY_PRESS
            or event.type == carb.input.KeyboardEventType.KEY_REPEAT
            ): 
            # print("event input", event.input)
            if event.input == carb.input.KeyboardInput.W:
                self.w = True
            if event.input == carb.input.KeyboardInput.S:
                self.s = True
            if event.input == carb.input.KeyboardInput.A:
                self.a = True
            if event.input == carb.input.KeyboardInput.D:
                self.d = True
            if event.input == carb.input.KeyboardInput.Q:
                self.q = True
            if event.input == carb.input.KeyboardInput.E:
                self.e = True
                

            if event.input == carb.input.KeyboardInput.UP:
                self.up = True
            if event.input == carb.input.KeyboardInput.DOWN:
                self.down = True
            if event.input == carb.input.KeyboardInput.LEFT:
                self.left = True
            if event.input == carb.input.KeyboardInput.RIGHT:
                self.right = True

            if event.input == carb.input.KeyboardInput.LEFT_CONTROL:
                self.left_control = True

        if event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # print("event release", event.input)
            if event.input == carb.input.KeyboardInput.W:
                self.w = False
            if event.input == carb.input.KeyboardInput.S:
                self.s = False
            if event.input == carb.input.KeyboardInput.A:
                self.a = False
            if event.input == carb.input.KeyboardInput.D:
                self.d = False
            if event.input == carb.input.KeyboardInput.Q:
                self.q = False
            if event.input == carb.input.KeyboardInput.E:
                self.e = False

            if event.input == carb.input.KeyboardInput.UP:
                self.up = False
            if event.input == carb.input.KeyboardInput.DOWN:
                self.down = False
            if event.input == carb.input.KeyboardInput.LEFT:
                self.left = False
            if event.input == carb.input.KeyboardInput.RIGHT:
                self.right = False

            if event.input == carb.input.KeyboardInput.LEFT_CONTROL:
                self.left_control = False

    def PoolUserControl(self):
        return self.user_control

    def PoolNetworkControl(self):
        return 0.1 if self.w else 0.25

    def QueryMove(self):
        move = [0, 0, 0]
        if self.w:
            move[0] += 1 
        if self.s:
            move[0] -= 1
        if self.a:
            move[1] += 1
        if self.d:
            move[1] -= 1
        if self.q:
            move[2] -= 1
        if self.e:
            move[2] += 1

        return move

    def QueryRotation(self):
        rotation = [0, 0]
        if self.up:
            rotation[0] += 1 
        if self.down:
            rotation[0] -= 1
        if self.left:
            rotation[1] += 1
        if self.right:
            rotation[1] -= 1

        return rotation
        
    def QueryGripper(self):
        if not self.left_control:
            return 1 # open
        else:
            return -1 # close