# controller
import omni
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

        # self.scale = 0.1
        
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


    def PoolUserControl(self):
        return self.user_control

    def PoolNetworkControl(self):
        return 0.1 if self.w else 0.25

    def QueryMove(self):
        move = [0, 0, 0]
        if self.w:
            move[2] += 1 
        if self.s:
            move[2] -= 1
        if self.a:
            move[0] += 1
        if self.d:
            move[0] -= 1

        return move

    def QueryTurn(self):
        turn = 0
        if self.q:
            turn += 90
        if self.e:
            turn -= 90

        return turn
        
