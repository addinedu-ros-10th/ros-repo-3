class State:
    def __init__(self, state_id, name, transitions):
        """
        transitions: dict { event_name: next_state_id }
        """
        self.state_id = state_id
        self.name = name
        self.transitions = transitions

    def __repr__(self):
        return f"State({self.state_id}, {self.name}, transitions={self.transitions})"


class StateMachine:
    def __init__(self, states, initial_state_id):
        self.states = states
        self.current_state = self.states[initial_state_id]

    def get_state(self):
        return self.current_state

    def recv_event(self, event):
        """
        Receive an external event and transition accordingly.
        """

        transitions = self.current_state.transitions

        # If the event exists → use it
        if event in transitions:
            next_id = transitions[event]

            if next_id not in self.states:
                raise Exception(f"[ERROR] Undefined next state ID {next_id}")

            self.current_state = self.states[next_id]
            print(f"[INFO] State machine get '{event}'! Next State is {self.current_state.name}")
            
        else:
            pass
            # print(f"[ERROR] No transition for event '{event}' in state {self.current_state.name}")

        return self.current_state


STATE_TABLE = {
    0: State(0, "CART_INIT",          {"NEXT": 1}),               # CHARGE_STBY
    1: State(1, "CHARGE_STBY",        {"NEXT": 2}),               # TASK_STBY

    # TASK_STBY → event-driven (online/offline)
    2: State(2, "TASK_STBY",          {"ONLINE": 3, "OFFLINE": 33}),
    3: State(3, "ONLINE_DRIVE",       {"PICKUP": 4, "PACKING": 6}),
    4: State(4, "ONLINE_PICKUP",      {"STBY": 5}),
    5: State(5, "ONLINE_STBY",        {"DRIVE": 3}),
    6: State(6, "ONLINE_PACKING",     {"END": 7}),
    7: State(7, "ONLINE_END",         {"RETURN": 255}),

    33: State(33, "OFFLINE_IDCHECK",  {"USERCHECK": 34}),
    34: State(34, "OFFLINE_USERCHECK",{"STBY": 35}),
    # OFFLINE_STBY → two branches (follow/drive)
    35: State(35, "OFFLINE_STBY",     {"FOLLOW": 36, "DRIVE": 37}),
    36: State(36, "OFFLINE_FOLLOW",   {"STBY": 35}),
    # OFFLINE_DRIVE → two branches (stby/packing)
    37: State(37, "OFFLINE_DRIVE",    {"STBY": 35, "PACKING": 38}),
    38: State(38, "OFFLINE_PACKING",  {"END": 39}),
    39: State(39, "OFFLINE_END",      {"RETURN": 255}),
    255: State(255, "RETRUN_CHRG",         {"CHARGE": 1}),               # CHARGE_STBY
}