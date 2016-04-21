import Global
from Switch import *
from copy import *


class Controller(object):

    def __init__(self, num):
        self.num = num
        self.PS = {}
        self.RS = {}
        self.AN = {}
        self.max_load = 0
        self.Avg_now = 0
        self.Avg_last = 0
        self.RList = {}

        # Sending, Receiving, Idle
        self.mode = "Idle"

    def clear_state(self):
        self.RS = {}
        self.RList = {}
        self.Avg_now = 0
        self.Avg_last = 0
        self.mode = "Idle"

    def get_controller_weight(self):
        return sum([v.get_weight() for k, v in self.RS.iteritems()])

    def has_real_switch(self, num):
        return num in self.RS

    def has_potential_switch(self, num):
        return num in self.PS

    def get_potential_switch(self, num):
        return self.PS[num]

    def get_PS(self):
        return self.PS

    def get_RS(self):
        return self.RS

    def get_num(self):
        return self.num

    # [start, end]
    def add_potential_switch(self, start, end):
        for i in range(start, end + 1):
            if not Global.TOTAL_SWITCH_SET.has_switch(i):
                self.PS[i] = Switch(i)
            else:
                self.PS[i] = Global.TOTAL_SWITCH_SET.get_switch(i)
            self.PS[i].add_potential_controller(self)

    def add_real_switch(self, switch):
        self.RS[switch.get_num()] = switch

    def remove_real_switch(self, switch):
        del self.RS[switch.get_num()]

    def add_AN(self, controller):
        self.AN[controller.get_num()] = controller

    def get_AN(self):
        return self.AN

    def get_max_load(self):
        return self.max_load

    def set_max_load(self, num):
        self.max_load = num

    def set_avg_now(self, num):
        self.Avg_now = num

    def set_avg_last(self, num):
        self.Avg_last = num

    def get_avg_last(self):
        return self.Avg_last

    def get_avg_now(self):
        return self.Avg_now

    def set_mode(self, global_args):

        # Sending mode
        if self.get_controller_weight() > global_args["EfnList"][self.num] or \
                        abs(self.get_controller_weight() - global_args["EfnList"][self.num]) < 0.001:
            self.mode = "Sending"

        # Receiving mode
        if self.get_controller_weight() < global_args["ThdList"][self.num] or \
                        abs(self.get_controller_weight() - global_args["ThdList"][self.num]) < 0.001:
            self.mode = "Receiving"

        # Idle mode
        if global_args["ThdList"][self.num] < self.get_controller_weight() < global_args["EfnList"][self.num]:
            self.mode = "Idle"

    def get_mode(self):
        return self.mode

    def get_RList(self):
        return self.RList

    def refresh_RList(self):

        dellist = []

        for num, controller in self.RList.iteritems():
            if controller.get_mode() == "Sending":
                dellist.append(num)

        for item in dellist:
            del self.RList[item]


class ControllerSet(object):

    def __init__(self):
        self.controller_set = {}

    def add_controller(self, controller):
        self.controller_set[controller.get_num()] = controller

    def get_controller_weight(self, num):
        return self.controller_set[num].get_controller_weight()

    def get_controller(self, num):
        return self.controller_set[num]

    def has_controller(self, num):
        return num in self.controller_set

    def get_controller_set(self):
        return self.controller_set

    def get_max_load(self, num):
        return self.controller_set[num].max_load


