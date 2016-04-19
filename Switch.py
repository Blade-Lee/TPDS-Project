from Controller import *
from copy import *


class Switch(object):

    def __init__(self, num, weight=0):
        self.weight = weight
        self.num = num
        self.PC = {}
        self.rc = {}

    def clear_state(self):
        self.rc = {}

    def get_weight(self):
        return self.weight

    def set_weight(self, weight):
        self.weight = weight

    def get_num(self):
        return self.num

    def has_potential_controller(self, num):
        return num in self.PC

    def has_real_controller(self, num):
        return num in self.rc

    def add_potential_controller(self, controller):
        self.PC[controller.get_num()] = controller

    def add_real_controller(self, controller):
        self.rc[controller.get_num()] = controller

    def remove_real_controller(self, controller):
        del self.rc[controller.get_num()]

    def get_PC(self):
        return self.PC

    def get_rc(self):
        return self.rc


class SwitchSet(object):

    def __init__(self):
        self.switch_set = {}

    def add_switch(self, switch):
        self.switch_set[switch.get_num()] = switch

    def get_switch_weight(self, num):
        return self.switch_set[num].get_weight()

    def has_switch(self, num):
        return num in self.switch_set

    def get_switch(self, num):
        return self.switch_set[num]

    def get_switch_set_weight(self):
        return sum([switch.get_weight() for num, switch in self.switch_set.iteritems()])

    def get_switch_set(self):
        return self.switch_set

