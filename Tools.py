import Global
import math, numpy
from Switch import *
from Controller import *
from gurobipy import *
from Algo import *
import matplotlib.pyplot as plt
import numpy as np


def set_max_load():
    return float(numpy.random.pareto(3)) * 100


def set_value(switch_num, con_num):

    if Global.TOTAL_CONTROLLER_SET.get_controller(con_num).has_real_switch(switch_num):
        return float(numpy.random.pareto(3)) * 1000
    else:
        return 0


def initial(ctl_num, switch_num, controllers, alpha, beta, gama):

    # 1 controller -> CONTROL_NUM switches
    Global.CONTROL_NUM = ctl_num

    Global.TOTAL_SWITCH = switch_num

    # >= TOTAL_SWITCH / CONTROL_NUM
    if controllers < int(math.ceil(Global.TOTAL_SWITCH / float(Global.CONTROL_NUM))):
        print "Too few controllers, please retry."
        return -1
    else:
        Global.TOTAL_CONTROLLER = controllers

    Global.TOTAL_CONTROLLER_SET = ControllerSet()

    Global.TOTAL_SWITCH_SET = SwitchSet()

    Global.alpha = alpha

    Global.beta = beta

    Global.gama = gama

    k = Global.TOTAL_SWITCH / ctl_num
    p = Global.TOTAL_SWITCH % ctl_num

    # controller [1, k]
    for i in range(0, k):
        temp_controller = Controller(i + 1)
        temp_controller.add_potential_switch(i*Global.CONTROL_NUM + 1,
                                             (i+1)*Global.CONTROL_NUM)
        for temp in range(i*Global.CONTROL_NUM + 1, (i+1)*Global.CONTROL_NUM + 1):
            Global.TOTAL_SWITCH_SET.add_switch(temp_controller.get_potential_switch(temp))
        Global.TOTAL_CONTROLLER_SET.add_controller(temp_controller)

    if p == 0:

        # controller [k + 1, m]
        division = Global.TOTAL_SWITCH / (Global.TOTAL_CONTROLLER - k + 1)
        for i in range(1, Global.TOTAL_CONTROLLER - k + 1):
            temp_controller = Controller(k+i)
            temp_controller.add_potential_switch(max(i*division-Global.CONTROL_NUM/2, 1),
                                                 min(i*division+Global.CONTROL_NUM/2 - 1, Global.TOTAL_SWITCH))
            for temp in range(max(i*division-Global.CONTROL_NUM/2, 1),
                              min(i*division+Global.CONTROL_NUM/2 - 1, Global.TOTAL_SWITCH)):
                Global.TOTAL_SWITCH_SET.add_switch(temp_controller.get_potential_switch(temp))
            Global.TOTAL_CONTROLLER_SET.add_controller(temp_controller)
    else:

        # controller k + 1
        temp_controller = Controller(k + 1)
        temp_controller.add_potential_switch(Global.TOTAL_SWITCH - Global.CONTROL_NUM + 1,
                                             Global.TOTAL_SWITCH)
        for temp in range(Global.TOTAL_SWITCH - Global.CONTROL_NUM + 1, Global.TOTAL_SWITCH):
            Global.TOTAL_SWITCH_SET.add_switch(temp_controller.get_potential_switch(temp))
        Global.TOTAL_CONTROLLER_SET.add_controller(temp_controller)

        # controller [k + 2, m]
        division = Global.TOTAL_SWITCH / (Global.TOTAL_CONTROLLER - k + 1)
        for i in range(2, Global.TOTAL_CONTROLLER - k + 1):
            temp_controller = Controller(k+i)
            temp_controller.add_potential_switch(max(i*division-Global.CONTROL_NUM/2, 1),
                                                 min(i*division+Global.CONTROL_NUM/2 - 1, Global.TOTAL_SWITCH))
            for temp in range(max(i*division-Global.CONTROL_NUM/2, 1),
                              min(i*division+Global.CONTROL_NUM/2 - 1, Global.TOTAL_SWITCH)):
                Global.TOTAL_SWITCH_SET.add_switch(temp_controller.get_potential_switch(temp))
            Global.TOTAL_CONTROLLER_SET.add_controller(temp_controller)

    '''
    for controller_num, controller in Global.TOTAL_CONTROLLER_SET.get_controller_set().iteritems():
        print "C%d: " % controller.get_num(), len([u for u, v in controller.get_PS().iteritems()]), "|",\
            sorted([u for u, v in controller.get_PS().iteritems()])[0], \
            sorted([u for u, v in controller.get_PS().iteritems()])[-1]
    '''

    # set switch weight
    for num, switch in Global.TOTAL_SWITCH_SET.get_switch_set().iteritems():
        switch.set_weight(float(numpy.random.pareto(3)))

    # set controller max load
    for num, controller in Global.TOTAL_CONTROLLER_SET.get_controller_set().iteritems():
        controller.set_max_load(set_max_load())

    '''
    for num, switch in Global.TOTAL_SWITCH_SET.get_switch_set().iteritems():
        print "No.%d, w:%f, PC:" % (num, switch.get_weight()), \
        sorted([o for o, u in switch.get_PC().iteritems()])

    for num, controller in Global.TOTAL_CONTROLLER_SET.get_controller_set().iteritems():
        print "No.%d, w:%f, PS:" % (num, controller.get_controller_weight()), \
        sorted([o for o, u in controller.get_PS().iteritems()])
    '''

    # set controller AN
    for num_1, controller in Global.TOTAL_CONTROLLER_SET.get_controller_set().iteritems():
        for num_2, switch in controller.get_PS().iteritems():
            for num_3, temp in switch.get_PC().iteritems():
                if num_1 != num_3:
                    controller.add_AN(temp)

    '''
    for num, controller in Global.TOTAL_CONTROLLER_SET.get_controller_set().iteritems():
        print "No.%d, w:%f, AN:" % (num, controller.get_controller_weight()), \
            sorted([o for o, u in controller.get_AN().iteritems()])
    '''

    # set Value
    Global.Value = [[set_value(i_1, i_2) for i_1, v_1 in Global.TOTAL_SWITCH_SET.get_switch_set().iteritems()] for
                    i_2, v_2 in Global.TOTAL_CONTROLLER_SET.get_controller_set().iteritems()]


def get_x_sj(x, j):

    new_dict = {}

    for key, val in x.iteritems():
        if key[1] == j:
            new_dict[key] = val

    return new_dict


def argmax(temp_dict):
    return [temp_pair for temp_pair, value in temp_dict.iteritems() if
            value == temp_dict[max(temp_dict, key=lambda i: temp_dict[i])]]


def argmin(temp_dict):
    return [temp_pair for temp_pair, value in temp_dict.iteritems() if
            value == temp_dict[max(temp_dict, key=lambda i: temp_dict[i])]]


# Standard Deviation (for theoretical analysis)

# SAV (for theoretical analysis)

# RWD (for real implementation)
# algo:
# 0: naive LBDC-CM
# 1: limited LBDC-CM
# 2: LBDC-CM prior
# 3: LBDC-DM
def RWD(global_args, algo):

    sigma = 0

    if algo == 2 or algo == 1:
        sigma = math.sqrt(sum([(controller.get_controller_weight() - global_args["Des_now"][num])
                               ** 2 for num, controller in
                               global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems()]) /
                          float(global_args["TOTAL_CONTROLLER"]))
    elif algo == 0:
        sigma = math.sqrt(sum([(controller.get_controller_weight() - global_args["Avg_now"])
                               ** 2 for num, controller in
                               global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems()]) /
                          float(global_args["TOTAL_CONTROLLER"]))
    else:
        sigma = math.sqrt(sum([(controller.get_controller_weight() - controller.get_avg_now())
                               ** 2 for num, controller in
                               global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems()]) /
                          float(global_args["TOTAL_CONTROLLER"]))

    return sigma






