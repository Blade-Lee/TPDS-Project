import Global
import math, numpy
from Switch import *
from Controller import *
from gurobipy import *
from Algo import *
import matplotlib.pyplot as plt
import numpy as np


def copy_global_args(input_global_args):

    global_args = {}

    global_args["TOTAL_SWITCH"] = input_global_args["TOTAL_SWITCH"]
    global_args["TOTAL_CONTROLLER"] = input_global_args["TOTAL_CONTROLLER"]

    global_args["TOTAL_CONTROLLER_SET"] = ControllerSet()
    for num, controller in input_global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():

        # set controller num
        temp_controller = Controller(num, controller.get_x_y()[0], controller.get_x_y()[1])

        # set max load
        temp_controller.set_max_load(controller.get_max_load())

        global_args["TOTAL_CONTROLLER_SET"].add_controller(temp_controller)

    global_args["TOTAL_SWITCH_SET"] = SwitchSet()
    for num, switch in input_global_args["TOTAL_SWITCH_SET"].get_switch_set().iteritems():

        # set switch num
        temp_switch = Switch(num, switch.get_x_y()[0], switch.get_x_y()[1])

        # set switch weight
        temp_switch.set_weight(switch.get_weight())

        global_args["TOTAL_SWITCH_SET"].add_switch(temp_switch)

        # set PC and PS
        for num_2, controller in switch.get_PC().iteritems():
            temp_controller = global_args["TOTAL_CONTROLLER_SET"].get_controller(num_2)
            temp_switch.add_potential_controller(temp_controller)
            temp_controller.add_ps(temp_switch)

        # set rc and RS
        for num_2, controller in switch.get_rc().iteritems():
            temp_controller = global_args["TOTAL_CONTROLLER_SET"].get_controller(num_2)
            temp_switch.add_real_controller(temp_controller)
            temp_controller.add_real_switch(temp_switch)

    # set controller AN
    for num, controller in input_global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():

        temp_controller = global_args["TOTAL_CONTROLLER_SET"].get_controller(num)
        for num_2, controller_2 in controller.get_AN().iteritems():
            temp_controller.add_AN(global_args["TOTAL_CONTROLLER_SET"].get_controller(num_2))

    global_args["alpha"] = input_global_args["alpha"]
    global_args["beta"] = input_global_args["beta"]
    global_args["gama"] = input_global_args["gama"]
    global_args["Avg_now"] = input_global_args["Avg_now"]
    global_args["Avg_last"] = input_global_args["Avg_last"]
    global_args["Thd"] = input_global_args["Thd"]
    global_args["Efn"] = input_global_args["Efn"]
    global_args["EfnList"] = deepcopy(input_global_args["EfnList"])
    global_args["ThdList"] = deepcopy(input_global_args["ThdList"])
    global_args["Des_now"] = deepcopy(input_global_args["Des_now"])
    global_args["Des_last"] = deepcopy(input_global_args["Des_last"])
    global_args["Value"] = deepcopy(input_global_args["Value"])

    return global_args


def set_max_load():
    return float(numpy.random.pareto(3)) * 100


def set_value(switch_num, con_num):

    if Global.TOTAL_CONTROLLER_SET.get_controller(con_num).has_real_switch(switch_num):
        return float(numpy.random.pareto(3)) * 1000
    else:
        return 0


def set_new_value(switch, controller):

    if controller.has_real_switch(switch.get_num()):
        x_1, y_1 = switch.get_x_y()
        x_2, y_2 = controller.get_x_y()
        return math.sqrt((x_1 - x_2) ** 2 + (y_1 - y_2) ** 2) * 0.01
    else:
        return 0


def get_square_bound(num_temp):
    for i in range(1, 17):
        if i ** 2 <= num_temp < (i + 1) ** 2:
            return i


def in_range(temp_1, temp_2, dist):
    x_1, y_1 = temp_1.get_x_y()
    x_2, y_2 = temp_2.get_x_y()
    if (abs(x_1 - x_2) < dist or abs(abs(x_1 - x_2) - dist) < 0.001) and \
            (abs(y_1 - y_2) < dist or abs(abs(y_1 - y_2) - dist) < 0.001):
        return True
    else:
        return False


def square_initial(controllers, alpha, beta, gama):

    Global.TOTAL_SWITCH = 10000

    Global.TOTAL_CONTROLLER = controllers

    Global.TOTAL_CONTROLLER_SET = ControllerSet()

    Global.TOTAL_SWITCH_SET = SwitchSet()

    Global.alpha = alpha

    Global.beta = beta

    Global.gama = gama

    #print "Setting nums"
    # set num switches and controllers
    switch_count = 1
    # (0,0) ~ (99, 99)
    for x in range(0, 100):
        for y in range(0, 100):
            Global.TOTAL_SWITCH_SET.add_switch(Switch(switch_count, x, y))
            switch_count += 1

    temp_ctl_num = controllers
    ctl_count = 1
    assigned = set({})

    while temp_ctl_num > 0:
        lower_bound = get_square_bound(temp_ctl_num)
        if lower_bound not in assigned:
            assigned.add(lower_bound)
            distance = 99 / float(lower_bound)
            for index_x in range(0, lower_bound):
                for index_y in range(0, lower_bound):
                    Global.TOTAL_CONTROLLER_SET.add_controller(
                        Controller(ctl_count, (index_x + 0.5) * distance, (index_y + 0.5) * distance))
                    ctl_count += 1
            temp_ctl_num -= lower_bound * lower_bound
        else:
            diag_ctl_num_1 = int(math.ceil(temp_ctl_num / 2.0))
            diag_ctl_num_2 = int(math.floor(temp_ctl_num / 2.0))

            if diag_ctl_num_1 > 0:
                x_distance_1 = 99 / float(diag_ctl_num_1)
                for index_x in range(0, diag_ctl_num_1):
                    Global.TOTAL_CONTROLLER_SET.add_controller(
                        Controller(ctl_count, (index_x + 0.5) * x_distance_1, (index_x + 0.5) * x_distance_1))
                    ctl_count += 1

            if diag_ctl_num_2 > 0:
                x_distance_2 = 99 / float(diag_ctl_num_2)
                for index_x in range(0, diag_ctl_num_2):
                    Global.TOTAL_CONTROLLER_SET.add_controller(
                        Controller(ctl_count, 100 - (index_x + 0.5) * x_distance_2, (index_x + 0.5) * x_distance_2))
                    ctl_count += 1

            temp_ctl_num = 0

    #print "Setting PC and PS"
    # set PC and PS for switches and controllers
    for num_1, switch in Global.TOTAL_SWITCH_SET.get_switch_set().iteritems():
        for num_2, controller in Global.TOTAL_CONTROLLER_SET.get_controller_set().iteritems():
            if in_range(switch, controller, 30):
                controller.add_ps(switch)
                switch.add_potential_controller(controller)

    #print "Setting switch weight"
    # set switch weight
    for num, switch in Global.TOTAL_SWITCH_SET.get_switch_set().iteritems():
        switch.set_weight(float(numpy.random.pareto(3)))

    #print "Setting controller max load"
    # set controller max load
    for num, controller in Global.TOTAL_CONTROLLER_SET.get_controller_set().iteritems():
        controller.set_max_load(set_max_load())

    #print "Setting controller AN"
    # set controller AN (must have at least one switch in common)
    for num_1, controller in Global.TOTAL_CONTROLLER_SET.get_controller_set().iteritems():
        for num_2, switch in controller.get_PS().iteritems():
            for num_3, temp in switch.get_PC().iteritems():
                if in_range(controller, temp, 40) and num_1 != num_3:
                    controller.add_AN(temp)

    #print "Setting Value"
    # set Value
    Global.Value = [[set_new_value(v_1, v_2) for i_1, v_1 in Global.TOTAL_SWITCH_SET.get_switch_set().iteritems()] for
                    i_2, v_2 in Global.TOTAL_CONTROLLER_SET.get_controller_set().iteritems()]


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
# 4: Original
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
    elif algo == 3:
        sigma = math.sqrt(sum([(controller.get_controller_weight() - controller.get_avg_now())
                               ** 2 for num, controller in
                               global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems()]) /
                          float(global_args["TOTAL_CONTROLLER"]))
    else:
        global_args["Avg_now"] = sum([v.get_controller_weight() for c, v in
                              global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems()]) / \
                                 float(global_args["TOTAL_CONTROLLER"])
        global_args["Thd"] = global_args["alpha"] * global_args["Avg_now"] + \
                             (1 - global_args["alpha"]) * global_args["Avg_last"]
        global_args["Efn"] = global_args["beta"] * global_args["Thd"]
        sigma = math.sqrt(sum([(controller.get_controller_weight() - global_args["Avg_now"])
                               ** 2 for num, controller in
                               global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems()]) /
                          float(global_args["TOTAL_CONTROLLER"]))
    return sigma






