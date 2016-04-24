import Global
from gurobipy import *
import random
from Tools import *
from CC import *


# Rounding Algorithms
def lbdc_dr(x):
    for j, switch in Global.TOTAL_SWITCH_SET.get_switch_set().iteritems():
        pair_list = argmax(get_x_sj(x, j))
        l = pair_list[0][0]

        if len(pair_list) > 1:
            controller_weight_dict = {}
            for u in [k[0] for k in pair_list]:
                controller_weight_dict[u] = Global.TOTAL_CONTROLLER_SET.get_controller_weight(u)

            l = argmin(controller_weight_dict)[0][0]

        x[l, j] = 1

        for i, controller in Global.TOTAL_CONTROLLER_SET.get_controller_set():
            if i != l:
                x[i, j] = 0


def lbdc_rr(x):
    pass


# Centralized Migration
def lbdc_ci():

    global_args = {}

    global_args["TOTAL_SWITCH"] = copy(Global.TOTAL_SWITCH)
    global_args["TOTAL_CONTROLLER"] = copy(Global.TOTAL_CONTROLLER)
    global_args["TOTAL_CONTROLLER_SET"] = copy(Global.TOTAL_CONTROLLER_SET)
    global_args["TOTAL_SWITCH_SET"] = copy(Global.TOTAL_SWITCH_SET)
    global_args["alpha"] = copy(Global.alpha)
    global_args["beta"] = copy(Global.beta)
    global_args["gama"] = copy(Global.gama)
    global_args["Avg_now"] = copy(Global.Avg_now)
    global_args["Avg_last"] = copy(Global.Avg_last)
    global_args["Thd"] = copy(Global.Thd)
    global_args["Efn"] = copy(Global.Efn)
    global_args["EfnList"] = copy(Global.EfnList)
    global_args["ThdList"] = copy(Global.ThdList)
    global_args["Des_now"] = copy(Global.Des_now)
    global_args["Des_last"] = copy(Global.Des_last)
    global_args["Value"] = copy(Global.Value)

    # clear states
    for num, controller in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
        controller.clear_state()

    for num, switch in global_args["TOTAL_SWITCH_SET"].get_switch_set().iteritems():
        switch.clear_state()

    # assignment
    RemList = [k for k, v in global_args["TOTAL_SWITCH_SET"].get_switch_set().iteritems()]
    for switch_num in RemList:
        pc_weights_dict = {}

        for num, controller in global_args["TOTAL_SWITCH_SET"].get_switch(switch_num).get_PC().iteritems():
            pc_weights_dict[num] = controller.get_controller_weight()

        l = argmin(pc_weights_dict)[0]
        global_args["TOTAL_CONTROLLER_SET"].get_controller(l).add_real_switch(
            global_args["TOTAL_SWITCH_SET"].get_switch(switch_num))
        global_args["TOTAL_SWITCH_SET"].get_switch(switch_num).add_real_controller(
            global_args["TOTAL_CONTROLLER_SET"].get_controller(l))

    # initial state
    global_args["Avg_last"] = sum([v.get_controller_weight() for c, v in
                           global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems()]) / \
                      float(global_args["TOTAL_CONTROLLER"])
    global_args["Avg_now"] = 0

    for c_1, v_1 in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
        global_args["Des_last"][c_1] = sum([v.get_weight() for c, v in
                                    global_args["TOTAL_SWITCH_SET"].get_switch_set().iteritems()]) / \
                               float(sum([k.get_max_load() for j, k in
                                          global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems()])) \
                               * v_1.get_max_load()
        global_args["Des_now"][c_1] = 0

    return global_args


def lbdc_cm(global_args):

    global_args["Avg_now"] = sum([v.get_controller_weight() for c, v in
                          global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems()]) / \
                             float(global_args["TOTAL_CONTROLLER"])

    global_args["Thd"] = global_args["alpha"] * global_args["Avg_now"] + \
                         (1 - global_args["alpha"]) * global_args["Avg_last"]
    global_args["Efn"] = global_args["beta"] * global_args["Thd"]

    PendList = set({})
    old_PendList = set({})
    OverList = set({})


    # Step 1
    for num, controller in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
        if controller.get_controller_weight() > global_args["Efn"]:
            OverList.add(num)

    if len(OverList) == 0:
        return global_args

    # Step 3
    while True:

        while len(OverList) > 0:

            # Step 2
            c_m = global_args["TOTAL_CONTROLLER_SET"].get_controller(
                max([x for x in OverList], key=lambda q: global_args["TOTAL_CONTROLLER_SET"].get_controller_weight(q)))

            c_m_an = [v for x, v in c_m.get_AN().iteritems()]

            # No c_n for this c_m
            if len([1 for v in c_m_an if abs(v.get_controller_weight() - global_args["Thd"]) < 0.001 or
                            v.get_controller_weight() > global_args["Thd"]]) == 0:
                OverList.remove(c_m.get_num())
                PendList.add(c_m.get_num())
            else:
                flag = True

                c_m_rs_refer_num = set({})

                for i, v in c_m.get_RS().iteritems():
                    c_m_rs_refer_num.add(i)

                while flag:

                    if len(c_m_rs_refer_num) == 0:
                        break

                    s_m = max([global_args["TOTAL_SWITCH_SET"].get_switch(x) for x in c_m_rs_refer_num],
                              key=lambda a: a.get_weight())

                    for c_f in c_m_an:
                        if c_f.get_num() in s_m.get_PC() and c_f.get_controller_weight() < global_args["Thd"]:
                            s_m.remove_real_controller(c_m)
                            c_m.remove_real_switch(s_m)
                            c_f.add_real_switch(s_m)
                            s_m.add_real_controller(c_f)
                            break

                    c_m_rs_refer_num.remove(s_m.get_num())

                    # w'(c_m) <= Thd
                    if c_m.get_controller_weight() < global_args["Thd"] \
                            or abs(c_m.get_controller_weight() - global_args["Thd"]) < 0.001:
                        flag = False

                if c_m.get_controller_weight() > global_args["Efn"]:
                    OverList.remove(c_m.get_num())
                    PendList.add(c_m.get_num())
                else:
                    OverList.remove(c_m.get_num())

        for item in PendList:
            OverList.add(item)

        if PendList == old_PendList:
            break
        else:
            old_PendList = copy(PendList)
            PendList = set({})

    # Step 5
    while True:

        # Step 4
        CC_total = CC_set(PendList)

        for each_CC in CC_total.get_CC_set():

            total_an = {}
            for each_con_num in each_CC:
                for an_num, an_con in \
                        global_args["TOTAL_CONTROLLER_SET"].get_controller(each_con_num).get_AN().iteritems():
                    if an_num not in total_an and an_num not in each_CC:
                        total_an[an_num] = an_con

            cc_weight = sum([global_args["TOTAL_CONTROLLER_SET"].get_controller_weight(x) for x in each_CC])
            an_weight = 0

            if len(total_an) > 0:
                an_weight = sum([k.get_controller_weight() for v, k in total_an.iteritems()])

            avg_local = (cc_weight+an_weight) / float(len(each_CC) + len(total_an))

            for c_j_num in each_CC:
                c_j = global_args["TOTAL_CONTROLLER_SET"].get_controller(c_j_num)

                if len(c_j.get_RS()) > 0:

                    while (c_j.get_controller_weight() > global_args["gama"] * avg_local
                           or abs(c_j.get_controller_weight() - global_args["gama"] * avg_local) < 0.001) \
                            and len(total_an) > 0:

                        s_max = max([v for k, v in c_j.get_RS().iteritems()], key=lambda o: o.get_weight())
                        c_min = min([v for k, v in total_an.iteritems()], key=lambda o: o.get_controller_weight())

                        s_max.remove_real_controller(c_j)
                        c_j.remove_real_switch(s_max)
                        c_min.add_real_switch(s_max)
                        s_max.add_real_controller(c_min)

                PendList.remove(c_j_num)

        if PendList == old_PendList:
            break
        else:
            old_PendList = copy(PendList)

    global_args["Avg_last"] = global_args["Avg_now"]

    return global_args


def limited_lbdc_cm(global_args):

    global_args["Des_now"] = {}
    global_args["ThdList"] = {}
    global_args["EfnList"] = {}

    for c_1, v_1 in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
        global_args["Des_now"][c_1] = sum([v.get_weight() for c, v in
                                    global_args["TOTAL_SWITCH_SET"].get_switch_set().iteritems()]) / \
                               float(sum([k.get_max_load() for j, k in
                                          global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems()])) * \
                              v_1.get_max_load()

        global_args["ThdList"][c_1] = global_args["alpha"] * global_args["Des_now"][c_1] + \
                                      (1 - global_args["alpha"]) * global_args["Des_last"][c_1]
        global_args["EfnList"][c_1] = global_args["beta"] * global_args["ThdList"][c_1]

    PendList = set({})
    old_PendList = set({})
    OverList = set({})

    # Step 1
    for num, controller in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
        if controller.get_controller_weight() > global_args["EfnList"][num]:
            OverList.add(num)

    if len(OverList) == 0:
        return global_args

    # Step 3
    while True:

        while len(OverList) > 0:

            # Step 2

            c_m = global_args["TOTAL_CONTROLLER_SET"].get_controller(
                max([x for x in OverList], key=lambda q: global_args["TOTAL_CONTROLLER_SET"].get_controller_weight(q)))

            c_m_an = [v for x, v in c_m.get_AN().iteritems()]

            # No c_n for this c_m
            if len([1 for v in c_m_an if abs(v.get_controller_weight() - global_args["ThdList"][v.get_num()]) < 0.001 or
                            v.get_controller_weight() > global_args["ThdList"][v.get_num()]]) == 0:
                OverList.remove(c_m.get_num())
                PendList.add(c_m.get_num())
            else:
                flag = True

                c_m_rs_refer_num = set({})

                for i, v in c_m.get_RS().iteritems():
                    c_m_rs_refer_num.add(i)

                while flag:

                    if len(c_m_rs_refer_num) == 0:
                        break

                    s_m = max([global_args["TOTAL_SWITCH_SET"].get_switch(x) for x in c_m_rs_refer_num],
                              key=lambda a: a.get_weight())

                    for c_f in c_m_an:
                        if c_f.get_num() in s_m.get_PC() and \
                                        c_f.get_controller_weight() < global_args["ThdList"][c_f.get_num()]:
                            s_m.remove_real_controller(c_m)
                            c_m.remove_real_switch(s_m)
                            c_f.add_real_switch(s_m)
                            s_m.add_real_controller(c_f)
                            break

                    c_m_rs_refer_num.remove(s_m.get_num())

                    # w'(c_m) <= Thd
                    if c_m.get_controller_weight() < global_args["ThdList"][c_m.get_num()] \
                            or abs(c_m.get_controller_weight() - global_args["ThdList"][c_m.get_num()]) < 0.001:
                        flag = False

                if c_m.get_controller_weight() > global_args["EfnList"][c_m.get_num()]:
                    OverList.remove(c_m.get_num())
                    PendList.add(c_m.get_num())
                else:
                    OverList.remove(c_m.get_num())

        for item in PendList:
            OverList.add(item)

        if PendList == old_PendList:
            break
        else:
            old_PendList = copy(PendList)
            PendList = set({})

    # Step 5
    while True:

        # Step 4
        CC_total = CC_set(PendList)

        for each_CC in CC_total.get_CC_set():

            total_an = {}
            for each_con_num in each_CC:
                for an_num, an_con in global_args["TOTAL_CONTROLLER_SET"].get_controller(each_con_num).get_AN().iteritems():
                    if an_num not in total_an and an_num not in each_CC:
                        total_an[an_num] = an_con

            cc_weight = sum([global_args["TOTAL_CONTROLLER_SET"].get_controller_weight(x) for x in each_CC])
            an_weight = 0

            load_sum_cc = sum([global_args["TOTAL_CONTROLLER_SET"].get_controller(v).get_max_load() for v in each_CC])
            load_sum_an = 0

            if len(total_an) > 0:
                an_weight = sum([k.get_controller_weight() for v, k in total_an.iteritems()])
                load_sum_an = sum([v.get_max_load() for k, v in total_an.iteritems()])

            e_local = (cc_weight+an_weight) / float(load_sum_an + load_sum_cc)

            for c_j_num in each_CC:

                c_j = global_args["TOTAL_CONTROLLER_SET"].get_controller(c_j_num)

                if len(c_j.get_RS()) > 0:

                    while (c_j.get_controller_weight() > global_args["gama"] * e_local * c_j.get_max_load()
                           or abs(c_j.get_controller_weight() - global_args["gama"] * e_local * c_j.get_max_load()) < 0.001) \
                            and len(total_an) > 0:

                        s_max = max([v for k, v in c_j.get_RS().iteritems()], key=lambda o: o.get_weight())
                        c_min = min([v for k, v in total_an.iteritems()], key=lambda o: o.get_controller_weight())

                        s_max.remove_real_controller(c_j)
                        c_j.remove_real_switch(s_max)
                        c_min.add_real_switch(s_max)
                        s_max.add_real_controller(c_min)

                PendList.remove(c_j_num)

        if PendList == old_PendList:
            break
        else:
            old_PendList = copy(PendList)

    for c_1, v_1 in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
        global_args["Des_last"][c_1] = global_args["Des_now"][c_1]

    return global_args


def prior_lbdc_cm(global_args):

    global_args["Des_now"] = {}
    global_args["ThdList"] = {}
    global_args["EfnList"] = {}

    for c_1, v_1 in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
        global_args["Des_now"][c_1] = sum([v.get_weight() for c, v in
                                    global_args["TOTAL_SWITCH_SET"].get_switch_set().iteritems()]) / \
                               float(sum([k.get_max_load() for j, k in
                                          global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems()])) *\
                                      v_1.get_max_load()

        global_args["ThdList"][c_1] = global_args["alpha"] * global_args["Des_now"][c_1] + (1 - global_args["alpha"]) * \
                                                                                   global_args["Des_last"][c_1]
        global_args["EfnList"][c_1] = global_args["beta"] * global_args["ThdList"][c_1]

    PendList = set({})
    old_PendList = set({})
    OverList = set({})

    # Step 1
    for num, controller in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
        if controller.get_controller_weight() > global_args["EfnList"][num]:
            OverList.add(num)

    if len(OverList) == 0:
        return global_args

    # Step 3
    while True:

        while len(OverList) > 0:

            # Step 2

            c_m = global_args["TOTAL_CONTROLLER_SET"].get_controller(
                max([x for x in OverList],
                    key=lambda q: global_args["TOTAL_CONTROLLER_SET"].get_controller_weight(q) / float(global_args["TOTAL_CONTROLLER_SET"].get_max_load(q))))

            c_m_an = [x for x, v in c_m.get_AN().iteritems() if v.get_controller_weight() < global_args["ThdList"][x]]

            # No c_n for this c_m
            if len([1 for x in c_m_an if abs(global_args["TOTAL_CONTROLLER_SET"].get_controller_weight(x) -
                                                     global_args["ThdList"][x]) < 0.001 or
                            global_args["TOTAL_CONTROLLER_SET"].get_controller_weight(x) > global_args["ThdList"][x]]) == 0:
                OverList.remove(c_m.get_num())
                PendList.add(c_m.get_num())
            else:
                flag = True

                c_m_ps_refer_num = set({})

                for i, v in c_m.get_PS().iteritems():
                    c_m_ps_refer_num.add(i)

                while flag:

                    s_k = 0

                    # c_f exists for this c_m
                    if len([1 for v in c_m_an if global_args["TOTAL_CONTROLLER_SET"].get_controller_weight(v) <
                            global_args["ThdList"][v]]) > 0:
                        c_f = c_m_an[0]

                        # break tie
                        s_k_list = [o for o in c_m_ps_refer_num if global_args["Value"][o][c_f] ==
                                    max([k for k in c_m_ps_refer_num],
                                        key=lambda u: global_args["Value"][u][c_f]) and
                                    global_args["TOTAL_SWITCH_SET"].get_switch(o).has_potential_controller(c_f)]
                        if len(s_k_list) > 1:
                            s_k_weight_list = [o for o in s_k_list if global_args["TOTAL_SWITCH_SET"].get_switch_weight(o) ==
                                               max([o for o in s_k_list],
                                                   key=lambda u:global_args["TOTAL_SWITCH_SET"].get_switch_weight(u))]
                            if len(s_k_weight_list) > 1:
                                s_k_ps_list = [o for o in s_k_weight_list if
                                               len(global_args["TOTAL_SWITCH_SET"].get_switch(o).get_PC()) ==
                                               min([o for o in s_k_weight_list],
                                                   key=lambda u:len(global_args["TOTAL_SWITCH_SET"].get_switch(u).get_PC()))]

                                s_k = global_args["TOTAL_SWITCH_SET"].get_switch(s_k_ps_list[0])
                            else:
                                s_k = global_args["TOTAL_SWITCH_SET"].get_switch(s_k_weight_list[0])
                        elif len(s_k_list) == 0:
                            c_m_an.remove(c_f)
                        else:
                            s_k = global_args["TOTAL_SWITCH_SET"].get_switch(s_k_list[0])

                        # Assign s_k to c_f
                        s_k.remove_real_controller(c_m)
                        c_m.remove_real_switch(s_k)
                        global_args["TOTAL_CONTROLLER_SET"].get_controller(c_f).add_real_switch(s_k)
                        s_k.add_real_controller(global_args["TOTAL_CONTROLLER_SET"].get_controller(c_f))

                    else:
                        # all w(c_f) >= Thd_f
                        flag = False

                    # w'(c_m) <= Thd_m
                    if c_m.get_controller_weight() < global_args["ThdList"][c_m.get_num()] \
                            or abs(c_m.get_controller_weight() - global_args["ThdList"][c_m.get_num()]) < 0.001:
                        flag = False

                if c_m.get_controller_weight() > global_args["EfnList"][c_m.get_num()]:
                    OverList.remove(c_m.get_num())
                    PendList.add(c_m.get_num())
                else:
                    OverList.remove(c_m.get_num())

        for item in PendList:
            OverList.add(item)

        if PendList == old_PendList:
            break
        else:
            old_PendList = copy(PendList)
            PendList = set({})

    # Step 5
    while True:

        # Step 4
        CC_total = CC_set(PendList)

        for each_CC in CC_total.get_CC_set():

            total_an = {}
            for each_con_num in each_CC:
                for an_num, an_con in global_args["TOTAL_CONTROLLER_SET"].get_controller(each_con_num).get_AN().iteritems():
                    if an_num not in total_an and an_num not in each_CC:
                        total_an[an_num] = an_con

            cc_weight = sum([global_args["TOTAL_CONTROLLER_SET"].get_controller_weight(x) for x in each_CC])
            an_weight = 0

            load_sum_cc = sum([global_args["TOTAL_CONTROLLER_SET"].get_controller(v).get_max_load() for v in each_CC])
            load_sum_an = 0

            if len(total_an) > 0:
                an_weight = sum([k.get_controller_weight() for v, k in total_an.iteritems()])
                load_sum_an = sum([v.get_max_load() for k, v in total_an.iteritems()])

            e_local = (cc_weight+an_weight) / float(load_sum_an + load_sum_cc)

            for c_j_num in each_CC:

                c_j = global_args["TOTAL_CONTROLLER_SET"].get_controller(c_j_num)

                if len(c_j.get_RS()):

                    while (c_j.get_controller_weight() > global_args["gama"] * e_local * c_j.get_max_load()
                           or abs(c_j.get_controller_weight() - global_args["gama"] * e_local * c_j.get_max_load()) < 0.001) \
                            and len(total_an) > 0:

                        s_max = max([v for k, v in c_j.get_RS().iteritems()], key=lambda o: o.get_weight())
                        c_min = min([v for k, v in total_an.iteritems()], key=lambda o: o.get_controller_weight())

                        s_max.remove_real_controller(c_j)
                        c_j.remove_real_switch(s_max)
                        c_min.add_real_switch(s_max)
                        s_max.add_real_controller(c_min)

                PendList.remove(c_j_num)

        if PendList == old_PendList:
            break
        else:
            old_PendList = copy(PendList)

    for c_1, v_1 in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
        global_args["Des_last"][c_1] = global_args["Des_now"][c_1]

    return global_args


# Distributed Migration
def lbdc_di():

    global_args = {}

    global_args["TOTAL_SWITCH"] = copy(Global.TOTAL_SWITCH)
    global_args["TOTAL_CONTROLLER"] = copy(Global.TOTAL_CONTROLLER)
    global_args["TOTAL_CONTROLLER_SET"] = copy(Global.TOTAL_CONTROLLER_SET)
    global_args["TOTAL_SWITCH_SET"] = copy(Global.TOTAL_SWITCH_SET)
    global_args["alpha"] = copy(Global.alpha)
    global_args["beta"] = copy(Global.beta)
    global_args["gama"] = copy(Global.gama)
    global_args["Avg_now"] = copy(Global.Avg_now)
    global_args["Avg_last"] = copy(Global.Avg_last)
    global_args["Thd"] = copy(Global.Thd)
    global_args["Efn"] = copy(Global.Efn)
    global_args["EfnList"] = copy(Global.EfnList)
    global_args["ThdList"] = copy(Global.ThdList)
    global_args["Des_now"] = copy(Global.Des_now)
    global_args["Des_last"] = copy(Global.Des_last)
    global_args["Value"] = copy(Global.Value)

    # clear states
    for num, controller in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
        controller.clear_state()

    for num, switch in global_args["TOTAL_SWITCH_SET"].get_switch_set().iteritems():
        switch.clear_state()
        real = global_args["TOTAL_CONTROLLER_SET"].get_controller(random.choice([i for i in switch.get_PC()]))
        switch.add_real_controller(real)
        real.add_real_switch(switch)

    # Initial state
    for num, controller in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
        controller.set_avg_last((sum([c.get_controller_weight() for k, c in
                                     controller.get_AN().iteritems()]) + controller.get_controller_weight())
                                / float(len(controller.get_AN()) + 1))

    return global_args


def lbdc_dm(global_args):

    # Calculate Avg, Efn and Thd
    global_args["ThdList"] = {}
    global_args["EfnList"] = {}

    for num, controller in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
        controller.set_avg_now((sum([c.get_controller_weight() for k, c in
                                     controller.get_AN().iteritems()]) + controller.get_controller_weight())
                                / float(len(controller.get_AN()) + 1))
        global_args["ThdList"][num] = global_args["alpha"] * controller.get_avg_now() + \
                                      (1 - global_args["alpha"]) * controller.get_avg_last()
        global_args["EfnList"][num] = global_args["beta"] * global_args["ThdList"][num]

    total_sending = 0

    # Initiating Mode
    for num, controller in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():

        controller.get_RList().clear()

        controller.set_mode(global_args)

        if controller.get_mode() == "Sending":
            total_sending += 1

    #print "Sending: %d/%d" % (total_sending, global_args["TOTAL_CONTROLLER"])

    # Start migration
    while total_sending > 0:

        for num, controller in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():

            if controller.get_mode() == "Sending":

                # Add c_i to RList
                for num_1, controller_1 in controller.get_AN().iteritems():
                    if controller_1.get_mode() == "Receiving" or controller_1.get_mode() == "Idle":
                        controller.get_RList()[num_1] = controller_1

                #print "RList:", [k for k in controller.get_RList()]

                while controller.get_controller_weight() > global_args["EfnList"][num]:

                    # Pick s_max
                    s_max = max([v for k, v in controller.get_RS().iteritems()], key=lambda i: i.get_weight())

                    #print "--s_max: %d" % s_max.get_num()
                    #print "--s_max_rc:", [i for i in s_max.get_rc()]
                    #print "--controller: %d" % num
                    #print "--s_max in con.RS:", s_max.get_num() in [i for i in controller.get_RS()]

                    # send out one c_j successfully
                    succeed = False
                    while not succeed:

                        controller.refresh_RList()

                        # find c_j
                        min_list = [v for k, v in controller.get_RList().iteritems() if k in s_max.get_PC()]

                        #print "min_list:", [k.get_num() for k in min_list]

                        c_j = 0

                        # no c_j available
                        if len(min_list) == 0:
                            second_min_list = [v for k, v in s_max.get_PC().iteritems() if k in controller.get_AN()]

                            c_j = min(second_min_list, key=lambda i: i.get_controller_weight())

                            s_max.remove_real_controller(controller)
                            controller.remove_real_switch(s_max)

                            c_j.add_real_switch(s_max)
                            s_max.add_real_controller(c_j)

                            c_j.set_mode(global_args)
                            controller.set_mode(global_args)

                            # c_j send CONFIRM
                            succeed = True
                        else:
                            c_j = min(min_list, key=lambda i: i.get_controller_weight())
                            c_j.set_mode(global_args)

                            # C_i send HELP[c_i, s_max] ot c_j
                            if c_j.get_mode() == "Receiving":
                                #print "Receiving"
                                if c_j.get_controller_weight() < global_args["ThdList"][c_j.get_num()]:
                                    # c_j return ACC
                                    # c_i send MIG[c_i, s_max] ot c_j

                                    #print "s_max: %d" % s_max.get_num()
                                    #print "s_max_rc:", [i for i in s_max.get_rc()]
                                    #print "controller: %d" % num
                                    #print "s_max in con.RS:", s_max.get_num() in [i for i in controller.get_RS()]

                                    s_max.remove_real_controller(controller)
                                    controller.remove_real_switch(s_max)

                                    c_j.add_real_switch(s_max)
                                    s_max.add_real_controller(c_j)

                                    c_j.set_mode(global_args)
                                    controller.set_mode(global_args)

                                    # c_j send CONFIRM
                                    succeed = True
                                else:
                                    # return REJ
                                    # remove s_max from RList
                                    del controller.get_RList()[c_j.get_num()]
                                    succeed = False

                            else:
                                #print "Idle"
                                #print "Efn:%f, weight:%f" % (global_args["EfnList"][c_j.get_num()], c_j.get_controller_weight())
                                if c_j.get_controller_weight() < global_args["EfnList"][c_j.get_num()] or \
                                    abs(c_j.get_controller_weight() - global_args["EfnList"][c_j.get_num()]) < 0.001:

                                    #print "s_max: %d" % s_max.get_num()
                                    #print "s_max.rc:", [i for i in s_max.get_rc()]
                                    #print "controller: %d" % num
                                    #print "s_max in con.RS:", s_max.get_num() in [i for i in controller.get_RS()]

                                    # c_j return ACC
                                    s_max.remove_real_controller(controller)
                                    controller.remove_real_switch(s_max)

                                    c_j.add_real_switch(s_max)
                                    s_max.add_real_controller(c_j)

                                    c_j.set_mode(global_args)
                                    controller.set_mode(global_args)

                                    succeed = True

                total_sending -= 1
                #break

    for num, controller in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
        controller.set_avg_last(controller.get_avg_now())

    return global_args







