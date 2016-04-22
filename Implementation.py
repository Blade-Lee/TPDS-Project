from Algo import *
import json


# Implementation
def implement_lbdc_cm(global_args):

    print "--LBDC-CM:"

    total = [0, 0, 0]

    result = 0

    while True:

        result = lbdc_cm(global_args)

        old_total = total

        total = [0, 0, 0]
        for num, controller in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
            if controller.get_controller_weight() < global_args["Thd"] or \
                    abs(controller.get_controller_weight() - global_args["Thd"]) < 0.001:
                total[0] += 1
            elif global_args["Thd"] < controller.get_controller_weight() < global_args["Efn"]:
                total[1] += 1
            elif controller.get_controller_weight() > global_args["Efn"] or \
                    abs(controller.get_controller_weight() - global_args["Efn"]) < 0.001:
                total[2] += 1
            else:
                print "Thd: %f, Efn: %f, weight:%f" % (global_args["Thd"],
                                                       global_args["Efn"], controller.get_controller_weight())

        if old_total[2] >= total[2]:
            break

    print "Thd:%f, Efn:%f" % (global_args["Thd"], global_args["Efn"])
    print total
    print "--End\n"
    return result


def implement_limited_lbdc_cm(global_args):
    print "--Limited LBDC-CM:"

    total = [0, 0, 0]
    result = 0

    while True:

        result = limited_lbdc_cm(global_args)

        old_total = total

        total = [0, 0, 0]
        for num, controller in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
            if controller.get_controller_weight() < global_args["ThdList"][num] or \
                    abs(controller.get_controller_weight() - global_args["ThdList"][num]) < 0.001:
                total[0] += 1
            elif global_args["ThdList"][num] < controller.get_controller_weight() < global_args["EfnList"][num]:
                total[1] += 1
            elif controller.get_controller_weight() > global_args["EfnList"][num] or \
                    abs(controller.get_controller_weight() - global_args["EfnList"][num]) < 0.001:
                total[2] += 1
            else:
                print "Thd: %f, Efn: %f, weight:%f" % (global_args["ThdList"][num],
                                                       global_args["EfnList"][num], controller.get_controller_weight())

        if old_total[2] >= total[2]:
            break

    print total
    print "--End\n"
    return result


def implement_prior_lbdc_cm(global_args):

    print "--Prior LBDC-CM:"

    total = [0, 0, 0]
    result = 0

    while True:

        result = prior_lbdc_cm(global_args)

        old_total = total

        total = [0, 0, 0]
        for num, controller in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
            if controller.get_controller_weight() < global_args["ThdList"][num] or \
                    abs(controller.get_controller_weight() - global_args["ThdList"][num]) < 0.001:
                total[0] += 1
            elif global_args["ThdList"][num] < controller.get_controller_weight() < global_args["EfnList"][num]:
                total[1] += 1
            elif controller.get_controller_weight() > global_args["EfnList"][num] or \
                    abs(controller.get_controller_weight() - global_args["EfnList"][num]) < 0.001:
                total[2] += 1
            else:
                print "Thd: %f, Efn: %f, weight:%f" % (global_args["ThdList"][num],
                                                       global_args["EfnList"][num], controller.get_controller_weight())

        if old_total[2] >= total[2]:
            break

    print total
    print "--End\n"
    return result


def implement_lbdc_dm(global_args):

    print "--LBDC-DM:"

    total = [0, 0, 0]
    result = 0

    while True:

        result = lbdc_dm(global_args)

        old_total = total

        total = [0, 0, 0]
        for num, controller in global_args["TOTAL_CONTROLLER_SET"].get_controller_set().iteritems():
            if controller.get_controller_weight() < global_args["ThdList"][num] or \
                    abs(controller.get_controller_weight() - global_args["ThdList"][num]) < 0.001:
                total[0] += 1
            elif global_args["ThdList"][num] < controller.get_controller_weight() < global_args["EfnList"][num]:
                total[1] += 1
            elif controller.get_controller_weight() > global_args["EfnList"][num] or \
                    abs(controller.get_controller_weight() - global_args["EfnList"][num]) < 0.001:
                total[2] += 1
            else:
                print "Thd: %f, Efn: %f, weight:%f" % (global_args["ThdList"][num],
                                                       global_args["EfnList"][num], controller.get_controller_weight())

        if old_total[2] >= total[2]:
            break

    print total
    print "--End\n"
    return result


# Output data
def output_lbdc_cm(start, end, step):

    result_x = []
    initial_result_y = []
    migration_result_y = []
    improve_result_y = []

    for con_num in range(start, end+10, step):

        square_initial(con_num, 0.7, 1.5, 1.3)

        global_args_original = lbdc_ci()

        rwd_1 = RWD(global_args_original, 4)

        global_args_temp = implement_lbdc_cm(global_args_original)

        result_x.append(con_num)

        rwd_2 = RWD(global_args_temp, 0)

        initial_result_y.append(rwd_1)
        migration_result_y.append(rwd_2)
        improve_result_y.append((rwd_1 - rwd_2) / float(rwd_1) * 100)

    with open('lbdc_cm.txt', 'w') as output_file:
        json.dump([result_x, initial_result_y, migration_result_y, improve_result_y], output_file)


def output_limited_lbdc_cm(start, end, step):

    result_x = []
    initial_result_y = []
    migration_result_y = []
    improve_result_y = []

    for con_num in range(start, end+10, step):

        square_initial(con_num, 0.7, 1.5, 1.3)

        global_args_original = lbdc_ci()

        rwd_1 = RWD(global_args_original, 4)

        global_args_temp = implement_limited_lbdc_cm(global_args_original)

        result_x.append(con_num)

        rwd_2 = RWD(global_args_temp, 1)

        initial_result_y.append(rwd_1)
        migration_result_y.append(rwd_2)
        improve_result_y.append((rwd_1 - rwd_2) / float(rwd_1) * 100)

    with open('limited_lbdc_cm.txt', 'w') as output_file:
        json.dump([result_x, initial_result_y, migration_result_y, improve_result_y], output_file)


def output_prior_lbdc_cm(start, end, step):

    result_x = []
    initial_result_y = []
    migration_result_y = []
    improve_result_y = []

    for con_num in range(start, end+10, step):

        square_initial(con_num, 0.7, 1.5, 1.3)

        global_args_original = lbdc_ci()

        rwd_1 = RWD(global_args_original, 4)

        global_args_temp = implement_prior_lbdc_cm(global_args_original)

        result_x.append(con_num)

        rwd_2 = RWD(global_args_temp, 2)

        initial_result_y.append(rwd_1)
        migration_result_y.append(rwd_2)
        improve_result_y.append((rwd_1 - rwd_2) / float(rwd_1) * 100)

    with open('prior_lbdc_cm.txt', 'w') as output_file:
        json.dump([result_x, initial_result_y, migration_result_y, improve_result_y], output_file)


def output_lbdc_dm(start, end, step):

    result_x = []
    initial_result_y = []
    migration_result_y = []
    improve_result_y = []

    for con_num in range(start, end + 10, step):

        square_initial(con_num, 0.7, 1.5, 1.3)

        global_args_original = lbdc_di()

        rwd_1 = RWD(global_args_original, 4)

        global_args_temp = implement_lbdc_dm(global_args_original)

        result_x.append(con_num)

        rwd_2 = RWD(global_args_temp, 3)

        initial_result_y.append(rwd_1)
        migration_result_y.append(rwd_2)
        improve_result_y.append((rwd_1 - rwd_2) / float(rwd_1) * 100)

    with open('lbdc_dm.txt', 'w') as output_file:
        json.dump([result_x, initial_result_y, migration_result_y, improve_result_y], output_file)


def output_total(start, end, step):

    result_x = []
    # initial_result_y = []
    l1 = []

    # migration_result_y_naive = []
    l2 = []
    # improve_result_y_naive = []
    l3 = []

    # migration_result_y_limited = []
    l4 = []
    # improve_result_y_limited = []
    l5 = []

    # migration_result_y_prior = []
    l6 = []
    # improve_result_y_prior = []
    l7 = []

    #migration_result_y_dm = []
    l8 = []
    #improve_result_y_dm = []
    l9 = []

    for con_num in range(start, end+10, step):

        square_initial(con_num, 0.7, 1.5, 1.3)

        result_x.append(con_num)

        global_args_original = lbdc_ci()

        rwd_1 = RWD(global_args_original, 4)
        l1.append(rwd_1)

        # naive lbdc-cm
        rwd_2 = RWD(implement_lbdc_cm(copy_global_args(global_args_original)), 0)
        l2.append(rwd_2)
        l3.append((rwd_1 - rwd_2) / float(rwd_1) * 100)

        # limited lbdc-cm
        rwd_2 = RWD(implement_limited_lbdc_cm(copy_global_args(global_args_original)), 1)
        l4.append(rwd_2)
        l5.append((rwd_1 - rwd_2) / float(rwd_1) * 100)

        # prior lbdc-cm
        rwd_2 = RWD(implement_prior_lbdc_cm(copy_global_args(global_args_original)), 2)
        l6.append(rwd_2)
        l7.append((rwd_1 - rwd_2) / float(rwd_1) * 100)

        # lbdc-dm
        rwd_2 = RWD(implement_lbdc_dm(copy_global_args(global_args_original)), 3)
        l8.append(rwd_2)
        l9.append((rwd_1 - rwd_2) / float(rwd_1) * 100)

    with open('total.txt', 'w') as output_file:
        json.dump([result_x, l1, l2, l3, l4, l5, l6, l7, l8, l9], output_file)


def test(start, end, step):

    result_x = []

    list_1 = []
    list_2 = []
    list_3 = []

    for con_num in range(start, end+10, step):

        square_initial(con_num, 0.7, 1.5, 1.3)

        global_args_original = lbdc_ci()

        global_args_1 = copy_global_args(global_args_original)
        list_1.append(RWD(global_args_1, 4))
        implement_lbdc_cm(global_args_1)

        global_args_2 = copy_global_args(global_args_original)
        list_2.append(RWD(global_args_2, 4))
        implement_limited_lbdc_cm(global_args_2)

        global_args_3 = copy_global_args(global_args_original)
        list_3.append(RWD(global_args_3, 4))
        implement_prior_lbdc_cm(global_args_3)

        result_x.append(con_num)

    with open('test.txt', 'w') as output_file:
        json.dump([result_x, list_1, list_2, list_3], output_file)


# Draw
def draw_con_num(algo, start, end, step):

    json_object = 0

    with open(algo+".txt", 'r') as input_file:
        json_object = json.load(input_file)

    result_x = json_object[0]
    initial_result_y = json_object[1]
    migration_result_y = json_object[2]
    improve_result_y = json_object[3]

    fig = plt.figure(figsize=(8, 4))
    ax1 = fig.add_subplot(111)
    ax2 = ax1.twinx()

    line_1, = ax1.plot(result_x, initial_result_y)
    line_2, = ax1.plot(result_x, migration_result_y)
    line_3, = ax2.plot(result_x, improve_result_y)

    plt.setp(line_1, color='r', linewidth=1.0, aa=True, marker='^', ms=9.0, ls='--')
    plt.setp(line_2, color='g', linewidth=1.0, aa=True, marker='s', ms=9.0, ls='--')
    plt.setp(line_3, color='b', linewidth=1.0, aa=True, marker='o', ms=9.0, ls='--')

    ax1.set_xlim(start-10, end+10)
    ax1.set_xticks(np.arange(start, end+10, step))
    ax2.set_xticks(np.arange(start, end+10, step))

    ax1.set_ylim(0, 1000)
    ax1.set_yticks(np.arange(0, 1100, 100))

    ax2.set_ylim(0, 100)
    ax2.set_yticks(np.arange(0, 110, 10))

    ax1.set_xlabel(r"Controller #", fontsize=20)
    ax1.set_ylabel(r"Relative Weight Deviation", fontsize=20)
    ax2.set_ylabel(r"Improvement ($\%$)", fontsize=20)

    plt.legend((line_1, line_2, line_3), ("Initial State", "After Migration", "Improvement"), loc='right', numpoints=1)

    plt.gcf().tight_layout()
    plt.savefig(algo+'.eps', format='eps')


def draw_total_con_num(start, end, step):

    json_object = 0

    with open(algo+".txt", 'r') as input_file:
        json_object = json.load(input_file)

    result_x = json_object[0]
    initial_result_y = json_object[1]
    migration_result_y = json_object[2]
    improve_result_y = json_object[3]

    fig = plt.figure(figsize=(8, 4))
    ax1 = fig.add_subplot(111)
    ax2 = ax1.twinx()

    line_1, = ax1.plot(result_x, initial_result_y)
    line_2, = ax1.plot(result_x, migration_result_y)
    line_3, = ax2.plot(result_x, improve_result_y)

    plt.setp(line_1, color='r', linewidth=1.0, aa=True, marker='^', ms=9.0, ls='--')
    plt.setp(line_2, color='g', linewidth=1.0, aa=True, marker='s', ms=9.0, ls='--')
    plt.setp(line_3, color='b', linewidth=1.0, aa=True, marker='o', ms=9.0, ls='--')

    ax1.set_xlim(start-10, end+10)
    ax1.set_xticks(np.arange(start, end+10, step))
    ax2.set_xticks(np.arange(start, end+10, step))

    ax1.set_ylim(0, 1000)
    ax1.set_yticks(np.arange(0, 1100, 100))

    ax2.set_ylim(0, 100)
    ax2.set_yticks(np.arange(0, 110, 10))

    ax1.set_xlabel(r"Controller #", fontsize=20)
    ax1.set_ylabel(r"Relative Weight Deviation", fontsize=20)
    ax2.set_ylabel(r"Improvement ($\%$)", fontsize=20)

    plt.legend((line_1, line_2, line_3), ("Initial State", "After Migration", "Improvement"), loc='right', numpoints=1)

    plt.gcf().tight_layout()
    plt.savefig(algo+'.eps', format='eps')


def draw_comp(start, end, step):

    cm_result_y = []
    limit_result_y = []
    prior_result_y = []
    dm_result_y = []

    for con_num in range(start, end + 10, step):

        initial(400, 10000, con_num, 0.7, 1.5, 1.3)

        cm_result_y.append(RWD(implement_lbdc_cm(lbdc_ci()), 0))
        limit_result_y.append(RWD(implement_limited_lbdc_cm(lbdc_ci()), 1))
        prior_result_y.append(RWD(implement_prior_lbdc_cm(lbdc_ci()), 2))
        dm_result_y.append(RWD(implement_lbdc_dm(lbdc_di()), 3))

    fig, ax = plt.subplots()
    index = np.arange(10)
    bar_width = 0.2

    rect1 = plt.bar(index, cm_result_y, bar_width, color='r')
    rect2 = plt.bar(index + bar_width, limit_result_y, bar_width, color='g')
    rect3 = plt.bar(index + 2 * bar_width, prior_result_y, bar_width, color='b')
    rect4 = plt.bar(index + 3 * bar_width, dm_result_y, bar_width, color='c')

    ax.set_xlabel(r"Controller #", fontsize=22)
    ax.set_ylabel(r"$RWD$", fontsize=22)
    ax.set_xticks(index + 2 * bar_width)
    ax.set_xticklabels(np.arange(start, end + 10, step), fontsize=18)
    plt.yticks(fontsize=18)

    plt.legend((rect1[0], rect2[0], rect3[0], rect4[0]),
               ("Naive LDBC-CM", "Limited LDBC-CM", "Priority LDBC-CM", "LDBC-DM"), loc='best', numpoints=1)

    plt.savefig('compare.eps', format='eps')


