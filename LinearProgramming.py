from gurobipy import *
import Global


def lp_example():
    try:
        model = Model("mip1")

        # Create variables
        x = model.addVar(vtype=GRB.BINARY, name="x")
        y = model.addVar(vtype=GRB.BINARY, name="y")
        z = model.addVar(vtype=GRB.BINARY, name="z")

        # Integrate with new variables
        model.update()

        # Set objective: maximize x + y + 2 z
        # one way: model.setObjective(x + y + 2 * z, GRB.MAXIMIZE)
        # the other way:
        obj = LinExpr()
        obj += x
        obj += y
        obj += 2*z
        model.setObjective(obj, GRB.MAXIMIZE)

        # Add constraint
        model.addConstr(x + 2 * y + 3 * z <= 4, "c0")
        model.addConstr(x + y >= 1, "c1")

        # Optimize model
        model.optimize()

        for v in model.getVars():
            print v.varName, v.x

        print "obj:", model.objVal

    except GurobiError:
        print 'Error reported'


def lp_example_complex():
    # Model data

    commodities = ['Pencils', 'Pens']
    nodes = ['Detroit', 'Denver', 'Boston', 'New York', 'Seattle']

    arcs, capacity = multidict({
      ('Detroit', 'Boston'):   100,
      ('Detroit', 'New York'):  80,
      ('Detroit', 'Seattle'):  120,
      ('Denver',  'Boston'):   120,
      ('Denver',  'New York'): 120,
      ('Denver',  'Seattle'):  120})
    arcs = tuplelist(arcs)

    cost = {
      ('Pencils', 'Detroit', 'Boston'):   10,
      ('Pencils', 'Detroit', 'New York'): 20,
      ('Pencils', 'Detroit', 'Seattle'):  60,
      ('Pencils', 'Denver',  'Boston'):   40,
      ('Pencils', 'Denver',  'New York'): 40,
      ('Pencils', 'Denver',  'Seattle'):  30,
      ('Pens',    'Detroit', 'Boston'):   20,
      ('Pens',    'Detroit', 'New York'): 20,
      ('Pens',    'Detroit', 'Seattle'):  80,
      ('Pens',    'Denver',  'Boston'):   60,
      ('Pens',    'Denver',  'New York'): 70,
      ('Pens',    'Denver',  'Seattle'):  30 }

    inflow = {
      ('Pencils', 'Detroit'):   50,
      ('Pencils', 'Denver'):    60,
      ('Pencils', 'Boston'):   -50,
      ('Pencils', 'New York'): -50,
      ('Pencils', 'Seattle'):  -10,
      ('Pens',    'Detroit'):   60,
      ('Pens',    'Denver'):    40,
      ('Pens',    'Boston'):   -40,
      ('Pens',    'New York'): -30,
      ('Pens',    'Seattle'):  -30 }

    # Create optimization model
    m = Model('netflow')

    # Create variables
    flow = {}
    for h in commodities:
        for i,j in arcs:
            flow[h,i,j] = m.addVar(ub=capacity[i,j], obj=cost[h,i,j],
                                   name='flow_%s_%s_%s' % (h, i, j))
    m.update()

    # Arc capacity constraints
    for i,j in arcs:
        m.addConstr(quicksum(flow[h,i,j] for h in commodities) <= capacity[i,j],
                    'cap_%s_%s' % (i, j))

    # Flow conservation constraints
    for h in commodities:
        for j in nodes:
            m.addConstr(
              quicksum(flow[h,i,j] for i,j in arcs.select('*',j)) +
                  inflow[h,j] ==
              quicksum(flow[h,j,k] for j,k in arcs.select(j,'*')),
                       'node_%s_%s' % (h, j))

    # Compute optimal solution
    m.optimize()

    # Print solution
    if m.status == GRB.Status.OPTIMAL:
        for h in commodities:
            print '\nOptimal flows for', h, ':'
            for i,j in arcs:
                if flow[h,i,j].x > 0:
                    print i, '->', j, ':', flow[h,i,j].x


def lp(switch_set, controller_set):

    # Create model

    m = Model('LBDC')

    # Add variables

    y = {}
    x = {}

    for i in range(1, Global.TOTAL_CONTROLLER + 1):
        y[i] = m.addVar(name="y_%d" % i)
        for j in range(1, Global.TOTAL_SWITCH + 1):
            x[i, j] = m.addVar(name="x_%d_%d" % (i, j))

    m.update()

    # Set Objective Function

    m.setObjective(quicksum([y[key] for key in y]) / float(Global.TOTAL_CONTROLLER), GRB.MINIMIZE)

    w_c_avg = LinExpr(quicksum([switch_set.get_switch_weight(j) * x[i, j] for j in range(1, Global.TOTAL_SWITCH + 1)
                                for i in range(1, Global.TOTAL_CONTROLLER + 1)]) / float(Global.TOTAL_CONTROLLER))

    # Add constraints

    for i in range(1, Global.TOTAL_CONTROLLER + 1):
        m.addConstr(y[i] >= quicksum([switch_set.get_switch_weight(j) *
                                      x[i, j] for j in range(1, Global.TOTAL_SWITCH + 1)]) - w_c_avg, name="l2_%d" % i)

    for i in range(1, Global.TOTAL_CONTROLLER + 1):
        m.addConstr(y[i] >= w_c_avg - quicksum([switch_set.get_switch_weight(j) *
                                                x[i, j] for j in range(1, Global.TOTAL_SWITCH + 1)]), name="l3_%d" % i)

    for j in range(1, Global.TOTAL_SWITCH + 1):
        m.addConstr(quicksum([x[i, j] for i in range(1, Global.TOTAL_CONTROLLER + 1)]) == 1, name="l4_%d" % j)

    for i in range(1, Global.TOTAL_CONTROLLER + 1):
        for j in range(1, Global.TOTAL_SWITCH + 1):
            if not controller_set.get_controller(i).has_potential_switch(j):
                m.addConstr(x[i, j] == 0, name="l5_%d_%d" % (i, j))
            elif switch_set.get_switch(j).has_potential_controller(i):
                m.addConstr(x[i, j] == 0, name="l5_%d_%d" % (i, j))

    for i in range(1, Global.TOTAL_CONTROLLER + 1):
        for j in range(1, Global.TOTAL_SWITCH + 1):
            m.addConstr(x[i, j] >= 0, name="l6_%d_%d" % (i, j))

    # Get the result

    m.optimize()

    if m.status == GRB.Status.OPTIMAL:
        return x
