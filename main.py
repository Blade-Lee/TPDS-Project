from Controller import Controller
from Switch import Switch
import Global
from LinearProgramming import *
from Algo import *
from Tools import *
from Implementation import *


def main():

    #draw_con_num('lbdc_dm', 30, 210, 20)
    #output_total(30, 210, 20)
    #draw_total_con_num(30, 210, 20)

    #output_total_avg(30, 210, 20, 20)
    #output_total_improve(30, 210, 20, 20)
    draw_total_avg_con_num(30, 210, 20)
    draw_total_improve(30, 210, 20)

if __name__ == "__main__":
    main()

