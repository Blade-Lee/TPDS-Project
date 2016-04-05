from Controller import Controller
from Switch import Switch


def main():
    c_1 = Controller()
    c_2 = Controller()
    s_1 = Switch(3)
    s_2 = Switch(5)

    print c_1.get_weight(), c_2.get_weight(), s_1.get_weight(), s_2.get_weight()

if __name__ == "__main__":
    main()

