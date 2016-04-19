import Global


class CC_set(object):

    def __init__(self, controller_nums):
        self.CC_set = []

        # calculate CCs
        for num in controller_nums:

            flag = False

            new_set = set()

            for item in self.CC_set:
                # num already exists in one CC
                if num in item:
                    new_set = item
                    flag = True
                    break
            # num not in any CC
            if not flag:
                self.CC_set.append(new_set)
                new_set.add(num)

            controller = Global.TOTAL_CONTROLLER_SET.get_controller(num)

            for an in controller.get_AN():
                if an not in controller_nums:
                    continue

                flag = False
                for item in self.CC_set:
                    # an already exists in one CC
                    if an in item:
                        flag = True
                        break
                # an not in any CC
                if not flag:
                    new_set.add(an)

    def get_CC_set(self):
        return self.CC_set
