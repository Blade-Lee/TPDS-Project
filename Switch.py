
class Switch(object):

    def __init__(self, weight=0):
        self.weight = weight
        self.PC = []
        self.rc = []

    def get_weight(self):
        return self.weight

    def set_weight(self, weight):
        self.weight = weight
