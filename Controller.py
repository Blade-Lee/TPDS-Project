class Controller(object):

    def __init__(self, weight=0):
        self.weight = weight
        self.PS = []
        self.RS = []

    def get_weight(self):
        return self.weight
