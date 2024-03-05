class Test():
    def __init__(self, input):
        self.input = input
        
    def change_input(self, num):
        self.input = num
        
    def get_input(self):
        return self.input

ls = [1,2,3]
ob = Test(ls)
ls[1] = 4
print(ob.get_input())

