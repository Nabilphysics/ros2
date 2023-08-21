class bengalChmap():
    name = 'Syed Taher'
    def __init__(self,a) -> None:
        self.x = a
        print("self.x= ",self.x)
    
    def printUsingExtFuntion(self):
        #a = printingFunction()
        
        self.x = self.x  + 1
        print(printingFunction('Nabil'))
    
    def anotherFunction(self):
        self.x = self.x + 1
    
    def changeName(self):
        name = 'Taha'
        print('Inside changName(self):',name)

def printingFunction(a):
    #return name + ' from printing function '
    #v = 5
    return a
a1 = bengalChmap(5)
a1.anotherFunction()
print('a1:',a1.x)
a1.printUsingExtFuntion()
print('a1:',a1.x)

a2 = bengalChmap(10)
a2.printUsingExtFuntion()
print('a2',a2.x)
a2.printUsingExtFuntion()
print('a2',a2.x)
print('a1:',a1.x)
print(a2.name)
print(a2.changeName())
print(a2.name)
print(a2.changeName())
#c = bengalChmap().printUsingExtFuntion()
