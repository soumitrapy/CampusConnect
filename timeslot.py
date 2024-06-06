class Order:
    def __init__(self, name, prefs) -> None:
        self.name = name
        self.prefs = []
        for pref in prefs:
            self.add_preference(pref)
    

    def add_preference(self, pref): # perf = (time, place)
        if len(self.prefs)==0:
            self.prefs.append(pref)
        else:
            pos = 0
            for prf in self.prefs:
                if prf[0]<pref[0]:
                    pos += 1
                else:
                    break
            self.prefs.insert(pos, pref)

    def __str__(self) -> str:
        return str(self.name)+':'+str(self.prefs)

def optimize_orders(orders):
    accepted = dict()
    for o in orders:
        for (tm, pl) in o.prefs:
            if tm in accepted:
                if accepted[tm] == pl:
                    accepted[tm] += 1
                    orders.remov
                


if __name__=='__main__':
    s1 = Order('s1',[(9,'a'),(10,'b'),(11,'c')])
    s2 = Order('s2',[(12,'c'),(11,'a')])
    s3 = Order('s3',[(9,'a'), (12,'c'),(10,'b'),(13,'d')])
    #print(s1, s2)
    orders = set({s1, s2, s3})
    print(len(orders))
    for o in orders:
        print(o)

        


        