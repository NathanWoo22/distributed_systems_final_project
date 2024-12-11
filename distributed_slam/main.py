import RoboNode
import time
import sys


bot = int(sys.argv[1])
file = f"seq{bot}_points.txt"
#file = f"test{bot}.txt"
test = RoboNode.RoboNode()
test.startUp(file,bot,[("localhost",50000),("localhost",50001),("localhost",50002),("localhost",50003)], 1,1)
if bot == 0:
    test.comChange(1, False)
    test.comChange(2, False)
elif bot == 1:
    test.comChange(3, False)
while not test.wrapped:
    time.sleep(0.1)
for i in test.dataStorage:
    print(f"key {i} : value size: {len(test.dataStorage[i])}")