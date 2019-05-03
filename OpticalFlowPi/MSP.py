from pymultiwii import MultiWii

serialPort = "/dev/ttyACM0"
board = MultiWii(serialPort)
while True:
    print(board.getData(MultiWii.ATTITUDE))