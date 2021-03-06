import matplotlib.pyplot as chart
import serial
import time
import numpy as np

board = serial.Serial(port = '/dev/tty.usbserial-DN02136Z', baudrate = 57600, timeout = 3)
time.sleep(3)

angleData = open('anglesData.txt', 'w')
PIDData = open('PIDData.txt', 'w')

def resetMenu(option):

	print("Please choose one of the following options to view:")
	print("1. Angle Data")
	print("2. PID Output")
	print("3. PID vs Angles")
	print("4. Gains")
	print("Press Ctrl + C to exit data streaming")
	option = raw_input("Choose your option: ")
	
	board.write('send')
	board.write(option)
	
	return option

def readData():
	while board.in_waiting < 1:
		pass
	data = board.readline()
	board.reset_input_buffer
	return data

def getPIDOutputs(data, PIDData):
	data = readData()
	rMark = data.find('r')
	pMark = data.find('p')
	yMark = data.find('y')
	sMark = data.find('s')
	oMark = data.find('o')
		
	checksum = [0, 0, 0]

	if sMark == 0:
		if oMark == 1:
			if (rMark - oMark) - 1 > 7:  
				rollPID = data[oMark+1:rMark]
				checksum[0] = 1

			if (pMark - rMark) - 1 > 7:
				pitchPID = data[rMark+1:pMark]
				checksum[1] = 1
			
			if (yMark - pMark) - 1 > 7:
				yawPID = (data[pMark+1:yMark])
				checksum[2] = 1
	
			if sum(checksum) == 3:
					PIDData.write(rollPID + ',' + pitchPID + ',' + yawPID + '\n')
					print([rollPID, pitchPID, yawPID])

def getAngles(data, angleData):
	data = readData()
	rMark = data.find('r')
	pMark = data.find('p')
	yMark = data.find('y')
	sMark = data.find('s')
	aMark = data.find('a')
		
	checksum = [0, 0, 0]

	if sMark == 0:
		if aMark == 1:
			if (rMark - aMark) - 1 > 7:  
				rollAngle = data[aMark+1:rMark]
				checksum[0] = 1

			if (pMark - rMark) - 1 > 7:
				pitchAngle = data[rMark+1:pMark]
				checksum[1] = 1
			
			if (yMark - pMark) - 1 > 7:
				yawAngle = (data[pMark+1:yMark])
				checksum[2] = 1
	
			if sum(checksum) == 3:
					angleData.write(rollAngle + ',' + pitchAngle + ',' + yawAngle + '\n')
					print([rollAngle, pitchAngle, yawAngle])

def PIDPlot(PIDData):
	roll, pitch, yaw = np.loadtxt(PIDData, delimiter=',', unpack=True)
	outputs = [roll, pitch, yaw]

	PIDPlot = chart.figure()
	PIDPlots = []

	for i in range(0,3):
		PIDPlots.append(PIDPlot.add_subplot(3, 1, i+1))
		PIDPlots[i].plot(outputs[i])
		chart.grid()
	
	chart.show()


def anglePlot(angleData):
	roll, pitch, yaw = np.loadtxt(angleData, delimiter=',', unpack=True)
	angles = [roll, pitch, yaw]

	anglePlot = chart.figure()
	anglePlots = []

	for i in range(0,3):
		anglePlots.append(anglePlot.add_subplot(3, 1, i+1))
		anglePlots[i].plot(angles[i])
		chart.grid()
	
	chart.show()

option = 0
option = resetMenu(option)

while(1):
	if option == '1':
		try:
			while(1):
				getAngles(readData(), angleData)
		except KeyboardInterrupt:
			angleData.close()
			anglePlot('anglesData.txt')
			resetMenu(option)
#			board.write('nend')
	elif option == '2':
		try:
			while(1):
				getPIDOutputs(readData(), PIDData)
		except KeyboardInterrupt:
			PIDData.close()
			PIDPlot('anglesData.txt')
			resetMenu(option)

	else:
			break
