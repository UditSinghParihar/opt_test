# Usage : python top.py poses_trajectory.txt loopClosures.txt


from sys import argv
import matplotlib.pyplot as plt
import math
import numpy as np
import os


def read(fileName):
	f = open(fileName, 'r')
	A = f.readlines()
	f.close()

	X = []
	Y = []
	THETA = []

	for line in A:
		(x, y, z, thetaX, thetaY, thetaZ) = line.split(' ')
		X.append(float(x))
		Y.append(float(y))
		THETA.append(float(thetaZ.rstrip('\n')))

	return (X, Y, THETA)


def draw(X, Y, THETA, title="Figure", thetaPlot=True):
	fig = plt.figure()

	if(thetaPlot == True):
		for i in range(len(THETA)):
			x2 = 0.5*math.cos(THETA[i]) + X[i]
			y2 = 0.5*math.sin(THETA[i]) + Y[i]
			plt.plot([X[i], x2], [Y[i], y2], 'b->', markersize=5)

	plt.plot(X, Y, 'ro', markersize=5)
	plt.plot(X, Y, 'k-', markersize=2)

	plt.title(title)
	fig.canvas.set_window_title(title)

	plt.show()


def calcTheta(x1, x2, y1, y2):
	if(x2 == x1):
		if(y2 > y1):
			theta = math.pi/2
		else:
			theta = 3*math.pi/2
	else:
		theta = math.atan((y2-y1)/(x2-x1))

	if(x2-x1 < 0):
		theta += math.pi

	return theta


def getTheta(X, Y):
	THETA = []

	for i in range(1, len(X)):
		theta = calcTheta(X[i-1], X[i], Y[i-1], Y[i])
		THETA.append(theta)

	THETA.append(THETA[-1])

	return THETA


def drawId(X, Y):
	fig = plt.figure()
	ax = plt.subplot(111)

	def on_plot_hover(event):
		for pt in ax.get_lines():
			if pt.contains(event)[0]:
				print("over %s" % pt.get_gid())

	for i in range(len(X)):
		ax.plot(X[i], Y[i], 'ro', markersize=3, gid=i)

	fig.canvas.mpl_connect('motion_notify_event', on_plot_hover)

	plt.show()


def addNoise(X, Y, THETA):
	xN = np.zeros(len(X)); yN = np.zeros(len(Y)); tN = np.zeros(len(THETA))
	xN[0] = X[0]; yN[0] = Y[0]; tN[0] = THETA[0]

	for i in range(1, len(X)):
		# Get T2_1
		p1 = (X[i-1], Y[i-1], THETA[i-1])
		p2 = (X[i], Y[i], THETA[i])
		T1_w = np.array([[math.cos(p1[2]), -math.sin(p1[2]), p1[0]], [math.sin(p1[2]), math.cos(p1[2]), p1[1]], [0, 0, 1]])
		T2_w = np.array([[math.cos(p2[2]), -math.sin(p2[2]), p2[0]], [math.sin(p2[2]), math.cos(p2[2]), p2[1]], [0, 0, 1]])
		T2_1 = np.dot(np.linalg.inv(T1_w), T2_w)
		del_x = T2_1[0][2]
		del_y = T2_1[1][2]
		del_theta = math.atan2(T2_1[1, 0], T2_1[0, 0])
		
		# Add noise
		if(i<5):
			xNoise = 0; yNoise = 0; tNoise = 0
		else:
			# xNoise = np.random.normal(0, 0.003); yNoise = np.random.normal(0, 0.003); tNoise = np.random.normal(0, 0.0003)
			xNoise = 0.0002; yNoise = 0.0002; tNoise = -0.0005
		
		del_xN = del_x + xNoise; del_yN = del_y + yNoise; del_thetaN = del_theta + tNoise

		# Convert to T2_1'
		T2_1N = np.array([[math.cos(del_thetaN), -math.sin(del_thetaN), del_xN], [math.sin(del_thetaN), math.cos(del_thetaN), del_yN], [0, 0, 1]])

		# Get T2_w' = T1_w' . T2_1'
		p1 = (xN[i-1], yN[i-1], tN[i-1])
		T1_wN = np.array([[math.cos(p1[2]), -math.sin(p1[2]), p1[0]], [math.sin(p1[2]), math.cos(p1[2]), p1[1]], [0, 0, 1]])
		T2_wN = np.dot(T1_wN, T2_1N)
		
		# Get x2', y2', theta2'
		x2N = T2_wN[0][2]
		y2N = T2_wN[1][2]
		theta2N = math.atan2(T2_wN[1, 0], T2_wN[0, 0])

		xN[i] = x2N; yN[i] = y2N; tN[i] = theta2N  

	# tN = getTheta(xN, yN)

	return (xN, yN, tN)


def writeOdom(X, Y, THETA):
	g2o = open('noise.g2o', 'w')

	for i, (x, y, theta) in enumerate(zip(X, Y, THETA)):
		line = "VERTEX_SE2 " + str(i) + " " + str(x) + " " + str(y) + " " + str(theta) + "\n"
		g2o.write(line)

	# Odometry
	g2o.write("\n# Odometry constraints\n\n")
	info_mat = "500.0 0.0 0.0 500.0 0.0 500.0"
	for i in range(1, len(X)):
		p1 = (X[i-1], Y[i-1], THETA[i-1])
		p2 = (X[i], Y[i], THETA[i])
		T1_w = np.array([[math.cos(p1[2]), -math.sin(p1[2]), p1[0]], [math.sin(p1[2]), math.cos(p1[2]), p1[1]], [0, 0, 1]])
		T2_w = np.array([[math.cos(p2[2]), -math.sin(p2[2]), p2[0]], [math.sin(p2[2]), math.cos(p2[2]), p2[1]], [0, 0, 1]])
		T2_1 = np.dot(np.linalg.inv(T1_w), T2_w)
		del_x = str(T2_1[0][2])
		del_y = str(T2_1[1][2])
		del_theta = str(math.atan2(T2_1[1, 0], T2_1[0, 0]))
		
		line = "EDGE_SE2 "+str(i-1)+" "+str(i)+" "+del_x+" "+del_y+" "+del_theta+" "+info_mat + "\n"
		g2o.write(line)

	g2o.write("FIX 0\n")
	
	return g2o


def optimize():
	cmd = "g2o -robustKernel Cauchy -robustKernelWidth 1 -o opt.g2o -i 20 noise.g2o > /dev/null 2>&1"
	os.system(cmd)


def readG2o(fileName):
	f = open(fileName, 'r')
	A = f.readlines()
	f.close()

	X = []
	Y = []
	THETA = []

	for line in A:
		if "VERTEX_SE2" in line:
			(ver, ind, x, y, theta) = line.split(' ')
			X.append(float(x))
			Y.append(float(y))
			THETA.append(float(theta.rstrip('\n')))

	return (X, Y, THETA)


def readLoops(fileName):
	f = open(fileName, 'r')
	A = f.readlines()
	f.close()

	src = []
	trg = []

	for line in A:
		(s, t) = line.split(' ')
		src.append(int(s))
		trg.append(int(t))

	return (src, trg)


def writeLoops(X, Y, THETA, g2o, src, trg, noise):
	g2o.write("\n\n# Loop constraints\n\n")
	info_mat = "700.0 0.0 0.0 700.0 0.0 700.0"

	for i in range(len(src)):
		p1 = (X[src[i]], Y[src[i]], THETA[src[i]])
		p2 = (X[trg[i]], Y[trg[i]], THETA[trg[i]])
		T1_w = np.array([[math.cos(p1[2]), -math.sin(p1[2]), p1[0]], [math.sin(p1[2]), math.cos(p1[2]), p1[1]], [0, 0, 1]])
		T2_w = np.array([[math.cos(p2[2]), -math.sin(p2[2]), p2[0]], [math.sin(p2[2]), math.cos(p2[2]), p2[1]], [0, 0, 1]])
		T2_1 = np.dot(np.linalg.inv(T1_w), T2_w)
		del_x = str(T2_1[0][2] + noise)
		del_y = str(T2_1[1][2] + noise)
		del_theta = str(math.atan2(T2_1[1, 0], T2_1[0, 0]) + noise)

		line = "EDGE_SE2 "+str(src[i])+" "+str(trg[i])+" "+del_x+" "+del_y+" "+del_theta+" "+info_mat
		g2o.write(line)
		g2o.write("\n")

	g2o.close()


def optWhole(xN, yN, tN, fileLoops, X, Y, THETA):
	g2o = writeOdom(xN, yN, tN)
	
	(src, trg) = readLoops(fileLoops)
	noise = 0.1
	writeLoops(X, Y, THETA, g2o, src, trg, noise)
	
	optimize()
	(xOpt, yOpt, tOpt) = readG2o("opt.g2o")
	draw(xOpt, yOpt, tOpt, "Noisy_optimized", thetaPlot=False)


def keyPoses(xN, yN, tN, X, Y, THETA, src, trg):
	poses = []
	for i in range(len(src)):
		poses.append((src[i], xN[src[i]], yN[src[i]], tN[src[i]], X[src[i]], Y[src[i]], THETA[src[i]])); poses.append((trg[i], xN[trg[i]], yN[trg[i]], tN[trg[i]], X[trg[i]], Y[trg[i]], THETA[trg[i]]))
	poses = sorted(set(poses))

	iK = []; xNK = []; yNK = []; tNK = []; XK = []; YK = []; THETAK = []
	for p in poses:
		iK.append(p[0]); xNK.append(p[1]); yNK.append(p[2]); tNK.append(p[3]); XK.append(p[4]); YK.append(p[5]); THETAK.append(p[6])

	srcK = []; trgK = []

	for i in range(len(src)):
		srcK.append(iK.index(src[i])); trgK.append(iK.index(trg[i]))	

	return (xNK, yNK, tNK, XK, YK, THETAK, srcK, trgK)


def	optParts(xN, yN, tN, fileLoops, X, Y, THETA):
	(src, trg) = readLoops(fileLoops)

	(xNK, yNK, tNK, XK, YK, THETAK, srcK, trgK) = keyPoses(xN, yN, tN, X, Y, THETA, src, trg)
	draw(xNK, yNK, tNK, "Noisy_keypoints", thetaPlot=True)

	g2o = writeOdom(xNK, yNK, tNK)
	noise = 0.01
	writeLoops(XK, YK, THETAK, g2o, srcK, trgK, noise)	

	optimize()
	(xOpt, yOpt, tOpt) = readG2o("opt.g2o")

	return (xOpt, yOpt, tOpt)


if __name__ == '__main__':
	filePoses = str(argv[1])
	fileLoops = str(argv[2])
	(X, Y, THETA) = read(filePoses)

	X = X[0:-1:20]; Y = Y[0:-1:20]; THETA = THETA[0:-1:20]
	THETA = getTheta(X, Y)
	draw(X, Y, THETA, "GroundTruth", thetaPlot=False)

	# drawId(X, Y)

	np.random.seed(42)
	(xN, yN, tN) = addNoise(X, Y, THETA)
	draw(xN, yN, tN, "Noise_full", thetaPlot=False)

	optWhole(xN, yN, tN, fileLoops, X, Y, THETA)

	(xOpt, yOpt, tOpt) = optParts(xN, yN, tN, fileLoops, X, Y, THETA)
	draw(xOpt, yOpt, tOpt, "Noisy_keypoints_optimized", thetaPlot=True)