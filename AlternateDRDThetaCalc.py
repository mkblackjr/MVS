# AlternateDRDThetaCalc.py

import math

def calc_DR_DTheta(x, y, targetX, targetY):

	R1 = math.sqrt(x**2+y**2)
	R2 = math.sqrt(targetX**2+targetY**2)
	
	Theta1 = atan2(y,x)
	Theta1 = (Theta1 < 0)*2*math.pi + Theta1
	Theta2 = atan2(targetY, targetX)
	Theta2 = (Theta2 < 0)*2*math.pi + Theta2

	dR = R2 - R1
	dTheta = Theta2 - Theta1

	return (dR, dTheta)