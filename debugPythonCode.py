import time

position = "R-11.3423T1.32Z-1.14421"
R_coordinate = ''
Theta_coordinate = ''
Z_coordinate = ''

isError = True
for char in position:
	position = position[1:]
	if(char == 'R'):
		isError = False
		break
if isError:
	isError = False
	raise ValueError('The encoded position from the Arduino was not ' + \
		'formatted properly. It did not include the "R" flag.')

isError = True
for char in position:
	position = position[1:]
	if(char != 'T'):
		R_coordinate += char
	else:
		isError = False
		break

if isError:
	isError = False
	raise ValueError('The encoded position from the Arduino was not ' + \
		'formatted properly. It did not include the "T" flag.')

isError = True
for char in position:
	position = position[1:]
	if(char != 'Z'):
		Theta_coordinate += char
	else:
		isError = False
		break
if isError:
	isError = False
	raise ValueError('The encoded position from the Arduino was not ' + \
		'formatted properly. It did not include the "Z" flag.')

for char in position:
	position = position[1:]
	if(char == 'R'):
		break
	else:
		Z_coordinate += char

R_coordinate = float(R_coordinate)
Theta_coordinate = float(Theta_coordinate)
Z_coordinate = float(Z_coordinate)


print("The current position coordinates are:\nR = " + str(R_coordinate) + \
	"\nTheta = "+ str(Theta_coordinate) + "\nZ = " + str(Z_coordinate))