from controller import Robot, DistanceSensor, Motor

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)
    
# Initialisation des moteurs
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Matrice de poids Braitenberg pour suivi du mur à droite :
# Ligne 0 : roue gauche | Ligne 1 : roue droite
# Colonnes : [ps0, ps1, ps2, ps3, ps4, ps5, ps6, ps7]
weights = [
    [-18.0, -14.0, -6.0,  0.0,  0.0,  0.0,  -4.0, -12.0],
    [ 12.0,  10.0,  6.0,  0.0,  0.0,  0.0,   4.0,  10.0]
]

# Vitesses de base asymétriques pour courber naturellement vers le mur droit
BASE_LEFT = 0.55 * MAX_SPEED
BASE_RIGHT = 0.30 * MAX_SPEED

while robot.step(TIME_STEP) != -1:
    # Lecture normalisée des 8 capteurs [0.0, 1.0]
    psValues = [ps[i].getValue() / 4095.0 for i in range(8)]

    # Calcul matriciel pur (V = V_base + W * S)
    leftSpeed = BASE_LEFT + sum(weights[0][i] * psValues[i] for i in range(8))
    rightSpeed = BASE_RIGHT + sum(weights[1][i] * psValues[i] for i in range(8))

    # Saturation aux vitesses limites du robot
    leftSpeed = max(-MAX_SPEED, min(MAX_SPEED, leftSpeed))
    rightSpeed = max(-MAX_SPEED, min(MAX_SPEED, rightSpeed))

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
