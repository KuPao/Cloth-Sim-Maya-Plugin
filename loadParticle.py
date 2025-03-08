import maya.cmds as cmds
import pymel.core as pm
import numpy as np
import sys
sys.path.append("E:/assignment/3DGame/pymel")
from Particle import *

singleFilter = "PartcileSimu (*.psim*)"
file_path = cmds.fileDialog2(fileFilter=singleFilter, dialogStyle=1, fileMode=1, returnFilter=False)
readfile = open(file_path[0],'r')

cmds.delete(cmds.ls("basicParticle*"))
cmds.delete(cmds.ls("spring*"))

Particle.P = []
Particle.X = None #Positions
Particle.V = None #Velocities
Particle.F = None #Forces
Particle.constrIdx = []
Spring.springs = []

result = pm.polySphere(r = 0.5, n = 'basicParticle')
transformName = result[0]
particleNum = 0

def addParticle(pTransformName, pid, pX, pY, pZ, sC, m):
    global particleNum
    
    instanceResult = pm.instance( pTransformName, n = pTransformName + '_instance#' )
    position = np.array([pX, pY, pZ])
    point = Particle(position, str(instanceResult[0]), sC, m, pid)

    if sC == True:
        Particle.constrIdx.append(pid)
        
    particleNum += 1

    Particle.particle_dict[str(instanceResult[0])] = point

    Particle.P.append(point)

    if not type(Particle.X) is np.ndarray:
        Particle.X = np.array([position])
    else:
        newPos = np.array([position])
        Particle.X = np.concatenate((Particle.X, newPos))
        
    if not type(Particle.V) is np.ndarray:
        Particle.V = np.zeros((1,3))
    else:
        newV = np.zeros((1,3))
        Particle.V = np.concatenate((Particle.V, newV))

    if not type(Particle.F) is np.ndarray:
        Particle.F = np.zeros((1,3))
    else:
        newF = np.zeros((1,3))
        Particle.F = np.concatenate((Particle.F, newF))
    
    pm.move( pX, pY, pZ, instanceResult )

def createSpring( length, ks, kd, pA, pB):
	xA = Particle.X[pA.id]
	xB = Particle.X[pB.id]

	c = createCylinder(xA, xB, pB)
	s = Spring(length, ks, kd, pA.name, pB.name, c)
	Spring.springs.append(s)

def createCylinder(xA, xB, target):
	c = cmds.polyCylinder(r=0.125, h=1, n="spring#")[0]
	l = np.linalg.norm((xB-xA))
	cmds.xform(c, t=(xA+xB)/2, s=(1,l,1), a=True)
	tgt = cmds.ls(target.name)
	cmds.select( clear=True )
	cmds.select(tgt, c)
	const = cmds.aimConstraint( aim=(0,1,0), worldUpVector=(0,1,0), worldUpType = "vector")
	return c

while True:
    line = readfile.readline()
    if not line:
        pm.delete(transformName)
        break
    words = line.split()
    if(words[0] == 'p' and len(words) == 7):
        addParticle( transformName, int(words[1]), float(words[2]), float(words[3]), float(words[4]), bool(int(words[6])), float(words[5]) )
        Particle.M = float(words[5])

    elif(words[0]=='s' and len(words) == 6):
        pA = Particle.P[int(words[4])]
        pB = Particle.P[int(words[5])]
        createSpring( float(words[1]), float(words[2]), float(words[3]), pA, pB)

print(Particle.constrIdx)