import pymel.core as pm
import maya.cmds as cmds
import numpy as np
import functools
import sys
sys.path.append("E:/assignment/3DGame/pymel")
from Particle import *

def addParticle(pTransformName, pX, pY, pZ, sC, m):
    global particleNum
    
    instanceResult = pm.instance( pTransformName, n = pTransformName + '_instance#' )
    position = np.array([pX, pY, pZ])
    point = Particle(position, str(instanceResult[0]), sC, m, particleNum)

    if sC == True:
        Particle.constrIdx.append(particleNum)
        
    particleNum += 1
    print(point.mass)
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
    
    print(Particle.X)
    pm.move( pX, pY, pZ, instanceResult )

def delCalC(p):
    for s in Spring.springs:
        if(p.name == s.nI_ or p.name == s.nJ_):
            pm.hide(s.cylinder)
            Spring.springs.remove(s)

def deleteParticle():
    selectionList = cmds.ls( selection=True, type='transform' )
    p = Particle.particle_dict[selectionList[0]]

    if not p.pinned:
        Particle.constrIdx.append(p.id)

    position = np.array([0, 0, 0])
    p.origin = position
    p.position = position
    p.pinned = True

    Particle.X[p.id] = np.array([0, 0, 0])
    Particle.V[p.id] = np.array([0, 0, 0])
    Particle.F[p.id] = np.array([0, 0, 0])
    pm.hide(selectionList[0])
    delCalC(p)

deleteParticle()