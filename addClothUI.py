import pymel.core as pm
import maya.cmds as cmds
import numpy as np
import functools
import sys
sys.path.append("E:/assignment/3DGame/pymel")
from Particle import *

CONST_F_ERROR = 1e-09

result = pm.polySphere(r = 0.5, n = 'basicParticle')
#print 'result: ' + str( result )

transformName = result[0]
#instanceGroupName = pm.group( em = True, n=transformName + '_instance_grp#' )
particleNum = 0

def createUI( pWindowTitle, pApplyCallback ):
    windowID = 'myWindowID'

    if pm.window( windowID, ex=True ):
        pm.deleteUI( windowID )
    
    pm.window( windowID, title = pWindowTitle, sizeable = False, resizeToFitChildren = True )
    pm.rowColumnLayout( numberOfColumns=4, columnWidth=[ (1,75), (2,60), (3,60), (4,60) ], columnOffset=[ (1,'right',3) ] )
    pm.text( label='Row:' )
    pm.separator( h=10, style='none' )
    pm.separator( h=10, style='none' )
    rowField = pm.intField( value = 5)

    pm.text( label='Column:' )
    pm.separator( h=10, style='none' )
    pm.separator( h=10, style='none' )
    colField = pm.intField( value = 5)

    pm.text( label='Mass:' )
    pm.separator( h=10, style='none' )
    pm.separator( h=10, style='none' )
    massField = pm.floatField( value = 30)

    pm.text( label='Length:' )
    pm.separator( h=10, style='none' )
    pm.separator( h=10, style='none' )
    lenField = pm.floatField( value = 5)

    pm.separator( h=10, style='none' )
    pm.separator( h=10, style='none' )
    pm.button( label='Apply', command=functools.partial( pApplyCallback,
                                                  rowField,
                                                  colField,
                                                  massField,
                                                  lenField ) )
    
    def cancelCallback( *pArgs ):
        if pm.window( windowID, exists=True ):
            pm.delete(transformName)
            #pm.xform( instanceGroupName, centerPivots=True )
            pm.deleteUI( windowID )
    
    pm.button( label='Cancel', command=cancelCallback )
    pm.showWindow()

def addParticle(pTransformName, pX, pY, pZ, sC, m):
    global particleNum
    
    instanceResult = pm.instance( pTransformName, n = pTransformName + '_instance#' )
    position = np.array([pX, pY, pZ])
    point = Particle(position, str(instanceResult[0]), sC, m, particleNum)

    if sC == True:
        Particle.constrIdx.append(particleNum)
        
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

def addCloth( row, col, mass, length):
    cmds.delete(cmds.ls("basicParticle_*"))
    cmds.delete(cmds.ls("spring*"))

    Particle.P = []
    Particle.X = None #Positions
    Particle.V = None #Velocities
    Particle.F = None #Forces
    Particle.M = mass
    Particle.constrIdx = []
    Spring.springs = []

    h = (row - 1) * length
    w = (col - 1) * length
    for i in range(0, row):
        for j in range(0, col):
            pos = np.asarray([i*length-h/2, 20, j*length-w/2])
            addParticle(transformName, pos[0], pos[1], pos[2], False, mass)

    for i, xi in enumerate(Particle.X):
        for j, xj in enumerate(Particle.X):
            taxicab_dist = np.linalg.norm((xi-xj),ord=1)
            euclid_dist = np.linalg.norm((xi-xj))
            if((abs(taxicab_dist) <= abs((2*length))+ CONST_F_ERROR) and i<j):
                #Create springs
                if(abs(taxicab_dist-length) <= CONST_F_ERROR):
                    #create stretch spring
                    c = createCylinder(xi, xj, Particle.P[j])
                    spring = Spring(euclid_dist,CONST_KS_STRETCH,CONST_KD_STRETCH,Particle.P[i].name,Particle.P[j].name,c)
                    Spring.springs.append(spring)
                if(euclid_dist < (2*length) and (abs(taxicab_dist-(2*length)) <= CONST_F_ERROR)):
                    #create shear springs
                    c = createCylinder(xi, xj, Particle.P[j])
                    spring = Spring(euclid_dist,CONST_KS_SHEAR,CONST_KD_SHEAR,Particle.P[i].name,Particle.P[j].name,c)
                    Spring.springs.append(spring)


def applyCallback( rowField, colField, massField, lenField, *pArgs ):
    
    row = pm.intField( rowField, query=True, value=True )
    col = pm.intField( colField, query=True, value=True )
    mass = pm.floatField( massField, query=True, value=True )
    length = pm.floatField( lenField, query=True, value=True )
    
    addCloth( row, col, mass, length )

createUI( 'Create Cloth', applyCallback )