import pymel.core as pm
import maya.cmds as cmds
import numpy as np
import functools
import sys
sys.path.append("E:/assignment/3DGame/pymel")
from Particle import *

def createUI( pWindowTitle, pApplyCallback ):
    windowID = 'myWindowID'

    if pm.window( windowID, ex=True ):
        pm.deleteUI( windowID )

    selectionList = cmds.ls( selection=True, type='transform' )
    p = Particle.particle_dict[selectionList[0]]
    
    pm.window( windowID, title = pWindowTitle, sizeable = False, resizeToFitChildren = True )
    pm.rowColumnLayout( numberOfColumns=4, columnWidth=[ (1,75), (2,60), (3,60), (4,60) ], columnOffset=[ (1,'right',3) ] )
    pm.text( label='Position:' )
    xPosField = pm.floatField( value = p.origin[0])
    yPosField = pm.floatField( value = p.origin[1])
    zPosField = pm.floatField( value = p.origin[2])

    pm.text( label='Static:' )
    pm.separator( h=10, style='none' )
    pm.separator( h=10, style='none' )
    staticBox = pm.checkBox(value = p.pinned, label = '', align = 'right')

    pm.text( label='Mass:' )
    pm.separator( h=10, style='none' )
    pm.separator( h=10, style='none' )
    massField = pm.floatField( value = p.mass)

    pm.separator( h=10, style='none' )
    pm.separator( h=10, style='none' )
    pm.button( label='Apply', command=functools.partial( pApplyCallback,
                                                  xPosField,
                                                  yPosField,
                                                  zPosField,
                                                  staticBox,
                                                  massField,
                                                  windowID ) )
    
    def cancelCallback( *pArgs ):
        if pm.window( windowID, exists=True ):
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

def reCalC(p):
    for s in Spring.springs:
        if(p.name == s.nI_ or p.name == s.nJ_):
            pA = Particle.particle_dict[s.nI_]
            pB = Particle.particle_dict[s.nJ_]
            l = np.linalg.norm((pB.position-pA.position))
            trans=(pA.position + pB.position)/2
            cmds.move( trans[0], trans[1], trans[2], s.cylinder, absolute=True )
            cmds.scale( 1, l, 1, s.cylinder, absolute=True )

def applyCallback( pXField, pYField, pZField, pStaticBox, pMField, windowID, *pArgs ):
    
    # print 'Apply button pressed.'
    selectionList = cmds.ls( selection=True, type='transform' )
    p = Particle.particle_dict[selectionList[0]]
    
    xPos = pm.floatField( pXField, query=True, value=True )
    yPos = pm.floatField( pYField, query=True, value=True )
    zPos = pm.floatField( pZField, query=True, value=True )
    sCheck = pm.checkBox( pStaticBox, query=True, value=True)
    mass = pm.floatField( pMField, query=True, value=True )

    if not sCheck and p.pinned:
        Particle.constrIdx.remove(p.id)
    elif sCheck and not p.pinned:
        Particle.constrIdx.append(p.id)

    position = np.array([xPos, yPos, zPos])
    p.origin = position
    p.position = position
    p.pinned = sCheck
    p.mass = mass

    Particle.X[p.id] = position
    pm.move(xPos, yPos, zPos, selectionList[0]) 
    reCalC(p)
    if pm.window( windowID, exists=True ):
            pm.deleteUI( windowID )
    #addParticle( transformName, xPos, yPos, zPos, sCheck, mass )

createUI( 'Edit Particle', applyCallback )