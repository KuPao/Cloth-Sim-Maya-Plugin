import pymel.core as pm
import maya.cmds as cmds
import numpy as np
import functools
import sys
sys.path.append("E:/assignment/3DGame/pymel")
from Particle import *

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
    pm.text( label='Position:' )
    xPosField = pm.floatField( value = 0)
    yPosField = pm.floatField( value = 0)
    zPosField = pm.floatField( value = 0)

    pm.text( label='Static:' )
    pm.separator( h=10, style='none' )
    pm.separator( h=10, style='none' )
    staticBox = pm.checkBox(value = 1, label = '', align = 'right')

    pm.text( label='Mass:' )
    pm.separator( h=10, style='none' )
    pm.separator( h=10, style='none' )
    massField = pm.floatField( value = 30)

    pm.separator( h=10, style='none' )
    pm.separator( h=10, style='none' )
    pm.button( label='Apply', command=functools.partial( pApplyCallback,
                                                  xPosField,
                                                  yPosField,
                                                  zPosField,
                                                  staticBox,
                                                  massField ) )
    
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

def applyCallback( pXField, pYField, pZField, pStaticBox, pMField, *pArgs ):
    
    # print 'Apply button pressed.'
    
    xPos = pm.floatField( pXField, query=True, value=True )
    yPos = pm.floatField( pYField, query=True, value=True )
    zPos = pm.floatField( pZField, query=True, value=True )
    sCheck = pm.checkBox( pStaticBox, query=True, value=True)
    mass = pm.floatField( pMField, query=True, value=True )
    
    print 'X: %s' % ( xPos )
    print 'Y: %s' % ( yPos )
    print 'Z: %s' % ( zPos )
    
    addParticle( transformName, xPos, yPos, zPos, sCheck, mass )

createUI( 'Add Particle', applyCallback )