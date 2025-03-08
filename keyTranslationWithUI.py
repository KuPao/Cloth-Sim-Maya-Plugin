# keyRotationWithUI.py

import maya.cmds as cmds
import numpy as np
import functools
import sys
sys.path.append("E:/assignment/3DGame/pymel")
from Particle import Particle
from Particle import Spring

CONST_FRAME = 40.0
CONST_STEP = 1.0/24.0 / CONST_FRAME

def createUI( pWindowTitle, pApplyCallback ):
    
    windowID = 'myWindowID'
    
    if cmds.window( windowID, exists=True ):
        cmds.deleteUI( windowID )
        
    cmds.window( windowID, title=pWindowTitle, sizeable=False, resizeToFitChildren=True )
    
    cmds.rowColumnLayout( numberOfColumns=3, columnWidth=[ (1,75), (2,60), (3,60) ], columnOffset=[ (1,'right',3) ] )
    
    cmds.text( label='Time Range:' )
    startTimeField = cmds.intField( value=cmds.playbackOptions( q=True, minTime=True ) )
    endTimeField = cmds.intField( value=cmds.playbackOptions( q=True, maxTime=True ) )
    
    cmds.text( label='Method:' )
    methodField = cmds.optionMenu()
    cmds.menuItem(label="Euler")
    cmds.menuItem(label="RK2")
    cmds.menuItem(label="RK4")
    cmds.menuItem(label="Implicit Euler")
    cmds.menuItem(label="Verlet")
    cmds.menuItem(label="Leapfrog")
    cmds.separator( h=10, style='none' )
    
    cmds.separator( h=10, style='none' )
    cmds.separator( h=10, style='none' )
    cmds.separator( h=10, style='none' )
    
    cmds.separator( h=10, style='none' )
    cmds.button( label='Apply', command=functools.partial( pApplyCallback,
                                                  startTimeField,
                                                  endTimeField,
                                                  methodField,
                                                  windowID ) )
    
    def cancelCallback( *pArgs ):
        if cmds.window( windowID, exists=True ):
            cmds.deleteUI( windowID )
    
    cmds.button( label='Cancel', command=cancelCallback )
    
    cmds.showWindow()

def force( selectionList ):
    for objectName in selectionList:
            particle = Particle.particle_dict[objectName]
            #Clear forces
            particle.F = [0.0, 0.0, 0.0]
            #Add gravitational force
            particle.F += particle.mass*particle.gravity
    for s in Spring.springs:
            """
            Calculates the spring force acting on particle i and j
            """
            pi = Particle.particle_dict[s.nI_]
            pj = Particle.particle_dict[s.nJ_]
            xi = pi.position #Pos of particle i
            xj = pj.position #Pos of particle j
            vi = pi.V #velocity of particle i
            vj = pj.V #velocity of particle j
            deltaX = xj-xi
            norm2 = np.linalg.norm(deltaX)
            spring_force = s.ks_ * deltaX/norm2 *(norm2-s.l0_)     # Spring force
            spring_damp_force = s.kd_ * np.dot((vi-vj),deltaX/norm2)*deltaX/norm2  #Damping on spring
    
            #Add forces
            pi.F += spring_force +spring_damp_force
            pj.F -= spring_force +spring_damp_force

def new_force(X, V):
    for i in range(0, Particle.X.shape[0]):
        #Clear forces
        Particle.F[i] = [0.0, 0.0, 0.0]
        #Add gravitational force
        Particle.F[i] += Particle.M*Particle.G

    for s in Spring.springs:
        """
        Calculates the spring force acting on particle i and j
        """
        #print(s.nI_)
        pi = Particle.particle_dict[s.nI_]
        pj = Particle.particle_dict[s.nJ_]
        xi = Particle.X[pi.id] #Pos of particle i
        xj = Particle.X[pj.id] #Pos of particle j
        vi = Particle.V[pi.id] #velocity of particle i
        vj = Particle.V[pj.id] #velocity of particle j
        deltaX = xj-xi
        norm2 = np.linalg.norm(deltaX)
        spring_force = s.ks_ * deltaX/norm2 *(norm2-s.l0_)     # Spring force
        spring_damp_force = s.kd_ * np.dot((vj-vi),deltaX/norm2)*deltaX/norm2  #Damping on spring
    
        #Add forces
        pi.F += spring_force +spring_damp_force
        Particle.F[pi.id] += (spring_force +spring_damp_force)
        pj.F -= spring_force +spring_damp_force
        Particle.F[pj.id] -= (spring_force +spring_damp_force)

def forceDerivatives(X,V):
    nParticles = len(Particle.P)
    Particle.Jx = np.zeros((3*nParticles, 3*nParticles))
    Particle.Jv = np.zeros((3*nParticles, 3*nParticles))
    for s in Spring.springs:
        pi = Particle.particle_dict[s.nI_]
        pj = Particle.particle_dict[s.nJ_]
        xi = Particle.X[pi.id]
        xj = Particle.X[pj.id]
        deltaX = xj - xi
        norm2 = np.linalg.norm(deltaX)
        I = np.identity(3)
        dxtdx = np.outer(deltaX, deltaX) / (norm2*norm2)

        #Position jacobian
        jx_sub = np.zeros((3,3))
        if(norm2 >= s.l0_):
            jx_sub = s.ks_*(dxtdx + (I-dxtdx)*(1-s.l0_/norm2))
        #jx_sub = s.ks_*(-I + s.l0_/norm2*(I - np.outer(deltaX,deltaX)/(norm2*norm2)))
        """
        Insert into jacobian
        Jx = | Jx_ii  Jx_ij | = | j_sub  -j_sub |
             | Jx_ji  Jx_jj |   | -j_sub  j_sub |
        """
        Particle.Jx[3*pi.id:3*pi.id+3, 3*pi.id:3*pi.id+3] += jx_sub
        Particle.Jx[3*pi.id:3*pi.id+3, 3*pj.id:3*pj.id+3] += -jx_sub
        Particle.Jx[3*pj.id:3*pj.id+3, 3*pi.id:3*pi.id+3] += -jx_sub
        Particle.Jx[3*pj.id:3*pj.id+3, 3*pj.id:3*pj.id+3] += jx_sub

        #Velocity jacobian
        jv_sub = s.kd_ * I
        Particle.Jv[3*pi.id:3*pi.id+3, 3*pi.id:3*pi.id+3] += jv_sub
        Particle.Jv[3*pi.id:3*pi.id+3, 3*pj.id:3*pj.id+3] += -jv_sub
        Particle.Jv[3*pj.id:3*pj.id+3, 3*pi.id:3*pi.id+3] += -jv_sub
        Particle.Jv[3*pj.id:3*pj.id+3, 3*pj.id:3*pj.id+3] += jv_sub

'''
def forceParticle(particle, X, V):
    particle = Particle.particle_dict[objectName]
    #Clear forces
    particle.F = [0.0, 0.0, 0.0]
    #Add gravitational force
    particle.F += particle.mass*particle.gravity
'''
    
def applyCallback( pStartTimeField, pEndTimeField, pMethodField, windowID, *pArgs ):
    
    # print 'Apply button pressed.'
    
    startTime = cmds.intField( pStartTimeField, query=True, value=True )
    
    endTime = cmds.intField( pEndTimeField, query=True, value=True )
    
    method = cmds.optionMenu( pMethodField, query=True, value=True )
    
    print 'Start Time: %s' % ( startTime )
    print 'End Time: %s' % ( endTime )
    print 'Method: %s' % ( method )
    
    selectionList = cmds.ls( selection=True, type='transform' )
    '''old setup'''
    '''for objectName in selectionList:
        particle = Particle.particle_dict[objectName]
        cmds.cutKey( objectName, time=(startTime, endTime), attribute='translateX' )
        cmds.cutKey( objectName, time=(startTime, endTime), attribute='translateY' )
        cmds.cutKey( objectName, time=(startTime, endTime), attribute='translateZ' )
        cmds.setKeyframe( objectName, time=startTime, attribute='translateX', value=particle.origin[0] )
        cmds.setKeyframe( objectName, time=startTime, attribute='translateY', value=particle.origin[1] )
        cmds.setKeyframe( objectName, time=startTime, attribute='translateZ', value=particle.origin[2] )
        particle.V = np.zeros(3)
        particle.position = particle.origin'''
    '''new setup'''
    for i in range(0, len(Particle.P)):
        particle = Particle.P[i]
        objectName = particle.name
        Particle.X[i] = particle.origin
        cmds.cutKey( objectName, time=(startTime, endTime), attribute='translateX' )
        cmds.cutKey( objectName, time=(startTime, endTime), attribute='translateY' )
        cmds.cutKey( objectName, time=(startTime, endTime), attribute='translateZ' )
        cmds.setKeyframe( objectName, time=startTime, attribute='translateX', value=Particle.X[i][0] )
        cmds.setKeyframe( objectName, time=startTime, attribute='translateY', value=Particle.X[i][1] )
        cmds.setKeyframe( objectName, time=startTime, attribute='translateZ', value=Particle.X[i][2] )
        Particle.V[i] = [0.0, 0.0, 0.0]
    
    for s in Spring.springs:
        cmds.cutKey( s.cylinder, time=(startTime, endTime), attribute='translateX' )
        cmds.cutKey( s.cylinder, time=(startTime, endTime), attribute='translateY' )
        cmds.cutKey( s.cylinder, time=(startTime, endTime), attribute='translateZ' )
        cmds.cutKey( s.cylinder, time=(startTime, endTime), attribute='scaleX' )
        cmds.cutKey( s.cylinder, time=(startTime, endTime), attribute='scaleY' )
        cmds.cutKey( s.cylinder, time=(startTime, endTime), attribute='scaleZ' )
        #reCalCylinder(s, startTime)
        newreCalCylinder(s, startTime)
    
    Particle.OldX = Particle.X
    print 'setup'
    if method=="Implicit Euler":
        CONST_FRAME = 1
        CONST_STEP = 1.0/24.0 / CONST_FRAME
    else:
        CONST_FRAME = 20
        CONST_STEP = 1.0/24.0 / CONST_FRAME
    print(CONST_STEP)
    for t in range(0, (endTime - startTime) * int(CONST_FRAME)):
        #force( selectionList )
        OldF = Particle.F
        new_force(Particle.X, Particle.V)
        #print(method)
        if method == "Euler":
            '''new ForwardEuler'''
            oldV = Particle.V
            Particle.V += CONST_STEP * Particle.F / Particle.M
            for c in Particle.constrIdx:
                Particle.V[c] = [0.0, 0.0, 0.0]
            Particle.X += CONST_STEP*oldV

        elif method == "RK2":
            #Step 1
            v1 = Particle.V
            a1 = Particle.F/Particle.M

            #Step 2
            v2 = Particle.V + CONST_STEP*0.75*a1
            X2 = Particle.X+CONST_STEP*0.75*v1
            V2 = Particle.V+CONST_STEP*0.75*a1
            new_force(X2,V2)
            a2 = Particle.F/Particle.M

            #Update Pos and Vel
            deltaX = CONST_STEP*(v1+2*v2)/3
            deltaV = CONST_STEP*(a1+2*a2)/3
            #Check if constrained
            for c in Particle.constrIdx:
                deltaV[c] = [0.0, 0.0, 0.0]
                deltaX[c] = [0.0, 0.0, 0.0]
            Particle.X += deltaX
            Particle.V += deltaV

        elif method == "RK4":
            #Step 1
            a1 = Particle.V
            a2 = Particle.F/Particle.M

            #Step 2
            b1 = Particle.V + CONST_STEP/2*a2
            Xtmp = Particle.X+CONST_STEP/2*a1
            Vtmp = Particle.V+CONST_STEP/2*a2
            new_force(Xtmp,Vtmp)
            b2 = Particle.F/Particle.M

            #Step 3
            c1 = Particle.V + CONST_STEP/2*b2
            Xtmp = Particle.X+CONST_STEP/2*b1
            Vtmp = Particle.V+CONST_STEP/2*b2
            new_force(Xtmp,Vtmp)
            c2 = Particle.F/Particle.M

            #Step 4
            d1 = Particle.V + CONST_STEP*c2
            Xtmp = Particle.X+CONST_STEP*c1
            Vtmp = Particle.V+CONST_STEP*c2
            new_force(Xtmp,Vtmp)
            d2 = Particle.F/Particle.M

            #Update Pos and Vel
            deltaX = CONST_STEP/6*(a1+2*b1+2*c1+d1)
            deltaV = CONST_STEP/6*(a2+2*b2+2*c2+d2)
            #Check if constrained
            for c in Particle.constrIdx:
                deltaV[c] = [0.0,0.0,0.0]
                deltaX[c] = [0.0,0.0,0.0]
            Particle.X += deltaX
            Particle.V += deltaV

        elif method == "Implicit Euler":
            forceDerivatives(Particle.X,Particle.V)
            nParticles = len(Particle.P)
            A = np.identity(3*nParticles)*Particle.M - CONST_STEP * Particle.Jv - CONST_STEP*CONST_STEP*Particle.Jx
            #b = CONST_STEP*(Particle.F.flatten()+CONST_STEP*np.dot(Particle.Jx, Particle.V.flatten()))
            b = CONST_STEP*(OldF.flatten()+CONST_STEP*np.dot(Particle.Jx, Particle.V.flatten()))

            deltaV = np.linalg.solve(A,b)
            deltaV = deltaV.reshape((-1,3))

            for c in Particle.constrIdx:
                deltaV[c] = [0.0,0.0,0.0]
            #deltaX = CONST_STEP*(Particle.V + deltaV)
            Particle.X += CONST_STEP*(Particle.V+deltaV)#/(1+CONST_STEP)

            Particle.V += deltaV

        elif method == "Verlet":
            temp = Particle.X
            oldV = Particle.V
            deltaV = (OldF + Particle.F) / 2 / Particle.M * CONST_STEP
            for c in Particle.constrIdx:
                deltaV[c] = [0.0, 0.0, 0.0]
            Particle.X += Particle.X - Particle.OldX + CONST_STEP*oldV
            Particle.V += deltaV
            Particle.OldX = temp

        elif method == "Leapfrog":
            oldV = Particle.V
            if(t == 0):
                oldV += CONST_STEP / 2 * Particle.F / Particle.M
            Particle.V += CONST_STEP * Particle.F / Particle.M
            for c in Particle.constrIdx:
                Particle.V[c] = [0.0, 0.0, 0.0]
            Particle.X += CONST_STEP*oldV

        if((t+1) % CONST_FRAME == 0):
            #print "setKey"
            #print((t+1) / 2 + 1)
            for i in range(0, len(Particle.P)):
                particle = Particle.P[i]
                objectName = particle.name
                cmds.setKeyframe( objectName, time=(t+1) / CONST_FRAME + 1, attribute='translateX', value=Particle.X[i][0] )
                cmds.setKeyframe( objectName, time=(t+1) / CONST_FRAME + 1, attribute='translateY', value=Particle.X[i][1] )
                cmds.setKeyframe( objectName, time=(t+1) / CONST_FRAME + 1, attribute='translateZ', value=Particle.X[i][2] )

            for s in Spring.springs:
                #reCalCylinder(s, t)
                newreCalCylinder(s, (t+1) / CONST_FRAME + 1)
            cmds.currentTime( (t+1) / CONST_FRAME + 1 )
    print 'caculate force'

    '''for objectName in selectionList:
        cmds.selectKey( objectName, time=(startTime, endTime), attribute='translateX', keyframe=True )
        cmds.keyTangent( inTangentType='linear', outTangentType='linear' )
        cmds.selectKey( objectName, time=(startTime, endTime), attribute='translateY', keyframe=True )
        cmds.keyTangent( inTangentType='linear', outTangentType='linear' )
        cmds.selectKey( objectName, time=(startTime, endTime), attribute='translateZ', keyframe=True )
        cmds.keyTangent( inTangentType='linear', outTangentType='linear' )'''

    for i in range(0, len(Particle.P)):
        particle = Particle.P[i]
        objectName = particle.name
        cmds.selectKey( objectName, time=(startTime, endTime), attribute='translateX', keyframe=True )
        cmds.keyTangent( inTangentType='linear', outTangentType='linear' )
        cmds.selectKey( objectName, time=(startTime, endTime), attribute='translateY', keyframe=True )
        cmds.keyTangent( inTangentType='linear', outTangentType='linear' )
        cmds.selectKey( objectName, time=(startTime, endTime), attribute='translateZ', keyframe=True )
        cmds.keyTangent( inTangentType='linear', outTangentType='linear' )
    '''
    pm.select(clear = True)
    s = cmds.ls("connect*")
    print(type(s))
    cmds.delete(s)
    '''
    
    if cmds.window( windowID, exists=True ):
        cmds.deleteUI( windowID )
    
createUI( 'Simulate', applyCallback )