import pymel.core as pm
import maya.cmds as cmds
import numpy as np
import math
from copy import deepcopy

CONST_KS_STRETCH    = 2.0e+2    # Spring constant (N/m)
CONST_KS_SHEAR      = 1.0e+2    # --||--
CONST_KS_BEND       = 0.0       #1.0e+3    # --||--
CONST_KD_STRETCH    = 20.1     # Damping coefficient
CONST_KD_SHEAR      = 20.1    # --||--
CONST_KD_BEND       = 0.001    # --||--

class Spring:
    """ Class for Springs connecting vertices in the spring system """
    springs = []
    def __init__(self, L0, ks, kd, I, J, c):
        self.l0_ = L0   # Rest length
        self.ks_ = ks   # Spring constant
        self.kd_ = kd   # Spring damping coeff
        self.nI_ = I  # name to first particle connected to spring
        self.nJ_ = J  # name to second ------||------
        self.cylinder = c
        
def createSpring():
	selectionList = cmds.ls( selection=True, type='transform' )
	pA = Particle.particle_dict[selectionList[0]]
	pB = Particle.particle_dict[selectionList[1]]
	xA = Particle.X[pA.id]
	xB = Particle.X[pB.id]
	euclid_dist = np.linalg.norm((pA.position-pB.position))
	euclid_dist = np.linalg.norm((xA-xB))

	c = createCylinder(xA, xB, pB)
	s = Spring(euclid_dist,CONST_KS_STRETCH,CONST_KD_STRETCH, pA.name, pB.name, c)
	Spring.springs.append(s)
	print(s.cylinder)

def createCylinder(xA, xB, target):
	c = cmds.polyCylinder(r=0.125, h=1, n="spring#")[0]
	l = np.linalg.norm((xB-xA))
	cmds.xform(c, t=(xA+xB)/2, s=(1,l,1), a=True)
	tgt = cmds.ls(target.name)
	cmds.select( clear=True )
	cmds.select(tgt, c)
	const = cmds.aimConstraint( aim=(0,1,0), worldUpVector=(0,1,0), worldUpType = "vector")
	return c

def reCalCylinder(s, t):
	pA = Particle.particle_dict[s.nI_]
	pB = Particle.particle_dict[s.nJ_]
	l = np.linalg.norm((pB.position-pA.position))
	trans=(pA.position + pB.position)/2
	cmds.move( trans[0], trans[1], trans[2], s.cylinder, absolute=True )
	cmds.scale( 1, l, 1, s.cylinder, absolute=True )
	cmds.setKeyframe(s.cylinder, attribute='translateX', time=t)
	cmds.setKeyframe(s.cylinder, attribute='translateY', time=t)
	cmds.setKeyframe(s.cylinder, attribute='translateZ', time=t)
	cmds.setKeyframe(s.cylinder, attribute='scaleX', time=t)
	cmds.setKeyframe(s.cylinder, attribute='scaleY', time=t)
	cmds.setKeyframe(s.cylinder, attribute='scaleZ', time=t)
	
def newreCalCylinder(s, t):
	pA = Particle.particle_dict[s.nI_]
	pB = Particle.particle_dict[s.nJ_]
	xA = Particle.X[pA.id]
	xB = Particle.X[pB.id]
	l = np.linalg.norm((xB-xA))
	trans=(xA + xB)/2
	cmds.move( trans[0], trans[1], trans[2], s.cylinder, absolute=True )
	cmds.scale( 1, l, 1, s.cylinder, absolute=True )
	cmds.setKeyframe(s.cylinder, attribute='translateX', time=t)
	cmds.setKeyframe(s.cylinder, attribute='translateY', time=t)
	cmds.setKeyframe(s.cylinder, attribute='translateZ', time=t)
	cmds.setKeyframe(s.cylinder, attribute='scaleX', time=t)
	cmds.setKeyframe(s.cylinder, attribute='scaleY', time=t)
	cmds.setKeyframe(s.cylinder, attribute='scaleZ', time=t)

class Particle:
    particle_dict = dict()
    P = []
    X = None #Positions
    V = None #Velocities
    F = None #Forces
    M = 30
    G = np.array([0, -9.81, 0])
    Jx = None
    Jv = None
    constrIdx = []
    OldX = None
    def __init__(self, pos, n, sC, m, idN):
		self.position = pos
		self.origin = np.array([pos[0], pos[1], pos[2]])
		self.name = n
		self.lastPosition = pos
		self.mass = m
		self.gravity = np.array([0, -9.81, 0])
		self.pinned = sC
		self.accY = 0.0
		self.accX = 0.0
		self.links = []
		self.V = np.zeros(3) #Velocities
		self.F = np.zeros(3) #Forces
		self.id = idN

class Cloth:
    def __init__(self, dimX, dimY, m, s):
        """
            Initialize
        """
        self.dX = dimX # Mesh dimension
        self.dY = dimY # Mesh dimension
        self.nElements = (dimX*dimY) #Number of elements/Particles
        self.X = np.zeros((self.nElements, 3)) #Positions
        self.V = np.zeros((self.nElements, 3)) #Velocities
        self.F = np.zeros((self.nElements, 3)) #Forces
        self.mass = m # Mass
        self.M = np.diag(np.zeros(3*self.nElements)+self.mass) #diagonal mass matrix
        self.sGravity = np.array([0, -9.81, 0]) #Standard gravity acceleration
        self.Jx = np.zeros((3*self.nElements, 3*self.nElements)) #Force Jacobian. J(x), used by implicit methods
        self.Jv = np.zeros((3*self.nElements, 3*self.nElements)) #Force Jacobian. J(v), used by implicit methods
        self.constrIdx = np.array([])
        
        self.__UniMeshParticleCreator(dimX,dimY,s)
        print("Particles initialized")
        self.springs = []
        self.torSprings = []
        self.__NewSpringCreator(self.X,s)
        print("Springs initialized")

    def __UniMeshParticleCreator(self,dimY, dimX, pSpacing):
        #Create Particles in uniform mesh
        self.particles = []
        for i in range(0,dimY):
            for j in range(0,dimX):
                pos = np.asarray([i*pSpacing,j*pSpacing,1.0])
                self.X[i*dimX+j] = pos