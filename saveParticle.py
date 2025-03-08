import maya.cmds as cmds
import sys
sys.path.append("E:/assignment/3DGame/pymel")
from Particle import Particle
from Particle import Spring

singleFilter = "PartcileSimu (*.psim*)"
file_path = cmds.fileDialog2(fileFilter=singleFilter, dialogStyle=1, fileMode=0, returnFilter=False)
writefile = open(file_path[0],'w')


data = ""
for i in range(len(Particle.P)):
    p = Particle.P[i]
    data += ("p " + str(p.id) + " " + str(p.origin[0]) + " " + str(p.origin[1]) + " " + str(p.origin[2]) + " " + str(Particle.M) + " ")
    if(p.pinned):
        data += "1\n"
    else:
        data += "0\n"
    writefile.write(data)
    data = ""

for s in Spring.springs:
    pi = Particle.particle_dict[s.nI_]
    pj = Particle.particle_dict[s.nJ_]
    data += "s " + str(s.l0_) + " " + str(s.ks_) + " " + str(s.kd_) + " " + str(pi.id) + " " + str(pj.id) + "\n"
    writefile.write(data)
    data = ""
writefile.write(data)