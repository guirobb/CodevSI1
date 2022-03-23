from numpy import cos, sin, array, pi, linalg, loadtxt

alpha = 40*pi/180
beta = 100*pi/180
phiR = -pi/2
thetaR = -pi/2-(alpha+beta)/2
psiR = 0

def matrice_sens_inverse(phiR,thetaR,psiR):
    r11R = cos(phiR) * cos(thetaR)
    r21R = sin(phiR) * cos(thetaR)
    r31R = -sin(thetaR)
    r12R = cos(phiR) * sin(thetaR) * sin(psiR) - sin(phiR) * cos(psiR)
    r22R = sin(phiR) * sin(thetaR) * sin(psiR) + cos(phiR) * cos(psiR)
    r32R = cos(thetaR) * sin(psiR)
    r13R = cos(phiR) * sin(thetaR) * cos(psiR) + sin(phiR) * sin(psiR)
    r23R = sin(phiR) * sin(thetaR) * cos(psiR) - cos(phiR) * sin(psiR)
    r33R = cos(thetaR) * cos(psiR)

    RR = array([[r11R, r12R, r13R], [r21R, r22R, r23R], [r31R, r32R, r33R]])
    return linalg.inv(RR)

X = loadtxt('X.txt')
Y =loadtxt('Y.txt')
Z = loadtxt('Z.txt')

#Chargement nombre de trame
N = loadtxt('N.txt')
a = matrice_sens_inverse(phiR,thetaR,psiR)
b