# -*- coding: utf-8 -*-
"""
Created on Tue Mar 20 09:59:46 2018

@author: Elisabeth Lys
"""
import time
start_time = time.process_time() # début mesure temps d'éxecusion

# On importe le module numpy qui permet de faire du calcul numérique
#import numpy as np
from numpy import meshgrid, sqrt, linspace, savetxt
# On importe le module matplotlib qui permet de générer des graphiques 2D et 3D
import matplotlib.pyplot as plt


#************************************************************************
#*********** Création de l'objet bouclier dans repère Objet (O,X,Y,Z) ***
#************************************************************************

#Nb pixel Objet (échantillonnage objet)
NbHO = 1024
NbVO = 768
#Rayon sphere mm
R = 375;
#Recul sphere mm
a = 300;
#Coordonnées matricielles des pts M de l'objet
[X,Y] = meshgrid(linspace(-600,600,NbHO),linspace(-450,450,NbVO));
#Affixe de l'objet (mm)
u = -1/3
b = 100
Za2 = (X>=0)* + (Y>=0)*75 + (200>=Y)*75 + (300>=X)*75

Z = Za2 * 75 + (X*u + b);
#Enregistrement des coordonnées matricelles objet
savetxt('X.txt', X, fmt='%-7.6f')   
savetxt('Y.txt', Y, fmt='%-7.6f')
savetxt('Z.txt', Z, fmt='%-7.6f')  


#************************************************************************
#************************ Affichage de l'objet  *************************
#************************************************************************

z_min, z_max = 0, abs(Z).max()
plt.figure();
plt.pcolor(X,Y,Z, cmap='gray', vmin=z_min, vmax=z_max)
plt.title('Z (mm) - Objet bouclier simulé')
# set the limits of the plot to the limits of the data
plt.axis([X.min(), X.max(), Y.min(), Y.max()])
plt.colorbar()
plt.savefig("Objet1.png")

print(time.process_time() - start_time, "seconds")  # fin mesure temps d'éxecusion