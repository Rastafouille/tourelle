# -*- coding: utf-8 -*-
"""
Created on Wed Feb  3 21:35:22 2016

@author: jf205732
"""
from numpy import hypot

# Développé à partir de http://fr.wikibooks.org/wiki/Math%C3%A9matiques_avec_Python_et_Ruby/Vecteurs_en_Python
class Point:
    def __init__(self,x,y):
        self.x=x
        self.y=y
 
    def affichage(self):
        return '('+str(self.x)+';'+str(self.y)+')'
 
    def milieu(self,p):
        return Point((self.x+p.x)/2,(self.y+p.y)/2)
 
    def vecteur(self,p):
        return Vecteur(p.x-self.x,p.y-self.y)
 
    def distance(self,p):
        return self.vecteur(p).norme()

class Vecteur:
    def __init__(self,x,y):
        self.x=x
        self.y=y
 
    def affichage(self):
        return '('+str(self.x)+';'+str(self.y)+')'

    def norme(self):
        return hypot(self.x,self.y)
 
    def __add__(self,v):
        return Vecteur(self.x+v.x,self.y+v.y)
 
    def __rmul__(self,r):
        return Vecteur(self.x*r,self.y*r)
 
    def __mul__(self,v):
        return self.x*v.x+self.y*v.y
 
    def colin(self,v):
        return self.x*v.y==self.y*v.x
    
    def det (self,v):
        return self.x*v.y-self.y*v.x
 
    def ortho(self,v):
        return self*v==0

if __name__ == '__main__':
    print "lib_geometry"