#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 04 16:36:16 2014
Class to operate turret and sensors 
"""
__author__ = "Julien Favrichon, Jérémy Seyssaud"
__copyright__ = "Copyright 2014, CEA"
__license__ = ""
__version__ = "1.0.7" #24/02/2015
__status__ = "Prototype"

import time # on peux appeler sleep par tim.sleep
from time import sleep # on peut directement utiliser sleep sans le time.
from dxl.dxlchain import * # importe toute methode du module dxlchain dans le dossier dxl
from abc import ABCMeta,abstractmethod
import math as mt
import vtk

import cv2.cv as cv
import cv2
#from cloud_plot import *
import json
import os
import numpy as np

from consts import *

class ResultTourelle(object):
    """
    Results class
    
    time
    data_type
    sensor_type
    """
    def __init__(self,mission="Rosetta"):
        self.mission = mission
        self.data=[]
                  
    def __str__(self):
        return "Results of campaign {0}".format(self.mission)  
    
    def addResultSensor(self,data,data_type,time,position):
        pass
    
    def save2Json(self):
        pass
    
    def save2HDF5(self):
        pass

class ResultSensor(object):
    """
    Abstract class for sensors    
    """
    __metaclass__ = ABCMeta

    @abstractmethod
    def addResult(self):
        pass
    @abstractmethod
    def getResults(self):
        pass

class ResultCamera(ResultSensor):
    """
    Results class for camera device
    
    time
    data_type
    sensor_type
    """
    def __init__(self,folder=".",mission="Rosetta"):
        self.mission = mission
        
        #Création du dossier résultats s'il n'existe pas
        folder_name = folder + "/" + self.mission
        self.folder = folder_name
        print "Folder name",folder_name
        if not os.path.exists(folder_name):
          os.makedirs(folder_name)
        assert os.path.isdir(folder_name),"Can't create folder"
        
        self.img_files = []
        self.pos = []
        
        self.metadata = {}
        self.metadata["name"] =  self.mission     
        self.metadata["time"] = time.strftime("%c")
        self.metadata["filename"] = self.img_files
        self.metadata["position"] = self.pos
        
        print "ResultCamera init Ok"
                   
    def addResult(self,data,current_time,position,acq_index):
        self.pos.append(position)      
        if position is not None:        
            pan,tilt = position
        else:
            pan = None
            tilt = None
        filenum="pan_"+str(pan)+"_tilt_"+str(tilt)
        filename=self.folder + "/snap_" + str(acq_index) + "_" + filenum +  ".jpg"
        self.img_files.append(filename)        
        print "filename",filename
        #filename = directory + '/' + datetime.now().strftime('%Y-%m-%d-%H%M%S') + ".jpg"
    
        try:
            cv2.imwrite(filename, data)
            print "write img",filename
        except:
            print "Can't write image file result"
    
    def getResults(self):
        return self.img_files
    
    def getResultsMeta(self):
        return self.metadata
        
    def getAcqPos(self):
        return self.pos
        
    def serialiseur(obj):
        if isinstance(obj, ResultCamera):
            return {"__class__": "ResultCamera",
                    "nom": obj.nom,
                    "musiques": obj.musiques}
        
        raise TypeError(repr(obj) + " n'est pas sérialisable !")
        
    def save2JSON(self):
        
        filename="./" + self.mission + "/datas_cam.json"
        """
        res_file=open(filename,"w")
        t = json.dumps(self.metadata)
        res_file.write(t)
        res_file.close()
        """
        with open(filename, 'w') as f:
            json.dump(self.metadata, f, indent=2)

class ResultTelemeter(ResultSensor):
    """
    Results class for telemeter device
    
    """
    def __init__(self,folder=".",mission="Rosetta"):
        self.mission = mission
       #Création du dossier résultats s'il n'existe pas
        folder_name = folder + "/" + self.mission
        self.folder = folder_name
        print "Folder name",folder_name
        if not os.path.exists(folder_name):
          os.makedirs(folder_name)
        assert os.path.isdir(folder_name),"Can't create folder"
        
        self.results = []
        self.resultsXYZ = []
        #TODO Passage de resultat sous dictionnaire
        
    def addResult(self,data,current_time,position,acq_index):
        print "len(data)",len(data)        
        if len(data) > 2: #X,Y,Z
            print "data telemeter",data
            self.addResultXYZ(data,current_time,position,acq_index)
        else: #PAN,TILT
            pan,tilt=position
            self.results.append((pan,tilt,data))
        
    def addResultXYZ(self,data,current_time,position,acq_index):
        x,y,z=data
        self.resultsXYZ.append((x,y,z,data))
            
    def getResults(self):
        return self.results
        
    def getResultsXYZ(self):
        return self.resultsXYZ

    def results2resultsXYZ(self):
        for res in self.results:
            T = np.matrix([[0],[SENSOR_TELEMETER_L],[SENSOR_TELEMETER_l+res[2]]])
            Rz = np.matrix([[mt.cos(mt.radians(res[0])),(-mt.sin(mt.radians(res[0]))),0],[mt.sin(mt.radians(res[0])),mt.cos(mt.radians(res[0])),0],[0,0,1]])
            Rx = np.matrix([[1,0,0],[0,mt.cos(mt.radians(-res[1])),(-mt.sin(mt.radians(-res[1])))],[0,mt.sin(mt.radians(-res[1])),mt.cos(mt.radians(-res[1]))]])
            pos = Rz*Rx*T
            self.resultsXYZ.append((pos[0],pos[1],pos[2]))
        print 'XYZ transformation OK'

    def _saveJSON(self,res):
         res_file=open("result_telemeter.txt","w")
         t = json.dumps(res)
         try:
             res_file.write(t)
             res_file.close()
         except:
             print "Save 2 JSON failed"
             return False
         else:
            return True
              
    def save2JSON(self):
        if self._saveJSON(self.results):
             print 'Save results to JSON OK'
        else:
            print 'Save results to JSON FAILED'
            
    def save2JSON_XYZ(self):
        if self._saveJSON(self.resultsXYZ):
             print 'Save results to JSON OK'
        else:
            print 'Save results to JSON FAILED'
         
    def saveXYZ2PCD(self):
        #res_file=open("result_telemeter.pcd","w")
        filename=self.folder + "/result_telemeter.pcd"
        buf = "# .PCD v.7 - Point Cloud Data file format\n"
        buf +="VERSION .7\n"
        buf +="FIELDS x y z\n"
        buf +="SIZE 4 4 4\n"
        buf +="TYPE F F F\n"
        buf +="COUNT 1 1 1\n"
        buf +="WIDTH "+str(len(self.results))+"\n"
        buf +="HEIGHT 1\n"
        buf +="VIEWPOINT 0 0 0 1 0 0 0\n"
        buf +="POINTS "+str(len(self.results))+"\n"
        buf +="DATA ascii\n"
        for res in self.resultsXYZ:
            buf += str(res[0])+" "+str(res[1])+" "+str(res[2])+"\n"
        with open(filename,"w") as res_file:
            res_file.write(buf)
        #res_file.close()
        print 'Save results to PCD OK'

    def saveXYZ2STL(self):
        filename = "test.stl"
        stlWriter = vtk.vtkSTLWriter()
        appendData = vtk.vtkAppendPolyData()
        for res in self.resultsXYZ:
            Sphere = vtk.vtkSphereSource()
            Sphere.SetRadius(0.01)
            Sphere.SetPhiResolution(5)
            Sphere.SetThetaResolution(5)
            Sphere.SetCenter(res[0],res[1],res[2])
            appendData.AddInputConnection(Sphere.GetOutputPort())
           # print i,len(self.cloud)
        stlWriter.SetInput(appendData.GetOutput())
        stlWriter.SetFileName(filename)
        stlWriter.Write()
        print 'Save resultsXYZ to STL OK'

    def plotXYZ(self):
        self.surfaceSize = 5
        ### Sphere Actor
        self.sphereActor1 = list()
        for res in self.resultsXYZ:
            sphere1 = vtk.vtkSphereSource()
            sphere1.SetCenter(res[0], res[1], res[2])
            sphere1.SetRadius(0.015)
            sphere1.SetThetaResolution(100)
            sphere1.SetPhiResolution(100)
            sphereMapper1 = vtk.vtkPolyDataMapper()
            sphereMapper1.SetInput(sphere1.GetOutput())
            sphereActor = vtk.vtkActor()
            sphereActor.SetMapper(sphereMapper1)
            sphereActor.GetProperty().SetColor(1.5, 0.5, 0.)
            self.sphereActor1.append(sphereActor)

        ###Axes Actor
        self.axesActor = vtk.vtkAxesActor()
        self.axesActor.AxisLabelsOn()

        ### Ground Actor
        plane = vtk.vtkPlaneSource()
        plane.SetCenter(0, 0, 0)
        plane.SetNormal(0, 0, 1)
        plane.SetXResolution(20)
        plane.SetYResolution(20)
        transform = vtk.vtkTransform()
        transform.Scale(self.surfaceSize, self.surfaceSize, 1)
        transF = vtk.vtkTransformPolyDataFilter()
        transF.SetInputConnection(plane.GetOutputPort())
        transF.SetTransform(transform)
        # mapper
        planeMapper = vtk.vtkPolyDataMapper()
        planeMapper.SetInput(transF.GetOutput())
        # actor
        self.planeActor = vtk.vtkActor()
        self.planeActor.SetMapper(planeMapper)
        self.planeActor.GetProperty().SetRepresentationToWireframe()

        ### Plot
        ren = vtk.vtkRenderer()
        ren.AddActor(self.axesActor)
        ren.AddActor(self.planeActor)
        for i in range(len(self.resultsXYZ)):
            ren.AddActor(self.sphereActor1[i])
        renWin = vtk.vtkRenderWindow()
        renWin.SetSize(800, 800)
        renWin.AddRenderer(ren)
        ren.SetBackground(0, 0, 0)
        iren = vtk.vtkRenderWindowInteractor()
        iren.SetRenderWindow(renWin)
        iren.Initialize()
        renWin.Render()
        iren.Start()


class ResultTest(ResultSensor):
    """
    Results class for test device
    
    """
    def __init__(self,mission="Rosetta",nb=0):
        self.mission = mission
        #Création du dossier résultats s'il n'existe pas
        """folder_name = "./" + self.mission
        if not os.path.exists(folder_name):
          os.mkdir(folder_name)
        assert os.path.isdir(folder_name),"Can't create folder" 
        """
        if nb == 0: #data list type
            self.data = []
        else: # numpy array type
            self.data = np.zeros((nb,3))
            self.index = 0
        print "ResultTest self.data",self.data   
        self.metadata = {}
        self.metadata["name"] =  self.mission     
        self.metadata["time"] = time.strftime("%c")
                   
    def addResult(self,data,current_time,position,acq_index):
        print "addResult position",position        
        pan,tilt = position        
        if isinstance(self.data, list) :
            self.data.append((pan,tilt,data))
        else:
            print ("index=",self.index)            
            self.data[self.index] = [pan,tilt,data]
            self.index += 1
            
    def getResults(self):
        return self.metadata,self.data
        
    def loadJSON(self,file_name="result_test.txt"):
        res_file=open(file_name,"r")      
        try:
            res=json.loads(res_file.readlines)
        except Exception,e:
               print "Error JSON LOAD:",e
               return
        
        print "loadJSON",res
        
    def save2JSON(self):
         filename = "result_test.txt"
         """
         res_file=open("result_test.txt","w")
         t = json.dumps(self.data)
         res_file.write(t)
         res_file.close()
         """
         with open(filename, 'w') as f:
            json.dump(self.data, f, indent=2)
    
    def save2NPY(self):
        """
        Save data in Numpy txt format         
        """
        np.savetxt("result_test.npy",self.data)
         
    def save2PCD(self):
        res_file=open("result_telemeter.pcd","w")
        
        buf = "# .PCD v.7 - Point Cloud Data file format\n"
        buf +="VERSION .7\n"
        buf +="FIELDS x y z\n"
        buf +="SIZE 4 4 4\n"
        buf +="TYPE F F F\n"
        buf +="COUNT 1 1 1\n"
        buf +="WIDTH "+str(len(self.data))+"\n"
        buf +="HEIGHT 1\n"
        buf +="VIEWPOINT 0 0 0 1 0 0 0\n"
        buf +="POINTS "+str(len(self.data))+"\n"
        buf +="DATA ascii\n"
        
        for res in self.data:
            buf += str(res[0])+" "+str(res[1])+" "+str(res[2])+"\n"
        
        res_file.write(buf)
        res_file.close()

if __name__ == "__main__": 
    print "Results Classes"