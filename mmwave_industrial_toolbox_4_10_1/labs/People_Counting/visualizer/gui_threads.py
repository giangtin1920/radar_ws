from PyQt5.QtCore import QDateTime, Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
        QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
        QSlider, QSpinBox, QStyleFactory, QTableWidget, QTabWidget, QTextEdit,
        QVBoxLayout, QWidget, QFileDialog)
from PyQt5.QtGui import QPainter, QColor, QFont

import pyqtgraph as pg
import pyqtgraph.opengl as gl
import random
import numpy as np
import time

from oob_parser import uartParserSDK
from graphUtilities import *

class parseUartThread(QThread):
        fin = pyqtSignal('PyQt_PyObject')

        def __init__(self, uParser):
                QThread.__init__(self)
                self.parser = uParser

        def run(self):
                pointCloud = self.parser.readAndParseUart()
                self.fin.emit(pointCloud)

class sendCommandThread(QThread):
        done = pyqtSignal()
        def __init__(self, uParser, command):
                QThread.__init__(self)
                self.parser = uParser
                self.command = command

        def run(self):
            self.parser.sendLine(self.command)
            self.done.emit()



class update2DQTGraphThread(QThread):
        done = pyqtSignal()

        def __init__(self, pointCloud, targets, numTargets, indexes, numPoints, trailData, activeTrails, trailPlots, plot2D, gatingPlot):
                QThread.__init__(self)
                self.plot2D = plot2D
                self.gatingPlot = gatingPlot
                self.pointCloud=pointCloud
                self.numTargets=numTargets
                self.indexes=indexes
                self.numPoints = numPoints
                self.targets = targets
                self.trailData = trailData
                self.activeTrails = activeTrails
                self.trailPlots = trailPlots
                self.colorArray = ('r','g','b','w')

        def run(self):
                #plot point Cloud
                print('updating 2d points')
                toPlot = [{'pos':self.pointCloud[:,i]} for i in range(np.shape(self.pointCloud)[1])]
                self.plot2D.setData(toPlot)
                #plot trails
                for i in range(20):
                    lifespan = int(self.activeTrails[i,0])
                    if (lifespan > 0):
                        #plot trail
                        if (lifespan > 100):
                            lifespan = 100
                        trDat = self.trailData[i,:lifespan,0:2]
                        #print(np.shape(trDat))
                        self.trailPlots[i].setData(trDat[:,0], trDat[:,1], pen=pg.mkPen(width=3,color=self.colorArray[i%3]))
                        self.trailPlots[i].setVisible(True)
                    else:
                        self.trailPlots[i].hide()

                #plot tracks
                if (self.numTargets > 0):
                    trackPlot = [{'pos':self.targets[1:3,i],'pen':pg.mkPen(width=25,color=self.colorArray[i%3])} for i in range(self.numTargets)]
                    self.gatingPlot.clear()
                    self.gatingPlot.setData(trackPlot)
                else:
                    self.gatingPlot.clear()
                self.done.emit()

class updateQTTargetThread3D(QThread):
    done = pyqtSignal()

    def __init__(self, pointCloud, targets, indexes, scatter, pcplot, numTargets, ellipsoids, coords,classifierOut=[], zRange=[-3, 3], gw=[], colorByIndex=False, drawTracks=True, bbox=[0,0,0,0,0,0], bbox_en=0):
        QThread.__init__(self)
        self.pointCloud = pointCloud
        self.targets = targets
        self.indexes = indexes
        self.scatter = scatter
        self.pcplot = pcplot
        self.colorArray = ('r','g','b','w')
        self.numTargets = numTargets
        self.ellipsoids = ellipsoids
        self.coordStr = coords
        self.classifierOut = classifierOut
        self.zRange = zRange
        self.gw = gw
        self.colorByIndex = colorByIndex
        self.drawTracks = drawTracks
        self.bbox = bbox
        self.bbox_en = bbox_en

    def drawTrack(self, index):
        #get necessary target data
        tid = int(self.targets[0,index])
        x = self.targets[1,index]
        y = self.targets[2,index]
        z = self.targets[3,index]
        xr = self.targets[11, index]
        yr = self.targets[10, index]
        zr = self.targets[12, index]
        edge_color = pg.glColor(self.colorArray[tid%3])
        track = self.ellipsoids[tid]
        #if classifier is on, set non human targets to white
        if (len(self.classifierOut) != 0):
            try:
                dTID = self.classifierOut[0].tolist()
                posit = dTID.index(tid)
                decision = self.classifierOut[1,posit]
            except Exception as ex:
                print ('Cannot find tid ', tid, ' in list:')
                print (dTID)
                print(ex)
            if(decision != 1):
                edge_color = pg.glColor('w')
        mesh = getBoxLinesCoords(x,y,z)
        track.setData(pos=mesh,color=edge_color,width=2,antialias=True,mode='lines')
        track.setVisible(True)
        #add text coordinates
        ####ctext = self.coordStr[tid]
        #print('coordstr = ',str(self.coordStr[tid]))
        ####ctext.setPosition(x,y,z)#+self.profile['sensorHeight'])
        ####ctext.setVisible(True)
        

    def run(self):
        #sanity check indexes = points
        if (len(self.indexes) != np.shape(self.pointCloud)[1]) and (len(self.indexes)):
            print ('I: ',len(self.indexes), ' P: ',  np.shape(self.pointCloud)[1])
        #clear all previous targets
        for e in self.ellipsoids:
            if (e.visible()):
                e.hide()
        for c in self.coordStr:
            if (c.visible()):
                c.hide()
        #remove points outside boundary box
        #only used in fall detection
        #if (self.bbox_en):
        #    to_delete = []
        #    for i in range(np.shape(self.pointCloud)[1]):
        #        x = self.pointCloud[0,i]
        #        y = self.pointCloud[1,i]
        #        z = self.pointCloud[2,i]
        #        if (x < self.bbox[0] or x > self.bbox[1]):
        #            to_delete.append(i)
        #        elif (y < self.bbox[2] or y > self.bbox[3]):
        #            to_delete.append(i)
        #        elif(z < self.bbox[4] or z > self.bbox[5]):
        #            to_delete.append(i)
        #    self.pointCloud=np.delete(self.pointCloud, to_delete, 1)
        #graph the points with colors
        toPlot = self.pointCloud[0:3,:].transpose()
        size = np.log2(self.pointCloud[4,:].transpose())
        colors = np.zeros((np.shape(self.pointCloud)[1], 4))
        if (self.colorByIndex):
            if (len(self.indexes) > 0):
                try:
                    for i in range(len(self.indexes)):
                        ind = int(self.indexes[i])
                        if (ind < 100):
                            color = pg.glColor(self.colorArray[ind%3])
                        else:
                            color = pg.glColor(self.colorArray[3])
                        colors[i,:] = color[:]
                    self.scatter.setData(pos=toPlot, color=colors, size=size)
                except:
                    print ('Index color fail')
                    self.scatter.setData(pos=toPlot, size=size)
            else:
                self.scatter.setData(pos=toPlot, size=size)
        else:
            for i in range(np.shape(self.pointCloud)[1]):
                #zs = self.zRange + (self.pointCloud[2,i] - self.zRange/2)
                zs = self.pointCloud[2,i]
                if (zs < self.zRange[0]) or (zs > self.zRange[1]):
                    colors[i]=pg.glColor('k')
                else:
                    colorRange = self.zRange[1]+abs(self.zRange[0]) 
                    #zs = colorRange/2 + zs 
                    #zs = self.zRange[0]-zs
                    zs = self.zRange[1] - zs 
                    #print(zs)
                    #print(self.zRange[1]+abs(self.zRange[0]))
                    #print(zs/colorRange)
                    colors[i]=pg.glColor(self.gw.getColor(abs(zs/colorRange)))
            self.scatter.setData(pos=toPlot, color=colors, size=size)
        #graph the targets
        if (self.drawTracks):
            for i in range(self.numTargets):
                self.drawTrack(i)
                '''
                try:
                    self.drawTrack(i)
                except:
                    # pass
                    print('missing track data')
                    print(self.numTargets)
                '''
        self.done.emit()


class updateHeightGraphs(QThread):
    done = pyqtSignal('PyQt_PyObject')

    def __init__(self, vs_in, plots, frameNum,ADCRAW_IN):
        QThread.__init__(self)
        self.plots = plots
        self.frameNum = frameNum
        self.vs = vs_in
        self.ADCRAW = ADCRAW_IN

    def run(self):
        #print("Target detected running updateHeightGraphs")
        #print(self.vs)
        out ={'wave0':[], 'wave1':[],'heart0':[],'heart1':[],'breath0':[],'breath1':[],\
                'heart_rate0':[], 'heart_rate1':[],'breathing_rate0':[],'breathing_rate1':[], \
                'x0':[],'x1':[],'y0':[],'y1':[],'z0':[],'z1':[],  \
                'id0':[],'id1':[],'range0':[],'range1':[],'angle0':[],'angle1':[], \
                'rangeidx0':[],'rangeidx1':[],'angleidx0':[],'angleidx1':[],'test':[]}
        #start by plotting height data, mean height, and delta height of first TID only
        #print(str(out))
        
        out['wave0'] = self.vs[0,0]
        out['wave1'] = self.vs[0,1]
        out['heart0'] = self.vs[1,0]
        out['heart1'] = self.vs[1,1]
        out['breath0'] = self.vs[2,0]
        out['breath1'] = self.vs[2,1]
        out['heart_rate0'] = self.vs[3,0]
        out['heart_rate1'] = self.vs[3,1]
        out['breathing_rate0'] = self.vs[4,0]
        out['breathing_rate1'] = self.vs[4,1]
        out['x0'] = self.vs[5,0]
        out['x1'] = self.vs[5,1]
        # out['x0'] = self.vs[13,0]
        # out['x1'] = self.vs[13,1]
        out['y0'] = self.vs[6,0]
        out['y1'] = self.vs[6,1]
        out['z0'] = self.vs[7,0]
        out['z1'] = self.vs[7,1]
        out['id0'] = self.vs[8,0]
        out['id1'] = self.vs[8,1]
        out['range0'] = self.vs[9,0]
        out['range1'] = self.vs[9,1]
        out['angle0'] = self.vs[10,0]
        out['angle1'] = self.vs[10,1]
        out['rangeidx0'] = self.vs[11,0]
        out['rangeidx1'] = self.vs[11,1]
        out['angleidx0'] = self.vs[12,0]
        out['angleidx1'] = self.vs[12,1]
        out['test'] = self.vs[13:674+13,0]
        out['ADCRAW'] = self.ADCRAW
        ###print(str(out))
        self.done.emit(out)

class zeroHeightGraphs(QThread):
    done = pyqtSignal('PyQt_PyObject')

    def __init__(self, vs_in, plots, frameNum,ADCRAW_IN):
        QThread.__init__(self)
        self.plots = plots
        self.frameNum = frameNum
        self.vs = vs_in
        self.ADCRAW = ADCRAW_IN

    def run(self):
        #print("No target detected running zeroHeightGraphs")
        out ={'wave0':[], 'wave1':[],'heart0':[],'heart1':[],'breath0':[],'breath1':[],\
                'heart_rate0':[], 'heart_rate1':[],'breathing_rate0':[],'breathing_rate1':[], \
                'x0':[],'x1':[],'y0':[],'y1':[],'z0':[],'z1':[],  \
                'id0':[],'id1':[],'range0':[],'range1':[],'angle0':[],'angle1':[], \
                'rangeidx0':[],'rangeidx1':[],'angleidx0':[],'angleidx1':[],'test':[]}
        #start by plotting height data, mean height, and delta height of first TID only
        #print(str(out))
        
        out['wave0'] = 0
        out['wave1'] = 0
        out['heart0'] = 0
        out['heart1'] = 0
        out['breath0'] = 0
        out['breath1'] = 0
        out['heart_rate0'] = 0
        out['heart_rate1'] = 0
        out['breathing_rate0'] = 0
        out['breathing_rate1'] = 0
        out['x0'] = 0
        out['x1'] = 0
        # out['x0'] = self.vs[13,0]
        # out['x1'] = self.vs[13,1]
        out['y0'] = 0
        out['y1'] = 0
        out['z0'] = 0
        out['z1'] = 0
        out['id0'] = 0
        out['id1'] = 0
        out['range0'] = 0
        out['range1'] = 0
        out['angle0'] = 0
        out['angle1'] = 0
        out['rangeidx0'] = 0
        out['rangeidx1'] = 0
        out['angleidx0'] = 0
        out['angleidx1'] = 0
        out['test'] = 0
        out['ADCRAW'] = 0
        self.done.emit(out)