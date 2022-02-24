import sys
from PyQt5 import QtGui
from PyQt5.QtCore import QDateTime, Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
        QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
        QSlider, QSpinBox, QStyleFactory, QTableWidget, QTableWidgetItem, QTabWidget, QTextEdit,
        QVBoxLayout, QWidget, QFileDialog, QButtonGroup)
from PyQt5.QtGui import QPixmap
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.pgcollections import OrderedDict
 
import random
import numpy as np
import time
import math
import struct
import statistics
import datetime
import matplotlib.pyplot as plt
import os

#from gl_classes import GLTextItem
from oob_parser import uartParserSDK
from gui_threads import *
from graphUtilities import *
compileGui = 0
#only when compiling
if (compileGui):
    from fbs_runtime.application_context.PyQt5 import ApplicationContext

class Window(QDialog):
    def __init__(self, parent=None, size=[]):
        super(Window, self).__init__(parent)
        # set window toolbar options, and title. #deb_gp
        self.setWindowFlags(
            Qt.Window |
            Qt.CustomizeWindowHint |
            Qt.WindowTitleHint |
            Qt.WindowMinimizeButtonHint |
            Qt.WindowMaximizeButtonHint |
            Qt.WindowCloseButtonHint
        )
        self.setWindowTitle("Vital Sign Visualizer")

        print('Python is ', struct.calcsize("P")*8, ' bit')
        print('Python version: ', sys.version_info)

        self.yzFlip = 0
        self.frameTime = 120
        self.graphFin = 1
        self.hGraphFin = 1
        self.threeD = 1
        self.lastFramePoints = np.zeros((5,1))
        self.plotTargets = 1
        self.frameNum = 0
        self.lastTID = []
        self.profile = {'startFreq': 60.25, 'numLoops': 64, 'numTx': 3, 'sensorHeight':3, 'maxRange':10, 'az_tilt':0, 'elev_tilt':0}
        self.lastFrameHadTargets = False
        self.sensorHeight = 1.5
        self.numFrameAvg = 10
        self.configSent = 0
        self.previousFirstZ = -1
        #timer to reset fall detected message
        self.fallTimer = QTimer()
        self.fallTimer.setSingleShot(True)
        self.fallTimer.timeout.connect(self.resetFallText)
        self.fallResetTimerOn = 0
        self.fallThresh = -0.22
        self.frame_counter = 0
        self.frame_counter2 = 0
        self.filter_counter = 0
        self.StartRecordFlag = 0
        self.adcraw_abs_buffer=np.zeros((128,100))
        self.adcraw_abs_var_threshold=np.zeros(128)
        self.heartrate_buf=np.zeros((128,1))
        self.breathrate_buf=np.zeros((128,1))
        self.buf_temp=np.zeros((128,1))
        self.heartrate_final=0
        self.breathrate_final=0
        self.heartrate_last=0;
        self.breathrate_last=0;
        self.now_time = datetime.datetime.now().strftime('%H%M%S')
        self.log_record_file_name = 'LogRecord_'+datetime.datetime.now().strftime('%Y%m%d-%H%M')
        self.numTargets_display=0

       
        #color gradients
        self.Gradients = OrderedDict([
    ('bw', {'ticks': [(0.0, (0, 0, 0, 255)), (1, (255, 255, 255, 255))], 'mode': 'rgb'}),
    ('hot', {'ticks': [(0.3333, (185, 0, 0, 255)), (0.6666, (255, 220, 0, 255)), (1, (255, 255, 255, 255)), (0, (0, 0, 0, 255))], 'mode': 'rgb'}),
    ('jet', {'ticks': [(1, (166, 0, 0, 255)), (0.32247191011235954, (0, 255, 255, 255)), (0.11348314606741573, (0, 68, 255, 255)), (0.6797752808988764, (255, 255, 0, 255)), (0.902247191011236, (255, 0, 0, 255)), (0.0, (0, 0, 166, 255)), (0.5022471910112359, (0, 255, 0, 255))], 'mode': 'rgb'}),
    ('summer', {'ticks': [(1, (255, 255, 0, 255)), (0.0, (0, 170, 127, 255))], 'mode': 'rgb'} ),
    ('space', {'ticks': [(0.562, (75, 215, 227, 255)), (0.087, (255, 170, 0, 254)), (0.332, (0, 255, 0, 255)), (0.77, (85, 0, 255, 255)), (0.0, (255, 0, 0, 255)), (1.0, (255, 0, 127, 255))], 'mode': 'rgb'}),
    ('winter', {'ticks': [(1, (0, 255, 127, 255)), (0.0, (0, 0, 255, 255))], 'mode': 'rgb'}),
    ('spectrum2', {'ticks': [(1.0, (255, 0, 0, 255)), (0.0, (255, 0, 255, 255))], 'mode': 'hsv'}),
])
        cmap = 'spectrum2'
        if (cmap in self.Gradients):
            self.gradientMode = self.Gradients[cmap]
        self.zRange = [-3,3]
        self.plotHeights = 1
        #gui size
        if (size):
            left = 50
            top = 50
            width = math.ceil(size.width()*0.9)
            height = math.ceil(size.height()*0.9)
            self.setGeometry(left, top, width, height)
        #persistent point cloud
        self.previousCloud = np.zeros((6,1150,10))
        self.previousPointCount = np.zeros((10,1))
        #self.previousIndex = np.zeros((1,1150,10))
        #images
        self.standingPicture = QPixmap('images/stickFigureStanding.png')
        self.fallingPicture = QPixmap('images/stickFigureFalling.png')
        #remove points outside boundary box
        self.bbox = [-1000, 1000, -1000, 1000, -1000, 1000]
        self.bbox_2 = [-1000, 1000, -1000, 1000, -1000, 1000]
        self.bbox_2_en = 0

        #vs plot data
        self.wave0 = np.zeros(286)
        self.wave1 = np.zeros(286)
        self.heart0 = np.zeros(286)
        self.heart1 = np.zeros(286)
        self.breath0 = np.zeros(286)
        self.breath1 = np.zeros(800)
        #setup graph pyqtgraph
        self.plot3DQTGraph()
        self.colorGradient()
        self.heightPlots()
        self.VSData()
        #self.plot2DQTGraph()

        #add connect options
        self.setConnectionLayout()
        self.setStatsLayout()
        self.setPlotControlLayout()
        self.setConfigLayout()
        #self.setControlLayout()
        self.setUpBoundaryBoxControls()
        self.setSensorPositionControls()

        # set the layout
        #create tab for different graphing options
        self.graphTabs = QTabWidget();
        self.graphTabs.addTab(self.pcplot, '3D Plot')
        #self.graphTabs.addTab(self.legacyPlot, '2D Plot')
        self.graphTabs.currentChanged.connect(self.whoVisible)

        gridlay = QGridLayout()
        gridlay.addWidget(self.comBox, 0,0,1,1)
        gridlay.addWidget(self.statBox, 1,0,1,1)
        gridlay.addWidget(self.configBox,2,0,1,1)
        gridlay.addWidget(self.plotControlBox,3,0,1,1)
        #gridlay.addWidget(self.boxTab,4,0,1,1)
        #gridlay.addWidget(self.spBox,5,0,1,1)
        gridlay.addWidget(self.graphTabs,1,1,6,2)
        #gridlay.addWidget(self.gw, 0, 2, 6, 1)
        #gridlay.addWidget(self.vsData0, 0,1,1,1)    #Y,X height  wideth
        #gridlay.addWidget(self.vsData1, 0,2,1,1)    #Y,X height  wideth
        gridlay.addWidget(self.vsData1, 0,1,1,2)    #Y,X height  wideth
        gridlay.addWidget(self.hPlot,0,3,6,2)
        gridlay.setColumnStretch(0,1)
        gridlay.setColumnStretch(1,2)
        gridlay.setColumnStretch(2,2)
        gridlay.setColumnStretch(3,1)
        gridlay.setColumnStretch(4,1)
        self.setLayout(gridlay)
        
    def heightPlots(self):
        self.heightPlots = {'breath0':[],'breath1':[],'heart0':[],'heart1':[]}
        yMin = -0.5
        yMax = 0.5
        self.hPlot = QGroupBox('vital sign')
        #setup instant height plot
        self.iHPlot = pg.PlotWidget()
        #self.iHPlot.setYRange(-0.3,0.3,padding=0.1)
        xMin = 226
        xMax = 512
        # self.iHPlot.setXRange(xMin,xMax,padding=0.1)
        # self.iHPlot.enableAutoRange(axis='y', enable=True)
        self.iHPlot.setYRange(-0.6,0.6,padding=0.1)
        # self.iHPlot.setTitle('breathing_waveform0__20s_data')
        self.iHPlot.setTitle('Heart_waveform0__20s_data')
        self.iHPlot.showGrid(x=True, y=True)
        self.iHPlot.setBackground('w')
        self.iHPlot.setMouseEnabled(False,False)
        #self.iHPlot.hideAxis('bottom')
        
        #add curve
        self.heightPlots['breath0']=pg.PlotCurveItem(pen=pg.mkPen(width=3, color='r'))
        self.iHPlot.addItem(self.heightPlots['breath0'])

              #setup Delta Height Plot
        self.dHPlot = pg.PlotWidget()
        # self.dHPlot.setTitle('heart_waveform0__20s_data')
        self.dHPlot.setTitle('Breathing_waveform0__20s_data')
        #self.dHPlot.setYRange(yMin,yMax,padding=0.1)
        # self.dHPlot.setYRange(-0.1,0.1,padding=0.1)
        self.dHPlot.enableAutoRange(axis='y', enable=True)
        xMin = 226
        xMax = 512
        # self.dHPlot.setXRange(xMin,xMax,padding=0.1)
        self.dHPlot.showGrid(x=True,y=True)
        self.dHPlot.setBackground('w')
        self.dHPlot.setMouseEnabled(False,False)
        #self.dHPlot.hideAxis('bottom')
        #add curve
        self.heightPlots['heart0']=pg.PlotCurveItem(pen=pg.mkPen(width=3, color='b'))
        self.dHPlot.addItem(self.heightPlots['heart0'])
        
        #setup mean height plot
        self.mHPlot = pg.PlotWidget()
        # self.mHPlot.setTitle('breathing_waveform1__20s_data')
        self.mHPlot.setTitle('Range_FFT_ABS')
        #self.mHPlot.setYRange(-1,1,padding=0.1)
        self.mHPlot.enableAutoRange(axis='y', enable=True)
        xMin = 226
        xMax = 512
        # self.mHPlot.setXRange(xMin,xMax,padding=0.1)
        self.mHPlot.showGrid(x=True,y=True)
        self.mHPlot.setBackground('w')
        self.mHPlot.setMouseEnabled(False,False)
        #self.mHPlot.hideAxis('bottom')
        #add curve
        self.heightPlots['breath1']=pg.PlotCurveItem(pen=pg.mkPen(width=3, color='r'))
        self.mHPlot.addItem(self.heightPlots['breath1'])
        self.heightPlots['breath1_test']=pg.PlotCurveItem(pen=pg.mkPen(width=10, color='b'))
        self.mHPlot.addItem(self.heightPlots['breath1_test'])
         
        self.heightPlots['breath1_test2']=pg.ScatterPlotItem([2], [3],size=10, symbol='+', color = 'b')
        self.mHPlot.addItem(self.heightPlots['breath1_test2'])

        self.HPlot = pg.PlotWidget()
        # self.HPlot.setTitle('heart_waveform1__20s_data')
        self.HPlot.setTitle('Result: B-breath R-Heart')
        #self.HPlot.setYRange(yMin,yMax,padding=0.1)
        self.HPlot.enableAutoRange(axis='y', enable=True)
        xMin = 226
        xMax = 512
        # self.HPlot.setXRange(xMin,xMax,padding=0.1)
        self.HPlot.showGrid(x=True,y=True)
        self.HPlot.setBackground('w')
        self.HPlot.setMouseEnabled(False,False)
        #self.HPlot.hideAxis('bottom')
        #add curve
        self.heightPlots['heart1']=pg.PlotCurveItem(pen=pg.mkPen(width=3, color='b'))
        self.HPlot.addItem(self.heightPlots['heart1'])
        self.heightPlots['heart1_test']=pg.PlotCurveItem(pen=pg.mkPen(width=3, color='r'))
        self.HPlot.addItem(self.heightPlots['heart1_test'])
        

        self.heightLayout = QGridLayout()
        self.heightLayout.addWidget(self.iHPlot,0,0)
        #self.heightLayout.addWidget(self.mHPlot,2,0)
        self.heightLayout.addWidget(self.dHPlot,1,0)
        #self.heightLayout.addWidget(self.HPlot,3,0)
        self.hPlot.setLayout(self.heightLayout)





        
    def WriteFile2(self, data):
        filepath=self.log_record_file_name + '.txt'
        binfile = open(filepath, 'a+') 
        binfile.write(data)
        binfile.close()        
        
    def VSData(self):
        #self.vsData0 = QGroupBox('Data0')
        self.vsData1 = QGroupBox('Data Output')
        
        self.xyz0 = QLabel('XYZ: 0.00,0.00,0.00')
        self.xyz1 = QLabel('XYZ: 0.00,0.00,0.00')
        self.range0 = QLabel('RIdx = 0')
        #self.range1 = QLabel('reson = 0')
        self.range1 = QLabel('Status')
        self.breathrate0 = QLabel('Breath = 0')
        self.breathrate1 = QLabel('Breath = 0')
        self.heartrate0 = QLabel('Heart = 0')
        self.heartrate1 = QLabel('Heart = 0')
        ft=QFont()
        ft.setPointSize(12)
        ft2=QFont()
        ft2.setPointSize(20)
        ft3=QFont()
        ft3.setPointSize(30)
        ft4=QFont()
        #ft4.setPointSize(8)
        ft4.setPointSize(20)
        self.xyz0.setFont(ft)
        self.xyz1.setFont(ft4)
        self.range0.setFont(ft)
        self.range1.setFont(ft)
        self.breathrate0.setFont(ft2)
        self.breathrate1.setFont(ft2)
        self.heartrate0.setFont(ft2)
        self.heartrate1.setFont(ft2)
        
        
        layoutDD = QGridLayout()
        layoutDD1 = QGridLayout()
        layoutDD.setSpacing(50);
        layoutDD1.setSpacing(50);
        
        #layoutDD.addWidget(self.xyz0,1,1)
        layoutDD.addWidget(self.range0,2,1)
        #layoutDD.addWidget(self.breathrate0,1,2)
        #layoutDD.addWidget(self.heartrate0,2,2)
    
        #layoutDD1.addWidget(self.xyz1,1,1)
        layoutDD1.addWidget(self.range1,2,1)
        layoutDD1.addWidget(self.range0,1,1)
        layoutDD1.addWidget(self.breathrate1,1,2)
        layoutDD1.addWidget(self.heartrate1,2,2)
        #self.vsData0.setLayout(layoutDD)
        #self.vsData0.addLayout(layoutDD1)
        self.vsData1.setLayout(layoutDD1)        
#
# left side pane layout
#
    def setConnectionLayout(self):
        self.comBox = QGroupBox('Connect to Com Ports')
        self.uartCom = QLineEdit('')    #deb_gp
        self.dataCom = QLineEdit('')    #deb_gp
        self.uartLabel = QLabel('UART COM:')
        self.dataLabel = QLabel('DATA COM:')
        self.connectStatus = QLabel('Not Connected')
        self.connectButton = QPushButton('Connect')
        self.RecordButton = QPushButton('Record')
        self.connectButton.clicked.connect(self.connectCom)
        self.RecordButton.clicked.connect(self.RecordStd)
        self.configLabel = QLabel('Config Type:')
        self.configType = QComboBox()
        self.configType.addItems(["Vital Signs", "3D People Counting", "SDK Out of Box Demo", "Long Range People Detection", "(Legacy) 2D People Counting", "(Legacy): Overhead People Counting"])
        self.comLayout = QGridLayout()
        self.comLayout.addWidget(self.uartLabel,0,0)
        self.comLayout.addWidget(self.uartCom,0,1)
        self.comLayout.addWidget(self.dataLabel,1,0)
        self.comLayout.addWidget(self.dataCom,1,1)
        self.comLayout.addWidget(self.configLabel,2,0)
        self.comLayout.addWidget(self.configType,2,1)
        self.comLayout.addWidget(self.connectButton,3,0)
        self.comLayout.addWidget(self.connectStatus,3,1)
        ####self.comLayout.addWidget(self.RecordButton,4,0)
        self.comBox.setLayout(self.comLayout)

    def setStatsLayout(self):
        self.statBox = QGroupBox('Statistics')
        self.frameNumDisplay = QLabel('Frame: 0')
        self.plotTimeDisplay = QLabel('Average Plot Time: 0 ms')
        self.numPointsDisplay = QLabel('Points: 0')
        self.numTargetsDisplay = QLabel('Targets: 0')
        self.statsLayout = QVBoxLayout()
        self.statsLayout.addWidget(self.frameNumDisplay)
        self.statsLayout.addWidget(self.plotTimeDisplay)
        self.statsLayout.addWidget(self.numPointsDisplay)
        self.statsLayout.addWidget(self.numTargetsDisplay)
        self.statBox.setLayout(self.statsLayout)

    def setPlotControlLayout(self):
        self.plotControlBox = QGroupBox('Plot Controls')
        self.staticclutter = QCheckBox('Display Static Points')
        self.plotByIndex = QCheckBox('Plot Point Color by Index')
        self.plotByHeight = QCheckBox('Plot Point Color By Height')
        self.plotTracks = QCheckBox('Plot Tracks')
        self.pointColorGroup = QButtonGroup()
        self.pointColorGroup.addButton(self.plotByIndex)
        self.pointColorGroup.addButton(self.plotByHeight)
        self.pointColorGroup.setExclusive(True)
        self.persistentFramesInput = QComboBox()
        self.persistentFramesInput.addItems(['1','2','3','4','5','6','7','8','9','10'])
        self.persistentFramesInput.setCurrentIndex(2)
        self.pFILabel = QLabel('# of Persistent Frames')
        self.orientationSelection = QComboBox()
        self.orientationSelection.addItems(['Side Mount', 'Overhead Mount'])
        self.orientationSelection.currentIndexChanged.connect(self.swapOrientations)
        self.oriLabel = QLabel('Orientation')
        #self.fallThreshInput = QLineEdit(str(self.fallThresh))
        #self.fallThreshInput.textEdited.connect(self.updateFallThresh)
        #self.fallTLabel = QLabel('Fall Detection Threshold')
        self.plotControlLayout = QGridLayout()
        self.plotControlLayout.addWidget(self.plotByIndex, 0, 0,1,1)
        self.plotControlLayout.addWidget(self.plotByHeight, 1, 0,1,1)
        self.plotControlLayout.addWidget(self.plotTracks, 2, 0,1,1)
        self.plotControlLayout.addWidget(self.persistentFramesInput,4,0,1,1)
        self.plotControlLayout.addWidget(self.pFILabel,4,1,1,1)
        self.plotControlLayout.addWidget(self.staticclutter,3,0,1,1)
        self.plotControlLayout.addWidget(self.orientationSelection, 5,0,1,1)
        self.plotControlLayout.addWidget(self.oriLabel, 5,1,1,1)
        #self.plotControlLayout.addWidget(self.fallThreshInput,4,0,1,1)
        #self.plotControlLayout.addWidget(self.fallTLabel,4,1,1,1)
        self.plotControlBox.setLayout(self.plotControlLayout)
        #initialize button values
        self.plotByHeight.setChecked(True)
        self.plotByIndex.setChecked(False)
        self.plotTracks.setChecked(True)


    def setConfigLayout(self):
        self.configBox = QGroupBox('Configuration')
        self.selectConfig = QPushButton('Select Configuration')
        self.sendConfig = QPushButton('Start and Send Configuration')
        self.startButton = QPushButton('Start without Send Configuration')
        self.selectConfig.clicked.connect(self.selectCfg)
        self.sendConfig.clicked.connect(self.sendCfg)
        self.startButton.clicked.connect(self.startFunc)       
        self.configTable = QTableWidget(5,2)
        #set parameter names
        self.configTable.setItem(0,0,QTableWidgetItem('Radar Parameter'))
        self.configTable.setItem(0,1,QTableWidgetItem('Value'))
        self.configTable.setItem(1,0,QTableWidgetItem('Max Range'))
        self.configTable.setItem(2,0,QTableWidgetItem('Range Resolution'))
        self.configTable.setItem(3,0,QTableWidgetItem('Max Velocity'))
        self.configTable.setItem(4,0,QTableWidgetItem('Velcoity Resolution'))
        self.configLayout = QVBoxLayout()
        self.configLayout.addWidget(self.selectConfig)
        self.configLayout.addWidget(self.sendConfig)
        self.configLayout.addWidget(self.startButton)
        self.configLayout.addWidget(self.configTable)       
        #self.configLayout.addStretch(1)
        self.configBox.setLayout(self.configLayout)

    def setControlLayout(self):
        self.controlBox = QGroupBox('Control')
        self.rangecfar = QSlider(Qt.Horizontal)
        self.azcfar = QSlider(Qt.Horizontal)
        self.snrthresh = QSlider(Qt.Horizontal)
        self.pointsthresh = QSlider(Qt.Horizontal)
        self.gatinggain = QSlider(Qt.Horizontal)
        self.staticclutter = QCheckBox('Static Clutter Removal')
        self.controlLayout = QVBoxLayout()
        self.rangelabel = QLabel('Range CFAR Threshold: ')
        self.azlabel = QLabel('Azimuth CFAR Threshold: ')
        self.snrlabel = QLabel('SNR Threshold: ')
        self.pointslabel = QLabel('Points Threshold: ')
        self.gatinglabel = QLabel('Gating Gain: ')
        self.controlLayout.addWidget(self.rangelabel)
        self.controlLayout.addWidget(self.rangecfar)
        self.controlLayout.addWidget(self.azlabel)
        self.controlLayout.addWidget(self.azcfar)
        self.controlLayout.addWidget(self.snrlabel)
        self.controlLayout.addWidget(self.snrthresh)
        self.controlLayout.addWidget(self.pointslabel)
        self.controlLayout.addWidget(self.pointsthresh)
        self.controlLayout.addWidget(self.gatinglabel)
        self.controlLayout.addWidget(self.gatinggain)
        self.controlLayout.addWidget(self.staticclutter)
        self.controlBox.setLayout(self.controlLayout)

    #boundary box control section
    def setBoxControlLayout(self, name):
        #set up one boundary box control
        boxControl = QGroupBox(name)
        #input boxes
        lx = QLineEdit('-6')
        rx = QLineEdit('6')
        ny = QLineEdit('0')
        fy = QLineEdit('6')
        bz = QLineEdit('-6')
        tz = QLineEdit('6')
        enable = QCheckBox()
        #labels
        lxL = QLabel('Left X')
        rxL = QLabel('Right X')
        nyL = QLabel('Near Y')
        fyL = QLabel('Far Y')
        bzL = QLabel('Bottom Z')
        tzL = QLabel('Top Z')
        enableL = QLabel('Enable Box')
        boxConLayout = QGridLayout()
        boxConLayout.addWidget(lxL, 0, 0,1,1)
        boxConLayout.addWidget(lx,0,1,1,1)
        boxConLayout.addWidget(rxL,0,2,1,1)
        boxConLayout.addWidget(rx,0,3,1,1)
        boxConLayout.addWidget(nyL, 1, 0,1,1)
        boxConLayout.addWidget(ny,1,1,1,1)
        boxConLayout.addWidget(fyL,1,2,1,1)
        boxConLayout.addWidget(fy,1,3,1,1)
        boxConLayout.addWidget(bzL, 2, 0,1,1)
        boxConLayout.addWidget(bz,2,1,1,1)
        boxConLayout.addWidget(tzL,2,2,1,1)
        boxConLayout.addWidget(tz,2,3,1,1)
        boxConLayout.addWidget(enableL,3,0,1,1)
        boxConLayout.addWidget(enable,3,1,1,1)
        ####boxControl.setLayout(boxConLayout)
        boundList = [lx,rx,ny,fy,bz,tz]
        for text in boundList:
            text.textEdited.connect(self.changeBoundaryBox)
        enable.stateChanged.connect(self.changeBoundaryBox)
        return {'boxCon':boxControl, 'boundList':boundList, 'checkEnable':enable, 'boxNum':-1}

    def setSensorPositionControls(self):
        self.az_tilt = QLineEdit('0')
        self.az_tiltL = QLabel('Azimuth Tilt')
        self.elev_tilt = QLineEdit('0')
        self.elev_tiltL = QLabel('Elevation Tilt')
        self.s_height = QLineEdit(str(self.profile['sensorHeight']))
        self.s_heightL = QLabel('Sensor Height')
        self.spLayout = QGridLayout()
        self.spLayout.addWidget(self.az_tilt,0,1,1,1)
        self.spLayout.addWidget(self.az_tiltL,0,0,1,1)
        self.spLayout.addWidget(self.elev_tilt,1,1,1,1)
        self.spLayout.addWidget(self.elev_tiltL,1,0,1,1)
        self.spLayout.addWidget(self.s_height,2,1,1,1)
        self.spLayout.addWidget(self.s_heightL,2,0,1,1)
        self.spBox = QGroupBox('Sensor Position')
        ####self.spBox.setLayout(self.spLayout)
        self.s_height.textEdited.connect(self.updateSensorPosition)
        self.az_tilt.textEdited.connect(self.updateSensorPosition)
        self.elev_tilt.textEdited.connect(self.updateSensorPosition)

    def updateSensorPosition(self):
        try:
            float(self.s_height.text())
            float(self.az_tilt.text())
            float(self.elev_tilt.text())
        except:
            print("fail to update")
            return
        command = "sensorPosition " + self.s_height.text() + " " + self.az_tilt.text() + " " + self.elev_tilt.text() + " \n"
        self.cThread = sendCommandThread(self.parser,command)
        self.cThread.start(priority=QThread.HighestPriority-2)
        self.gz.translate(dx=0,dy=0,dz=self.profile['sensorHeight'])
        self.profile['sensorHeight'] = float(self.s_height.text())
        self.gz.translate(dx=0,dy=0,dz=-self.profile['sensorHeight'])

    def setUpBoundaryBoxControls(self):
        #set up all boundary box controls
        self.boundaryBoxes = []
        self.boxTab = QTabWidget()
        for i in range(2):
            name = 'Box'+str(i)
            self.boundaryBoxes.append(self.setBoxControlLayout(name))
            toAdd = self.boundaryBoxes[i]
            toAdd['boxNum'] = i
            #if (i == 0):
            #    toAdd['checkEnable'].setChecked(True)
            self.boxTab.addTab(toAdd['boxCon'], name)
    #for live tuning when available
    #def changeBoundaryBox(self):
    #    #send box values
    #    numBoxes = 0
    #    for box in self.boundaryBoxes:
    #        if(box['checkEnable'].isChecked()):
    #            numBoxes += 1
    #    boundaryString = "LiveScenery " + str(numBoxes) + " " 
    #    for box in self.boundaryBoxes:
    #        if(box['checkEnable'].isChecked()):
    #            for text in box['boundList']:
    #                val = text.text()
    #                val = val.replace(" ","")
    #                try:
    #                    float(val)
    #                except:
    #                    print('nothing here')
    #                    return
    #                boundaryString += text.text() + " "
    #            self.drawBoundaryBox3d(box['boxNum'])
    #        else:
    #            print("Setting box ", box['boxNum'], " invisisble")
    #            self.boundaryBoxViz[box['boxNum']].setVisible(False)
    #            self.bottomSquare[box['boxNum']].setVisible(False)
    #    boundaryString += "\n"
    #    if (self.configSent):
    #        print(boundaryString)
    #        self.cThread = sendCommandThread(self.parser,boundaryString)
    #        self.cThread.start(priority=QThread.HighestPriority-2)
    #for tuning before demo start
    def changeBoundaryBox(self):
        #send box values
        numBoxes = 0
        for box in self.boundaryBoxes:
            if(box['checkEnable'].isChecked()):
                numBoxes += 1
        boundaryString = "boundaryBox "
        staticString = "staticBoundaryBox " 
        flip = 1;
        for box in self.boundaryBoxes:
            if(box['checkEnable'].isChecked()):
                for text in box['boundList']:
                    val = text.text()
                    val = val.replace(" ","")
                    try:
                        num=float(val)
                    except:
                        print('nothing here')
                        return
                    boundaryString += text.text() + " "
                    num += flip*0.5
                    flip = flip*-1
                    staticString += str(num) + " "
                self.drawBoundaryBox3d(box['boxNum'])
            else:
                print("Setting box ", box['boxNum'], " invisisble")
                self.boundaryBoxViz[box['boxNum']].setVisible(False)
                self.bottomSquare[box['boxNum']].setVisible(False)
        boundaryString += "\n"
        if (self.configSent):
           print(boundaryString)
           self.cThread = sendCommandThread(self.parser,boundaryString)
           self.cThread.start(priority=QThread.HighestPriority-2)
        staticString += "\n"
        self.cfg[self.boundaryLine] = boundaryString
        #self.cfg[self.staticLine] = staticString

    def setBoundaryTextVals(self, profile):
        #update box text values based on config
        for box in self.boundaryBoxes:
            bList = box['boundList']
            bList[0].setText(str(profile['leftX']))
            bList[1].setText(str(profile['rightX']))
            bList[2].setText(str(profile['nearY']))
            bList[3].setText(str(profile['farY']))
            bList[4].setText(str(profile['bottomZ']))
            bList[5].setText(str(profile['topZ']))

    def setBoundaryTextVals2(self,bbox_data):
        bList = self.boundaryBoxes[1]['boundList']
        bList[0].setText(str(bbox_data[0]))
        bList[1].setText(str(bbox_data[1]))
        bList[2].setText(str(bbox_data[2]))
        bList[3].setText(str(bbox_data[3]))
        bList[4].setText(str(-2))
        bList[5].setText(str(2))

    def drawBoundaryGrid(self, mRange):
        #re-draw the grid based on boundary box 2-dimensional
        bList = self.boundaryBoxes[self.boxTab.currentIndex()]['boundList']
        xL = mRange*2
        xC = 0
        yL = mRange
        yC = yL/2
        self.gz.resetTransform()
        ####self.gz.setSize(x=mRange*2, y=mRange)
        #self.gz.translate(0,0,0)
        ####self.gz.translate(dx=xC, dy=yC, dz=-2)
        if (self.orientationSelection.currentText() == 'Overhead Mount'):
            #self.gz.setSize(x=mRange*2, y=mRange*2)
            self.gz.setSize(x=8, y=8)
            self.gz.translate(dx=xC, dy=0, dz=-2)
        else:
            self.gz.setSize(x=mRange*2, y=mRange)
            #self.gz.translate(0,0,0)
            self.gz.translate(dx=xC, dy=yC, dz=-2)

    def drawBoundaryBox3d(self, index):
        #print(index)
        bList = self.boundaryBoxes[index]['boundList']
        xl = float(bList[0].text())
        xr = float(bList[1].text())
        yl = float(bList[2].text())
        yr = float(bList[3].text())
        #####zl = float(bList[4].text())
        #####zr = float(bList[5].text())
        if (self.orientationSelection.currentText() == 'Side Mount'):
            zl = float(bList[4].text())-self.profile['sensorHeight'] #set z low of bbox to world coords
            zr = float(bList[5].text())-self.profile['sensorHeight'] #set z hi of bbox to world coords
            print('Sensor Height = ',str(self.profile['sensorHeight']))
        elif (self.orientationSelection.currentText() == 'Overhead Mount'):
            zl = float(bList[4].text())-self.profile['sensorHeight']
            zr = float(bList[5].text())-self.profile['sensorHeight']
        print(xl,yl,zl,xr,yr,zr)

        self.bbox = [xl, xr, yl, yr, zl, zr]
        boxLines = getBoxLines(xl,yl,zl,xr,yr,zr)
        squareLine = getSquareLines(xl,yl,xr,yr,zl)
        if (self.boundaryBoxViz[index].visible() == False):
            print ("Setting Box ", str(index), " to visible")
            self.boundaryBoxViz[index].setVisible(True)
            self.bottomSquare[index].setVisible(True)
        self.boundaryBoxViz[index].setData(pos=boxLines,color=pg.glColor('r'),width=2,antialias=True,mode='lines')
        self.bottomSquare[index].setData(pos=squareLine,color=pg.glColor('b'),width=2,antialias=True,mode='line_strip')
        print('Drew both boxes')

    def colorGradient(self):
        self.gw = pg.GradientWidget(orientation='right')
        self.gw.restoreState(self.gradientMode)

    def swapOrientations(self):
        print('orientation changed')
        print(self.orientationSelection.currentText())
        dsh = self.profile['sensorHeight']
        if (self.orientationSelection.currentText() == 'Overhead Mount'):
            self.yzFlip = 1
            self.evmBox.rotate(90,1,0,0)
            self.gz.resetTransform()
            #self.gz.setSize(x=self.profile['maxRange']*2, y=self.profile['maxRange']*2)
            self.gz.setSize(x=8, y=8)
            #self.gz.translate(0,0,0)
            self.gz.translate(dx=0, dy=0, dz=-self.profile['sensorHeight'])
            self.changeBoundaryBox()
            self.zRange = [-1*self.profile['sensorHeight'], 0]
        elif (self.orientationSelection.currentText() == 'Side Mount'):
            self.yzFlip = 0
            self.evmBox.rotate(-90,1,0,0)
            self.gz.resetTransform()
            self.gz.setSize(x=self.profile['maxRange']*2, y=self.profile['maxRange'])
            #self.gz.translate(0,0,0)
            self.gz.translate(dx=0, dy=self.profile['maxRange']/2, dz=-self.profile['sensorHeight'])
            self.changeBoundaryBox()
            self.zRange = [-3, 3]

    def plot3DQTGraph(self):
        sphereDebug = 0
        self.pcplot = gl.GLViewWidget()
        dummy = np.zeros((1,3))
        #use if need to debug the ellipsoid drawing
        if (sphereDebug == 1):
            colorArray = ('r','g','b','w','y')
            colors = np.zeros((42,4))
            for c in range(0,7):
                colors[c*6:c*6+6,:] = pg.glColor(colorArray[c%5])
            sphereTrigs = getSphereMesh()
            self.sphere =gl.GLMeshItem(vertexes=sphereTrigs,smooth=False,drawEdges=True,edgeColor=pg.glColor('w'),drawFaces=False)
            self.pcplot.addItem(self.sphere)
        # create the background grids
        self.gz = gl.GLGridItem()
        self.gz.translate(0, 0, -1*self.profile['sensorHeight'])
        self.boundaryBoxViz = [gl.GLLinePlotItem(), gl.GLLinePlotItem()]
        self.bottomSquare = [gl.GLLinePlotItem(), gl.GLLinePlotItem()]
        for box in self.boundaryBoxViz:
            box.setVisible(False)
        self.scatter = gl.GLScatterPlotItem(size=5)
        self.scatter.setData(pos=dummy)
        self.pcplot.addItem(self.gz)
        self.pcplot.addItem(self.boundaryBoxViz[0])
        self.pcplot.addItem(self.boundaryBoxViz[1])
        self.pcplot.addItem(self.bottomSquare[0])
        self.pcplot.addItem(self.bottomSquare[1])
        self.pcplot.addItem(self.scatter)
        #create box to represent device
        verX = 0.0625
        verY = 0.05
        verZ = 0.125
        verts = np.empty((2,3,3))
        verts[0,0,:] = [-verX, 0, verZ]
        verts[0,1,:] = [-verX,0,-verZ]
        verts[0,2,:] = [verX,0,-verZ]
        verts[1,0,:] = [-verX, 0, verZ]
        verts[1,1,:] = [verX, 0, verZ]
        verts[1,2,:] = [verX, 0, -verZ]
        self.evmBox = gl.GLMeshItem(vertexes=verts,smooth=False,drawEdges=True,edgeColor=pg.glColor('r'),drawFaces=False)
        self.pcplot.addItem(self.evmBox)
        #add mesh objects for ellipsoids
        self.coordStr = []
        self.ellipsoids = []
        #self.dataImages = []
        edgeColor = pg.glColor('k')
        for m in range(0,20):
            #zeroEllipsoid = getSphereMesh(xc=100*m,yc=100*m,zc=100*m,xRadius=0.1,yRadius=0.1,zRadius=0.1)
            #mesh = gl.GLMeshItem()
            #mesh.setMeshData(vertexes=zeroEllipsoid,smooth=False,drawEdges=True,edgeColor=edgeColor,drawFaces=False)
            mesh = gl.GLLinePlotItem()
            mesh.setVisible(False)
            self.pcplot.addItem(mesh)
            self.ellipsoids.append(mesh)
            #text = GLTextItem()
            #text.setGLViewWidget(self.pcplot)
            #text.setVisible(False)
            #self.pcplot.addItem(text)
            #self.coordStr.append(text)
            
    def updateGraph(self, parsedData):
        updateStart = int(round(time.time()*1000))
        self.useFilter = 0
        classifierOutput = []
        pointCloud = parsedData[0]
        targets = parsedData[1]
        #print(str(targets))
        #print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        indexes = parsedData[2]
        numPoints = parsedData[3]
        numTargets = parsedData[4]
        self.numTargets_display = numTargets
        self.frameNum = parsedData[5]
        fail = parsedData[6]
        classifierOutput = parsedData[7]
        VS = parsedData[8]
        ADCRAW = parsedData[9]
        fallDetEn = 0
        indicesIn = []

        #print('graph numPoints = ',numPoints)
        #print('graph point cloud X = ',pointCloud[0,0])
        #print('graph point cloud Y = ',pointCloud[1,0])
        #print('graph point cloud Z = ',pointCloud[2,0])
        
        #pass target XYZ vals and rotate due to elevation tilt angle (rotX uses Euler rotation around X axis)
        #print('elev_tilt = ',self.profile['elev_tilt'])
        #print('targets = ',targets)
        rotTargetDataX,rotTargetDataY,rotTargetDataZ = rotX (targets[1],targets[2],targets[3],-1*self.profile['elev_tilt'])
        #print('Rotated Data TID,X,Y = ' +str(rotTargetDataX)+', '+str(rotTargetDataY)+', '+str(rotTargetDataZ))
        targets[1] = rotTargetDataX
        targets[2] = rotTargetDataY
        targets[3] = rotTargetDataZ
        
        #pass pointCloud XYZ vals and rotate due to elevation tilt angle (rotX uses Euler rotation around X axis)
        for i in range(numPoints):
            #print('graph point cloud pt = ',pointCloud[:,i])
            #print('graph point cloud Y = ',pointCloud[1][i])
            #print('graph point cloud Z = ',pointCloud[2][i])
            rotPointDataX,rotPointDataY,rotPointDataZ = rotX ([pointCloud[0,i]],[pointCloud[1,i]],[pointCloud[2,i]],-1*self.profile['elev_tilt'])
            #print('graph point cloud rotated pt = ',rotPointDataX,rotPointDataY,rotPointDataZ)
            #print('graph point cloud Y = ',pointCloud[1][i])
            #print('graph point cloud Z = ',pointCloud[2][i])
            pointCloud[0,i] = rotPointDataX
            pointCloud[1,i] = rotPointDataY
            pointCloud[2,i] = rotPointDataZ

        if (fail != 1):
            #left side
            pointstr = 'Points: '+str(numPoints)
            targetstr = 'Targets: '+str(numTargets)
            self.numPointsDisplay.setText(pointstr)
            self.numTargetsDisplay.setText(targetstr)
            #right side fall detection
            peopleStr = 'Number of Detected People: '+str(numTargets)
            if (numTargets == 0):

                fdestr = 'Fall Detection Disabled - No People Detected'
            elif (numTargets == 1):

                fdestr = 'Fall Detection Enabled'
                fallDetEn = 1
            elif (numTargets > 1):
                fdestr = 'Fall Detected Disabled - Too Many People'
            #self.numDetPeople.setText(peopleStr)
            #self.fallDetEnabled.setText(fdestr)
        if (len(targets) < 13):
            targets = []
            classifierOutput = []
        if (fail):
            return
        #remove static points
        #check for mounting position
        if (self.yzFlip == 1):
            pointCloud[[1, 2]] = pointCloud[[2, 1]]
            pointCloud[2,:] = -1*pointCloud[2,:]
            targets[[2,3]] = targets[[3,2]]
            targets[3,:] = -1*targets[3,:]
        #remove static points
        if (self.configType.currentText() == '3D People Counting' or self.configType.currentText() == 'Capon3DAOP' or self.configType.currentText() == 'Sense and Detect HVAC Control' or self.configType.currentText() == '60Low EVM' or self.configType.currentText() == "Vital Signs"):
            if (not self.staticclutter.isChecked()):
                statics = np.where(pointCloud[3,:] == 0)
                try:
                    firstZ = statics[0][0]
                    numPoints = firstZ
                    pointCloud = pointCloud[:,:firstZ]
                    indexes = indexes[:,:self.previousFirstZ]
                    self.previousFirstZ = firstZ
                except:
                    firstZ = -1
        #point cloud persistence
        fNum = self.frameNum%10
        if (numPoints):
            self.previousCloud[:5,:numPoints,fNum] = pointCloud[:5,:numPoints]
            self.previousCloud[5,:len(indexes),fNum] = indexes
        self.previousPointCount[fNum]=numPoints
        #plotting 3D - get correct point cloud (persistent points and synchronize the frame)
        if (self.configType.currentText() == 'SDK3xPeopleCount'):
            pointIn = pointCloud
        #elif (self.configType.currentText() == 'Capon3D' or self.configType.currentText() == 'Capon3DAOP'):
        #    totalPoints = 0
        #    persistentFrames = int(self.persistentFramesInput.currentText())
        #    #allocat new array for all the points
        #    for i in range(0,persistentFrames):
        #        totalPoints += self.previousPointCount[fNum-i]
        #    pointIn = np.zeros((5,int(totalPoints)))
        #    indicesIn = np.ones((1, int(totalPoints)))*255
        #    totalPoints = 0
        #    #fill array
        #    for i in range(0,persistentFrames):
        #        prevCount = int(self.previousPointCount[fNum-i])
        #        pointIn[:,totalPoints:totalPoints+prevCount] = self.previousCloud[:,:prevCount,fNum-i]
        #        if (numTargets > 0):
        #            indicesIn[0,totalPoints:totalPoints+prevCount] = self.previousIndex[0,:prevCount,fNum-i]
        #        totalPoints+=prevCount
        else:
            totalPoints = 0
            persistentFrames = int(self.persistentFramesInput.currentText())
            #allocat new array for all the points
            for i in range(1,persistentFrames+1):
                totalPoints += self.previousPointCount[fNum-i]
            pointIn = np.zeros((5,int(totalPoints)))
            indicesIn = np.ones((1, int(totalPoints)))*255
            totalPoints = 0
            #fill array
            for i in range(1,persistentFrames+1):
                prevCount = int(self.previousPointCount[fNum-i])
                pointIn[:,totalPoints:totalPoints+prevCount] = self.previousCloud[:5,:prevCount,fNum-i]
                if (numTargets > 0):
                    indicesIn[0,totalPoints:totalPoints+prevCount] = self.previousCloud[5,:prevCount,fNum-i]
                totalPoints+=prevCount
        if (self.graphFin):
            self.plotstart = int(round(time.time()*1000))
            self.graphFin = 0
            if (self.threeD):
                try:
                    indicesIn = indicesIn[0,:]
                except:
                    indicesIn = []
                self.get_thread = updateQTTargetThread3D(pointIn, targets, indicesIn, self.scatter, self.pcplot, numTargets, self.ellipsoids, self.coordStr, classifierOutput, self.zRange, self.gw, self.plotByIndex.isChecked(), self.plotTracks.isChecked(), self.bbox,self.boundaryBoxes[0]['checkEnable'].isChecked())
                self.get_thread.done.connect(self.graphDone)
                self.get_thread.start(priority=QThread.HighestPriority-1)
            else:
                npc = pointIn[0:2,:]
                print (np.shape(npc))
                self.legacyThread = update2DQTGraphThread(npc, targets, numTargets, indexes, numPoints, self.trailData, self.activeTrail, self.trails, self.scatter2D, self.gatingScatter)
                self.legacyThread.done.connect(self.graphDone)
                self.legacyThread.start(priority=QThread.HighestPriority-1)
        else:
            return
        #pointIn = self.previousCloud[:,:int(self.previousPointCount[fNum-1]),fNum-1]

        #state tracking
        if (numTargets > 0):
            self.lastFrameHadTargets = True
        else:
            self.lastFrameHadTargets = False
        if (numTargets):
            self.lastTID = targets[0,:]
        else:
            self.lastTID = []
        


        # added by Alex 11/2021 to only update graphs if there is a detected target else the graphs will freeze and not update

        #########################################
        vital_sign_enable = 0
        
        if(self.bbox_2_en):
            for i in range(numTargets):
                x = targets[1,i]
                y = targets[2,i]
                z = targets[3,i]
                if (not(x < self.bbox_2[0] or x > self.bbox_2[1]) and not(y < self.bbox_2[2] or y > self.bbox_2[3]) and not(z < self.bbox_2[4] or z > self.bbox_2[5])):
                    vital_sign_enable = 1
                    print("Target detected in special region")
        
        #print("No target detected")
        
        #if (self.hGraphFin and (vital_sign_enable == 1)): // include if you want second boundary box check

        #########################################

        # added by Alex 12/2021 extra logic to only update graphs if numTargets is greater than 0 

        if (self.hGraphFin and (numTargets > 0)):
            self.hGraphFin = 0
            #self.heightThread = updateHeightGraphs(self.targetSize, self.heightPlots, self.frameNum, self.lastTID)



            self.heightThread = updateHeightGraphs(VS, self.heightPlots, self.frameNum,ADCRAW)
            self.heightThread.done.connect(self.heightGraphDone)
            self.heightThread.start(priority=QThread.HighestPriority-2)
        elif(self.hGraphFin and (numTargets == 0)):
            self.hGraphFin = 0
            self.heightThread = zeroHeightGraphs(VS, self.heightPlots, self.frameNum,ADCRAW)
            self.heightThread.done.connect(self.heightGraphDone)
            self.heightThread.start(priority=QThread.HighestPriority-2)  


    def graphDone(self):
        plotend = int(round(time.time()*1000))
        plotime = plotend - self.plotstart
        try:
            if (self.frameNum > 1):
                self.averagePlot = (plotime*1/self.frameNum) + (self.averagePlot*(self.frameNum-1)/(self.frameNum))
            else:
                self.averagePlot = plotime
        except:
            self.averagePlot = plotime
        self.graphFin = 1
        pltstr = 'Average Plot time: '+str(plotime)[:5] + ' ms'
        fnstr = 'Frame: '+str(self.frameNum)
        self.frameNumDisplay.setText(fnstr)
        self.plotTimeDisplay.setText(pltstr)
        
    def list_move_left(A,a):
        for i in range(a):
            A.insert(len(A),A[0])
            A.remove(A[0])
        return A
    def shift(xs, n):
        if n >= 0:
            return np.r_[np.full(n, np.nan), xs[:-n]]
        else:
            return np.r_[xs[-n:], np.full(-n, np.nan)]
    

        
    def heightGraphDone(self, toPlot):
        self.hGraphFin = 1
        x=np.arange(0,286)
        x1=np.arange(0,512)
        
        self.wave0 = np.roll(self.wave0,-1)
        self.wave1 = np.roll(self.wave1,-1)
        self.heart0 = np.roll(self.heart0,-1)
        self.heart1 = np.roll(self.heart1,-1)
        self.breath0 = np.roll(self.breath0,-1)
        #self.breath1 = np.roll(self.breath1,-1)
        self.wave0[285] = toPlot['wave0']
        self.wave1[285] = toPlot['wave1']
        self.heart0[285] = toPlot['heart0']
        self.heart1[285] = toPlot['heart1']
        self.breath0[285] = toPlot['breath0']
        #self.breath1[285] = toPlot['breath1']
        #self.breath1[285] = toPlot['breathing_rate0']
        #print(toPlot['test'])    
        #print(toPlot['test'])
        self.breath1[0:674] = toPlot['test']
        # self.breath1[128:256] = toPlot['test']
        # self.breath1[286-128:286] = toPlot['test']
        # print(self.wave0)
        # print(self.breath1)
        
        self.ADCRAW = toPlot['ADCRAW']
        # print(self.ADCRAW)
        adcraw_abs=np.zeros(128)
        
        if self.StartRecordFlag == 1:
            self.RecordButton.setText('Recording')
        else:
            self.RecordButton.setText('Press to Record')

        ####threshold_log = math.log(toPlot['id0'],2)
        # print(threshold_log)
        #if(threshold_log<32.0):
        # if(self.numTargets_display==0):
            # toPlot['breathing_rate0'] = 0
            # self.breathrate_final = 0
            # self.heartrate_final = 0
            # toPlot['heart_rate0'] = 0
            # toPlot['id1'] = 0
            # toPlot['x0'] = 0
            # toPlot['y0'] = 0
            # toPlot['z0'] = 0
            # toPlot['rangeidx0'] = 0
            # toPlot['x1'] = 0
            # toPlot['y1'] = 0
            # toPlot['z1'] = 0
            # toPlot['rangeidx1'] = 0
            # toPlot['angle1'] = 0
            # self.breath1[0:674] = 0
            # toPlot['range1'] = 0
            # toPlot['rangeidx1'] = 0

        # for i in range(0,8):
            # for j in range(0,5):
                # adcraw_abs[i*5+j] = self.ADCRAW[i+j*8,0]*self.ADCRAW[i+j*8,0] + self.ADCRAW[i+j*8,1]*self.ADCRAW[i+j*8,1]
            # self.adcraw_abs_buffer[i,self.frame_counter]=adcraw_abs[i]
            # adcraw_abs_var_threshold[i] = 400000
            # adcraw_abs[i] = math.log(adcraw_abs[i],2)
       
            #*self.ADCRAW(i,0) + self.ADCRAW(i,1)*self.ADCRAW(i,1)
        
        # print(adcraw_abs_var)
        # print(adcraw_abs)
        # print(self.frame_counter)
        if self.frame_counter2 < self.profile['refresh_frame_count']:
            self.frame_counter2 = self.frame_counter2 + 1
        else:
            self.frame_counter2 = 0
            # str8 = 'Frame: ' + str(int(self.frame_counter)) +'\tRangeIdx: '+str(int(toPlot['rangeidx0'])) +'\tRange: '+str('%.2f'%(toPlot['rangeidx0']*0.0632)) + 'm\tBreath: ' + str('%.2f'%toPlot['breathing_rate0']) +'\tBreath Vote: '+('%.2f%%'%(toPlot['rangeidx1']/0.45)) + '\tHeart: '+str('%.2f'%toPlot['heart_rate0']) + '\tHeart Vote: '+('%.2f%%'%(toPlot['range1']/0.45))+ '\tBreath_deviation: '+('%.2f'%(toPlot['angle1']))+' '
            # print(str8)
            
            if self.filter_counter < 10:
                self.breathrate_buf[self.filter_counter,0] = float(toPlot['breathing_rate0']);
                self.heartrate_buf[self.filter_counter,0] = float(toPlot['heart_rate0']);
                self.filter_counter = self.filter_counter + 1
                self.breathrate_final = float(toPlot['breathing_rate0']);
                self.heartrate_final = float(toPlot['heart_rate0']); 
                # print(self.heartrate_buf[0:9,0])
            else:
                # self.breathrate_buf.insert(len(self.breathrate_buf),self.breathrate_buf[0])
                # self.breathrate_buf.remove(self.breathrate_buf[0])
                # self.heartrate_buf.insert(len(self.heartrate_buf),self.breathrate_buf[0])
                # self.heartrate_buf.remove(self.heartrate_buf[0])
                # self.heartrate_buf[1:9] = self.heartrate_buf[2:10]
                # self.shift(self.breathrate_buf[1:10],-1);
                # self.shift(self.heartrate_buf[1:10],-1);
                self.breathrate_buf[self.filter_counter,0] = float(toPlot['breathing_rate0']);
                self.heartrate_buf[self.filter_counter,0] = float(toPlot['heart_rate0']);
                # print(self.filter_counter)
                self.breathrate_buf[0:10,0] = self.breathrate_buf[1:11,0]
                # self.breathrate_buf[9,0]=float(toPlot['breathing_rate0']);
                # self.breathrate_buf[0:9,0]=self.buf_temp[0:9,0]
                
                # self.heartrate_last = statistics.median(self.heartrate_buf[0:10]); 
                self.heartrate_buf[0:10,0] = self.heartrate_buf[1:11,0]
                # self.heartrate_buf[9,0]=float(toPlot['heart_rate0']);
                # self.heartrate_buf[0:9,0]=self.buf_temp[0:9,0]
                # self.breathrate_buf[10] = self.frame_counter#float(toPlot['breathing_rate0']);
                # self.heartrate_buf[10] = self.frame_counter#float(toPlot['heart_rate0']);
                # print(self.breathrate_buf[0:10,0])
                # print(self.heartrate_buf[0:10,0])
                self.breathrate_final = statistics.median(self.breathrate_buf[0:10]);
                self.heartrate_final = statistics.median(self.heartrate_buf[0:10]);   
                if self.heartrate_last - self.heartrate_final > 8:
                    self.heartrate_final = self.heartrate_last
                self.heartrate_last = self.heartrate_final
                if self.breathrate_last - self.breathrate_final > 5:
                    self.breathrate_final = self.breathrate_last
                self.breathrate_last = self.breathrate_final
                str8 = 'Breath: ' + str(self.breathrate_final) +'\tHeart: '+str(self.heartrate_final)+' '
                # print(str8)
            
            self.now_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
            str8 = 'Time: ' + self.now_time + '\tFrame: ' + str(int(self.frame_counter+1)) +'\tRangeIdx: '+str(int(toPlot['rangeidx0'])) +'\t\tRange: '+str('%.2f'%(toPlot['rangeidx0']*0.0632)) + 'm\tBreath_RAW: ' + str(int(toPlot['breathing_rate0']))+'\tHeart_RAW: ' + str(int(toPlot['heart_rate0']))+             '\tBreath_filter: ' + str('%.2f'%float(self.breathrate_final)) +'\tBreath Vote: '+('%.2f%%'%(toPlot['rangeidx1']/0.45)) + '\tHeart_filter: '+str('%.2f'%float(self.heartrate_final)) + '\tHeart Vote: '+('%.2f%%'%(toPlot['range1']/0.45))+ '\tBreath_deviation: '+('%.2f'%(toPlot['angle1']))+' \n'
            print(str8)
            ###print('threshold_log: '+str(threshold_log))
            
            str8 = 'Time: ' + self.now_time + '\tFrame: ' + str(int(self.frame_counter+1)) +'\tRangeIdx: '+str(int(toPlot['rangeidx0'])) +'\t\tRange: '+str('%.2f'%(toPlot['rangeidx0']*0.0632)) + 'm\tBreath_RAW: ' + str(int(toPlot['breathing_rate0']))+'\tHeart_RAW: ' + str(int(toPlot['heart_rate0']))+             '\tBreath_filter: ' + str('%.2f'%float(self.breathrate_final)) +'\tBreath Vote: '+('%.2f%%'%(toPlot['rangeidx1']/0.45)) + '\tHeart_filter: '+str('%.2f'%float(self.heartrate_final)) + '\tHeart Vote: '+('%.2f%%'%(toPlot['range1']/0.45))+ '\tBreath_deviation: '+('%.2f'%(toPlot['angle1']))+'\tBreath_rate_array:' + str(list(np.around(self.breath1[456:501], decimals=2)))+'\tHeart_rate_array:' + str(list(np.around(self.breath1[501:546], decimals=2)))+' \n'
            # str9 = 'Breath rate' + str(list(np.around(self.breath1[456:500], decimals=2)))
            
            # print(str9)
            # self.WriteFile2(str8)
            # self.WriteFile2(str8)
            # print(self.breath1[456:501])
            # print(self.breath1[501:544])
            # print(self.breath1[544:546])
            x_idx=self.breath1[1]
            y_idx=self.breath1[2]
            # print(x_idx)
            # print(y_idx)
            
            # a = np.reshape(self.breath1[200:456],(16,16))
            # testfig = plt.figure(2)
            # testfig = plt.ion()
            # testfig = plt.clf()
            # testfig = plt.title("2D heatmap(W/O fftshift)")
            # # for t in range(0,10):
            # # testfig = plt.colorbar()
            # # a[1][1] += int(self.frame_counter)
            # testfig = plt.imshow(a)
            # testfig = plt.plot(y_idx, x_idx, 'r*')
            # testfig = plt.show()
            
        self.frame_counter=self.frame_counter+1
        # if self.frame_counter == 100 :
            # # self.frame_counter = 0
            # adcraw_abs_var = (np.std(self.adcraw_abs_buffer, axis = 1))
            # # print(adcraw_abs_var)
            # # print(adcraw_abs)
            # # self.heightPlots['heart1_test'].setData(x1[:100],adcraw_abs_var[:100])
            # self.heightPlots['breath1'].setData(x1[:128],self.breath1[:128])
            # # self.heightPlots['breath1_test'].setData(x1[:100],self.adcraw_abs_var_threshold[:100])
            # if self.StartRecordFlag == 1:
                # for i in range(0,128):
                    # self.adcraw_abs_var_threshold[i] = adcraw_abs_var[i]*1.5
                # self.heightPlots['breath1_test'].setData(x1[:100],self.adcraw_abs_var_threshold[:100])
                # self.StartRecordFlag = 0


        self.breath2 = toPlot['test']
        
        # self.heightPlots['breath0'].setData(x,self.breath0)
        #self.heightPlots['breath1'].setData(x,toPlot['test'])
        # self.heightPlots['breath1'].setData(x,self.breath1)
        # self.heightPlots['breath1'].setData(x1,self.breath2)
        # self.heightPlots['heart0'].setData(x,self.heart0)
        self.heightPlots['heart0'].setData(x1[3:98],self.breath1[3:98])
        # print('BREATH')
        # print(self.breath1[1:99])
        # self.heightPlots['heart1'].setData(x,self.heart1)
        # if self.frame_counter % 10 == 0:
        # self.heightPlots['breath0'].setData(x1[1:300],self.breath1[129:428])
        self.heightPlots['breath0'].setData(x1[3:98],self.breath1[103:198])
        # print('HEART')
        # print(self.breath1[101:199])
        # print(self.breath1[101:145])
        # print(self.breath1[151:299])
        # self.heightPlots['breath0'].setData(x1[1:50],self.breath1[129:178])
        #print(self.breath1[1:256])
        self.heightPlots['breath1'].setData(x1[0:60],self.breath1[547:547+60])
        # self.heightPlots['breath1_test'].setData(x1[10],self.breath1[546+10:546+12])
        self.heightPlots['breath1_test2'].setData(x=[x1[int(toPlot['rangeidx0'])]],y=[self.breath1[547+int(toPlot['rangeidx0'])]])
        self.heightPlots['heart1'].setData(x1[1:45],self.breath1[456:500])
        self.heightPlots['heart1_test'].setData(x1[1:43],self.breath1[501:543])



                    
        str0 = 'Breath: ' + str(int(toPlot['breathing_rate0']))
        # str1 = 'Breath: '+str(int(toPlot['breathing_rate1']))+'\r\nHeart: '+str(int(toPlot['heart_rate1']))
        str1 = 'Breath: '+str('%.2f'%float(self.breathrate_final))
        str2 = 'Heart: '+str(int(toPlot['heart_rate0']))
        #str3 = 'Fall flag = '+str(int(toPlot['id1']))
        str3 = 'Heart: '+str('%.2f'%float(self.heartrate_final))
        
        
        # str4 = 'XYZ: '+('%.1f'%toPlot['x0']) + ',' +('%.1f'%toPlot['y0']) + ','+('%.1f'%toPlot['z0'])+' '
        str4 = 'breath_idx: '+('%.1f'%toPlot['x0']) + ',\r\nheart_idx: ' +('%.1f'%toPlot['y0']) + ',\r\nangle_idx: '+('%.1f'%toPlot['z0'])+' '
        str5 = 'RIdx: '+str(int(toPlot['rangeidx0']))
        # str6 = 'XYZ: '+('%.2f'%toPlot['x1']) + ',' +('%.2f'%toPlot['y1']) + ','+('%.2f'%toPlot['z1'])+' '
        str6 = 'X-m: '+('%.2f'%toPlot['x1']) + ',\r\nY-m: ' +('%.2f'%toPlot['y1']) + ',\r\nZ-m: '+('%.2f'%toPlot['z1']) + ',\r\nHeart Vote: '+('%.2f'%toPlot['range1'])+',\r\nBreath Vote: '+('%.2f'%toPlot['rangeidx1'])+' '
        
        # if(int(toPlot['z1']) < 500000):
            # str7 = 'No presence'
            # ft5=QFont()
            # ft5.setPointSize(12)
            # self.range1.setFont(ft5)
            # str1 = 'Breath: NA'+'\r\nHeart: NA'+'\r\nNo presence'
        # el
        str7 = 'Presence'
        if(toPlot['angle1'] == 0):
            str7 = 'No presence'
            ft5=QFont()
            ft5.setPointSize(12)
            self.range1.setFont(ft5)
            str1 = ' '
            str3 = ''
        elif(toPlot['angle1'] < 0.02):
            str7 = 'Hold Breath'
            ft5=QFont()
            ft5.setPointSize(12)
            self.range1.setFont(ft5)
            #str1 = 'Breath: 0'+'\r\nHeart: 0'+'\r\nHold Breath'
            str1 =  'Hold Breath'
            str3 = ''
        elif(toPlot['angle1'] >= 0.02):
            str7 = 'Presence'
            ft5=QFont()
            ft5.setPointSize(12)
            self.range1.setFont(ft5)
        # str7 = 'Presence'
        ft5=QFont()
        ft5.setPointSize(12)
        self.range1.setFont(ft5)
        # if(int(toPlot['rangeidx1']) == 1):
            # str7 = 'Reason = '+str(int(toPlot['rangeidx1']))+' Energy Breath'
            # ft5=QFont()
            # ft5.setPointSize(12)
            # self.range1.setFont(ft5)
        # elif(int(toPlot['rangeidx1']) == 2):
            # str7 = 'Reason = '+str(int(toPlot['rangeidx1']))+' Energy peakabs'        
            # ft5=QFont()
            # ft5.setPointSize(12)
            # self.range1.setFont(ft5)
        # elif(int(toPlot['rangeidx1']) == 3):
            # str7 = 'Reason = '+str(int(toPlot['rangeidx1']))+' Energy Heart'        
            # ft5=QFont()
            # ft5.setPointSize(12)
            # self.range1.setFont(ft5)
        # elif(int(toPlot['rangeidx1']) == 4):
            # str7 = 'Reason = '+str(int(toPlot['rangeidx1']))+' Peakabs Var'        
            # ft5=QFont()
            # ft5.setPointSize(12)
            # self.range1.setFont(ft5)
        # elif(int(toPlot['rangeidx1']) == 5):
            # str7 = 'Reason = '+str(int(toPlot['rangeidx1']))+' CFAR'        
            # ft5=QFont()
            # ft5.setPointSize(12)
            # self.range1.setFont(ft5)
        # elif(int(toPlot['rangeidx1']) == 6):
            # str7 = str(int(toPlot['rangeidx1']))+': Hold Breath'        
            # ft5=QFont()
            # ft5.setPointSize(20)
            # self.range1.setFont(ft5)
            # #self.range0.setText(str7)
        # elif(int(toPlot['rangeidx1']) == 0):
            # str7 = 'Reason = '+str(int(toPlot['rangeidx1']))    
            # ft5=QFont()
            # ft5.setPointSize(12)
            # self.range1.setFont(ft5)
        # else:
            # str7 = 'Reason = Others'    
            # ft5=QFont()
            # ft5.setPointSize(12)
            # self.range1.setFont(ft5)
        self.breathrate0.setText(str0)
        self.breathrate1.setText(str1)
        self.heartrate0.setText(str2)
        self.heartrate1.setText(str3)
        self.xyz0.setText(str4)
        self.xyz1.setText(str6)
        self.range0.setText(str5)
        self.range1.setText(str7)



    def resetFallText(self):
        self.fallAlert.setText('Standing')
        self.fallPic.setPixmap(self.standingPicture)
        self.fallResetTimerOn = 0

    def updateFallThresh(self):
        try:
            newThresh = float(self.fallThreshInput.text())
            self.fallThresh = newThresh
            self.fallThreshMarker.setPos(self.fallThresh)
        except:
            print('No numberical threshold')

    def connectCom(self):
        #get parser
        self.parser = uartParserSDK(type=self.configType.currentText())
        self.parser.frameTime = self.frameTime
        print('Parser type: ',self.configType.currentText())
        #init threads and timers
        self.uart_thread = parseUartThread(self.parser)
        if (self.configType.currentText() != 'Replay'):
            self.uart_thread.fin.connect(self.parseData)
        self.uart_thread.fin.connect(self.updateGraph)
        self.parseTimer = QTimer()
        self.parseTimer.setSingleShot(False)
        self.parseTimer.timeout.connect(self.parseData)        
        try:
            uart = "COM"+ self.uartCom.text()       #deb_gp
            data = "COM"+ self.dataCom.text()       #deb_gp
#TODO: find the serial ports automatically.
            self.parser.connectComPorts(uart, data)
            self.connectStatus.setText('Connected')     #deb_gp
            self.connectButton.setText('Disconnect')    #deb_gp
#TODO: create the disconnect button action
        except:
            self.connectStatus.setText('Unable to Connect')
        if (self.configType.currentText() == "Replay"):
            self.connectStatus.setText('Replay')
    def RecordStd(self):
        self.StartRecordFlag = 1
        self.frame_counter = 0

#
# Select and parse the configuration file
# TODO select the cfgfile automatically based on the profile.
    def selectCfg(self):
        try:
            self.parseCfg(self.selectFile())
        except:
            print('No cfg file selected!')
    def selectFile(self):
        fd = QFileDialog()
        filt = "cfg(*.cfg)"
        filename = fd.getOpenFileName(directory='./chirp_configs',filter=filt)    #deb_gp - added folder name
        return filename[0]
    def parseCfg(self, fname):
        cfg_file = open(fname, 'r')
        self.cfg = cfg_file.readlines()
        counter = 0
        chirpCount = 0
        for line in self.cfg:
            args = line.split()
            if (len(args) > 0):
                if (args[0] == 'cfarCfg'):
                    zy = 4
                    #self.cfarConfig = {args[10], args[11], '1'}
                elif (args[0] == 'AllocationParam'):
                    zy=3
                    #self.allocConfig = tuple(args[1:6])
                elif (args[0] == 'GatingParam'):
                    zy=2
                    #self.gatingConfig = tuple(args[1:4])
                elif (args[0] == 'SceneryParam' or args[0] == 'boundaryBox'):
                    self.boundaryLine = counter
                    self.profile['leftX'] = float(args[1])
                    self.profile['rightX'] = float(args[2])
                    self.profile['nearY'] = float(args[3])
                    self.profile['farY'] = float(args[4])
                    if (self.configType.currentText() == '3D People Counting' or self.configType.currentText() == '60Low EVM' or self.configType.currentText() == "Vital Signs"):
                        self.profile['bottomZ'] = float(args[5])
                        self.profile['topZ'] = float(args[6])
                    else:
                        self.profile['bottomZ'] = float(-3)
                        self.profile['topZ'] = float(3)
                    self.setBoundaryTextVals(self.profile)
                    self.boundaryBoxes[0]['checkEnable'].setChecked(True)
                elif(args[0] == 'boundaryBox2'):


                    x1 = float(args[1])
                    xr = float(args[2])
                    y1 = float(args[3])
                    yr = float(args[4])
                    z1 = float(args[5])
                    zr = float(args[6])
                    #self.boundaryBoxes[1]['checkEnable'].setChecked(True)

                    self.bbox_2 = [float(args[1]),float(args[2]),float(args[3]),float(args[4]),float(args[5]),float(args[6])]
                    self.bbox_2_en = 1
                    self.setBoundaryTextVals2(self.bbox_2)
                    self.boundaryBoxes[1]['checkEnable'].setChecked(True)

                elif (args[0] == 'staticBoundaryBox'):
                    self.staticLine = counter
                elif (args[0] == 'profileCfg'):
                    self.profile['startFreq'] = float(args[2])
                    self.profile['idle'] = float(args[3])
                    self.profile['adcStart'] = float(args[4])
                    self.profile['rampEnd'] = float(args[5])
                    self.profile['slope'] = float(args[8])
                    self.profile['samples'] = float(args[10])
                    self.profile['sampleRate'] = float(args[11])
                    print(self.profile)
                elif (args[0] == 'frameCfg'):
                    self.profile['numLoops'] = float(args[3])
                    self.profile['numTx'] = float(args[2])+1
                elif (args[0] == 'chirpCfg'):
                    chirpCount += 1
                elif (args[0] == 'sensorPosition'):
                    self.profile['sensorHeight'] = float(args[1])
                    self.profile['az_tilt'] = float(args[2])
                    self.profile['elev_tilt'] = float(args[3])
                elif (args[0] == 'vitalsign'):
                    self.profile['refresh_frame_count'] = float(args[1])
                    self.profile['window_size'] = float(args[2])
            counter += 1
        self.profile['maxRange'] = self.profile['sampleRate']*1e3*0.9*3e8/(2*self.profile['slope']*1e12)
        ####maxRange = self.profile['sampleRate']*1e3*0.9*3e8/(2*self.profile['slope']*1e12)
        #update boundary box
        ####self.drawBoundaryGrid(maxRange)
        self.gz.translate(0, 0, 3-self.profile['sensorHeight']) #reposition the ground level to be at sensor height
        self.changeBoundaryBox() #redraw bbox from cfg file values
        #update chirp table values
        bw = self.profile['samples']/(self.profile['sampleRate']*1e3)*self.profile['slope']*1e12
        rangeRes = 3e8/(2*bw)
        Tc = (self.profile['idle']*1e-6 + self.profile['rampEnd']*1e-6)*chirpCount
        lda = 3e8/(self.profile['startFreq']*1e9)
        maxVelocity = lda/(4*Tc)
        velocityRes = lda/(2*Tc*self.profile['numLoops']*self.profile['numTx'])
        self.configTable.setItem(1,1,QTableWidgetItem(str(self.profile['maxRange'])[:5]))
        self.configTable.setItem(2,1,QTableWidgetItem(str(rangeRes)[:5]))
        self.configTable.setItem(3,1,QTableWidgetItem(str(maxVelocity)[:5]))
        self.configTable.setItem(4,1,QTableWidgetItem(str(velocityRes)[:5]))
        #update sensor position
        self.az_tilt.setText(str(self.profile['az_tilt']))
        self.elev_tilt.setText(str(self.profile['elev_tilt']))
        self.s_height.setText(str(self.profile['sensorHeight']))

    def sendCfg(self):
        try:
            if (self.configType.currentText() != "Replay"):
                self.parser.sendCfg(self.cfg)
                self.configSent = 1
                self.parseTimer.start(self.frameTime)
        except:
            print ('No cfg file selected!')

    def startFunc(self):
        self.configSent = 1
        self.parseTimer.start(self.frameTime)
        print ('Starting Application!')

    # Needed ?? deb_gp
    # def setParser(self, uParser):
    #     self.parser = uParser

    def parseData(self):
        self.uart_thread.start(priority=QThread.HighestPriority)

    def whoVisible(self):
        if (self.threeD):
            self.threeD = 0
        else:
            self.threeD = 1
        print('3d: ', self.threeD)

if __name__ == '__main__':
    if (compileGui):
        appctxt = ApplicationContext()
        app = QApplication(sys.argv)
        screen = app.primaryScreen()
        size = screen.size()
        main = Window(size=size)
        main.show()
        exit_code = appctxt.app.exec_()
        sys.exit(exit_code)
    else:
        app = QApplication(sys.argv)
        screen = app.primaryScreen()
        size = screen.size()
        main = Window(size=size)
        main.show()
        sys.exit(app.exec_())