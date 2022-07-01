import os
import rospy
import rospkg
import json
import numpy
import threading
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from mira_msgs.msg import Results
import time
import any_msgs.srv
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import Qt, Signal, Slot, QTimer
from python_qt_binding.QtGui import QIcon, QPixmap

from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg


class MiraDsPlugin(Plugin):

    def __init__(self, context):
        super(MiraDsPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MiraDsPlugin')

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mira_ds'), 'resource', 'plugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        self.miraAutoAcquireTopicName = "/mira_ds/auto_acquire"
        self.miraAcquireTopicName = "/mira_ds/acquire"
        self.miraConnectTopicName = "/mira_ds/connect"
        self.connect_publisher = rospy.Publisher(self.miraConnectTopicName, Bool, queue_size=1)
        self.acquire_publisher = rospy.Publisher(self.miraAcquireTopicName, Bool, queue_size=1)
        self.autoAcquire_publisher = rospy.Publisher(self.miraAutoAcquireTopicName, Bool, queue_size=1)
        self.pub = rospy.Publisher('/mira_ds/changeLaser', String, queue_size=10)

        rospy.Subscriber("/mira_ds/result", Results, self.result_callback)
        rospy.Subscriber("/mira_ds/result_connect", Bool, self.result_connect_callback)
        rospy.Subscriber("/mira_ds/result_acquire", Bool, self.result_acquire_callback)

        self._widget.pushButton_connect.clicked.connect(self.connect_button_slot)
        self._widget.pushButton_armLaser.clicked.connect(self.armLaser_button_slot)
        self._widget.pushButton_getRange.clicked.connect(self.getRange_button_slot)
        self._widget.pushButton_autofocus.clicked.connect(self.autofocus_button_slot)
        self._widget.pushButton_start.clicked.connect(self.start_button_slot)
        self._widget.checkBox_autofocus.clicked.connect(self.show_autofocus_control)

        self._widget.pushButton_start.setEnabled(False)
        self._widget.pushButton_start.setText("...")

        path_to_error_icon = os.path.join(rospkg.RosPack().get_path('rqt_mira_ds'), 'resource', 'error.png')
        pixmap = QPixmap(path_to_error_icon)
        self._widget.label_armLaser.setPixmap(pixmap)
        self._widget.label_autofocus.setPixmap(pixmap)

        self._widget.groupBox_autofocus.hide()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_slot)
        self.finishedThread = False

        self.received_results = False

        self.scanNumber = 1

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.pub.unregister()
        self.connect_publisher.unregister()
        self.autoAcquire_publisher.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    @Slot() # autofocus control callback function    
    def show_autofocus_control(self):
        if self._widget.checkBox_autofocus.isChecked():
            self._widget.groupBox_autofocus.show()
        else:
            self._widget.groupBox_autofocus.hide()

    @Slot() # Button callback function
    def connect_button_slot(self):
        if self._widget.pushButton_connect.text() == "Connect":
            rospy.loginfo("<<<[Mira_DS_Plugin]>>>  connect_button_slot] publishing")
            self._widget.label_status.setText("connecting...")
            self._widget.pushButton_connect.setEnabled(False)
            message = Bool()
            message.data = True
            self.connect_publisher.publish(message)
            QTimer.singleShot(10000, self.connect_timeout_slot)
        else :
            self._widget.label_status.setText("disconnecting...")
            self._widget.pushButton_connect.setEnabled(False)
            message = Bool()
            message.data = False
            self.connect_publisher.publish(message)
            QTimer.singleShot(10000, self.disconnect_timeout_slot)

    @Slot() # connect timeout callback function
    def connect_timeout_slot(self):
        if not(self._widget.pushButton_connect.isEnabled()):
            self._widget.label_status.setText("Connecting Timeout!")
            self._widget.pushButton_connect.setEnabled(True)

    @Slot() # connect timeout callback function
    def disconnect_timeout_slot(self):
        if not(self._widget.pushButton_connect.isEnabled()):
            self._widget.label_status.setText("Disconnecting Timeout!")
            self._widget.pushButton_connect.setEnabled(True)

    def result_connect_callback(self, message):
        rospy.loginfo("<<<[Mira_DS_Plugin]>>> result_connect_callback!")
        self._widget.pushButton_connect.setEnabled(True)
        if self._widget.pushButton_connect.text() == "Connect":
            if message.data:
                self._widget.pushButton_connect.setText("Disconnect")
                self._widget.label_status.setText("CONNECTED")
                self._widget.pushButton_start.setEnabled(True)
                self._widget.pushButton_start.setText("Start")
            else:
                self._widget.label_status.setText("ERROR connecting")
        else:
            if message.data:
                self._widget.pushButton_connect.setText("Connect")
                self._widget.label_status.setText("DISCONNECTED")
                self._widget.pushButton_start.setEnabled(True)
                self._widget.pushButton_start.setText("Start")
            else:
                self._widget.label_status.setText("ERROR disconnecting")

    @Slot() # Button callback function
    def armLaser_button_slot(self):
        if self._widget.pushButton_armLaser.text() == "Arm Laser":
            self._widget.label_status.setText("arming laser...")
        else:
            self._widget.label_status.setText("disarming laser...")
        self.laser_thread = threading.Thread(target=self.thread_arm_laser)
        self.laser_thread.start()

    def thread_arm_laser(self):
        if self._widget.pushButton_armLaser.text() == "Arm Laser":
            rospy.wait_for_service('/mira_ds/arm_laser')
            armLaser = rospy.ServiceProxy('/mira_ds/arm_laser', Trigger)
            response = armLaser()
            path_to_icon = os.path.join(rospkg.RosPack().get_path('rqt_mira_ds'), 'resource', 'warning_icon.png')
            pixmap = QPixmap(path_to_icon)
            self._widget.label_armLaser.setPixmap(pixmap)
            self._widget.pushButton_armLaser.setText("Disarm Laser")
            self._widget.label_status.setText("Laser Armed!")
            self.finishedThread = True
        else:
            rospy.wait_for_service('/mira_ds/disarm_laser')
            disarmLaser = rospy.ServiceProxy('/mira_ds/disarm_laser', Trigger)
            response = disarmLaser()
            self._widget.pushButton_armLaser.setText("Arm Laser")
            path_to_icon = os.path.join(rospkg.RosPack().get_path('rqt_mira_ds'), 'resource', 'error.png')
            pixmap = QPixmap(path_to_icon)
            self._widget.label_armLaser.setPixmap(pixmap)
            self._widget.label_status.setText("Laser Disarmed")
            self.finishedThread = True

    @Slot() # Button callback function
    def getRange_button_slot(self):
        self._widget.label_status.setText("getting range...")
        self.range_thread = threading.Thread(target=self.thread_get_range)
        self.range_thread.start()

    def thread_get_range(self):
        rospy.wait_for_service('/mira_ds/get_range')
        getRange = rospy.ServiceProxy('/mira_ds/get_range', Trigger)
        response = getRange()
        self._widget.label_getRange.setText(response.message)
        self._widget.label_status.setText("Got Range")
        self.finishedThread = True

    @Slot() # Button callback function
    def autofocus_button_slot(self):
        self._widget.label_status.setText("setting autofocus...")
        self.autofocus_thread = threading.Thread(target=self.thread_set_autofocus)
        self.autofocus_thread.start()

    def thread_set_autofocus(self):
        rospy.wait_for_service('/mira_ds/autofocus')
        autofocus = rospy.ServiceProxy('/mira_ds/autofocus', Trigger)
        response = autofocus()
        path_to_icon = os.path.join(rospkg.RosPack().get_path('rqt_mira_ds'), 'resource', 'ok.png')
        pixmap = QPixmap(path_to_icon)
        self._widget.label_autofocus.setPixmap(pixmap)
        self._widget.label_status.setText("Set Autofocus")
        self.finishedThread = True

    @Slot() # Button callback function
    def start_button_slot(self):
        self._widget.label_status.setText("starting...")
        self.clean_plotting_widget()
        if self._widget.comboBox_start.currentText() == "ASFO - Glimpse":
            self.acquire_ASFO_ss()
        elif self._widget.comboBox_start.currentText() == "Acquire":
            self.acquire()
        elif self._widget.comboBox_start.currentText() == "ASFO":
            self.acquire_ASFO()
        else:
            self._widget.label_status.setText("Did not match start entry!")

    def acquire_ASFO(self):
        self._widget.pushButton_armLaser.setText("Arm Laser")
        self._widget.label_status.setText("Acquiring ASFO")
        self.success = False
        self.finishedThread = True
        self.state = 0
        self.timer.start(100)

    def acquire(self):
        self._widget.label_status.setText("acquiring...")
        self.finishedThread = False
        self.state = 5
        self.timer.start(100)
        self.acquire_thread = threading.Thread(target=self.thread_acquire)
        self.acquire_thread.start()

    def acquire_ASFO_ss(self):
        self._widget.label_status.setText("Acquiring ASFO ss")
        self.state = 5
        self.timer.start(100)
        self.autoacquire_thread = threading.Thread(target=self.thread_acquire_ASFO_ss)
        self.autoacquire_thread.start()
        
    def thread_acquire_ASFO_ss(self):
        self._widget.pushButton_start.setEnabled(False)
        message = Bool()
        if self._widget.radioButton_xtr.isChecked():
            message.data = True
        else:
            message.data = False
        self.autoAcquire_publisher.publish(message)

    def thread_acquire(self):
        self._widget.pushButton_start.setEnabled(False)
        message = Bool()
        if self._widget.radioButton_xtr.isChecked():
            message.data = True
        else:
            message.data = False
        self.acquire_publisher.publish(message)

    def sleep(self):
        self._widget.label_status.setText("sleeping...")
        hello_str = "raman"
        self.pub.publish(hello_str)
        self.sleep_thread = threading.Thread(target=self.thread_sleep)
        self.sleep_thread.start()

    def thread_sleep(self):
        time.sleep(10)
        self.finishedThread = True

    @Slot() # Button callback function
    def update_slot(self):
        if self.state == 0:
            if self.finishedThread == True:
                self.finishedThread = False
                self.state = 1
                self.armLaser_button_slot()
        elif self.state == 1:
            if self.finishedThread == True:
                self.finishedThread = False
                self.state = 2
                self.getRange_button_slot()
        elif self.state == 2:
            if self.finishedThread == True:
                if self._widget.label_getRange.text() == '0':
                    self._widget.label_status.setText("Range is 0!")
                    self.timer.stop()
                    return
                self.finishedThread = False
                self.state = 3
                self.autofocus_button_slot()
        elif self.state == 3:
            if self.finishedThread == True:
                self.finishedThread = False
                self.state = 4
                self.sleep()
        elif self.state == 4:
            if self.finishedThread == True:
                self.finishedThread = False
                self.state = 5
                self.acquire()
        elif self.state == 5:
            if self.finishedThread == True:
                self.finishedThread = False
                self.state = 0
                self.printResults()
                self.dumpCsvFile()
                self.timer.stop()

    def printResults(self):
        rospy.loginfo("<<<[Mira_DS_Plugin]>>>  printResults]")

        if self.received_results:
            self._widget.label_result_name.setText(self.match)
            self._widget.label_result_hazard.setText(self.hazard_level)
            self._widget.label_result_operating_procedure.setText(self.operating_prozedure)

            # # Create plot widget
            self.wavelength = numpy.linspace(self.first_wavenumber, self.first_wavenumber + self.intensities_count - 1, self.intensities_count)

            self.graphWidget = pg.PlotWidget()
            self._widget.gridLayout_spectrum.addWidget(self.graphWidget)

            self.graphWidget.setBackground('w')

            self.graphWidget.addLegend()
            pen_red = pg.mkPen(color=(255, 0, 0))
            pen_green = pg.mkPen(color=(0, 255, 0))
            pen_blue = pg.mkPen(color=(0, 0, 255))
            self.graphWidget.plot(self.wavelength, self.intensities, pen=pen_red, name="intensities")
            self.graphWidget.plot(self.wavelength, self.baselines, pen=pen_green, name="baseline")
            self.graphWidget.plot(self.wavelength, self.top_match_intensities, pen=pen_blue, name="matched intensities")


    def addToPlot(self, plot, x, y, offset, title, color):
        for i in range(len(y)):
            y[i] = y[i] + offset
        #qwt.QwtPlotCurve.make(x, y, title, plot, linecolor=color, antialiased=True)

    def result_callback(self, message):
        rospy.loginfo("<<<[Mira_DS_Plugin]>>>  result_callback]!")
        self._widget.pushButton_start.setEnabled(True)
        #self._widget.label_status.setText("Acquired")
        self.received_results = True
        #self.finishedThread = True
        self.match = message.match
        self.hazard_level = message.hazard_level
        self.operating_prozedure = message.operating_prozedure
        self.currentRange = message.range
        self.correlations = message.correlations
        self.first_wavenumber = message.first_wavenumber
        self.intensities_count = message.intensities_count
        self.intensities = numpy.array(message.intensities)
        self.baselines = numpy.array(message.baselines)
        self.top_match_intensities = numpy.array(message.top_match_intensities)

    def result_acquire_callback(self, message):
        rospy.loginfo("[MiraDS : result_acquire_callback] result_acquire_callback!")
        self._widget.pushButton_start.setEnabled(True)
        if message.data:
            self._widget.label_status.setText("Acquire successful")
        else:
            self._widget.label_status.setText("Acquire unsuccessful")
        self.finishedThread = True

    def clean_plotting_widget(self):
        self._widget.label_result_name.setText("")
        self._widget.label_result_hazard.setText("")
        self._widget.label_result_operating_procedure.setText("")
        for i in reversed(range(self._widget.gridLayout_spectrum.count())): 
            widgetToRemove = self._widget.gridLayout_spectrum.itemAt(i).widget()
            # remove it from the layout list
            self._widget.gridLayout_spectrum.removeWidget(widgetToRemove)
            # remove it from the gui
            widgetToRemove.setParent(None)

    def dumpCsvFile(self):
        if ( self._widget.checkBox_dumpCSV.isChecked()):
            rospack = rospkg.RosPack()
            results_folder_path = rospack.get_path('rqt_mira_ds') + "/results/"
            file_path = results_folder_path + "scan_" + str(self.scanNumber) + "_" + self._widget.lineEdit_filename.text()
            rospy.loginfo("[MiraDS : dumpCsvFile] file: " + file_path)
            csv_array = numpy.vstack((self.wavelength, self.intensities.T)).T
            numpy.savetxt(file_path + ".csv", csv_array, fmt='%1.1f', delimiter=",")
            scan_infos = [ 'Matched Mixture: ', self.match, '\nHazard level: ', self.hazard_level, '\nOperating Procedure: ', self.operating_prozedure, '\nLaserRange: ', str(self.currentRange)]
            with open(file_path + ".txt", 'w') as file:
                file.writelines(scan_infos)
                file.write('\n')
                file.write('Ĉorrelations:    ')
                for element in self.correlations:
                    file.write(element)
                    file.write("   ;   ")
            rospy.loginfo("<<<[Mira_DS_Plugin]>>> finished dumping files :)")
            self.scanNumber = self.scanNumber + 1
