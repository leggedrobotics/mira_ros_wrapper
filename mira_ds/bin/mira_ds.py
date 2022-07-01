#! /usr/bin/env python3

from time import sleep
import json
import time
import rospy
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from MR_Python_API import MiraDS
from std_msgs.msg import String, Bool
from mira_msgs.msg import Results

class MiraDs(object):

    def __init__(self):
        self.device = MiraDS.MiraDS_client(MiraDS.Comm_Type.SERIAL)
        rospy.Service("/mira_ds/arm_laser", Trigger, self.armLaserCallback)
        rospy.Service("/mira_ds/disarm_laser", Trigger, self.disarmLaserCallback)
        rospy.Service("/mira_ds/get_range", Trigger, self.getRangeCallback)
        rospy.Service("/mira_ds/autofocus", Trigger, self.autofocusCallback)

        rospy.Subscriber("/mira_ds/connect", Bool, self.connectMiraCallback)
        rospy.Subscriber("/mira_ds/acquire", Bool, self.acquireCallback)
        rospy.Subscriber("/mira_ds/auto_acquire", Bool, self.autoAcquireCallback)

        self.result_publisher = rospy.Publisher('/mira_ds/result', Results, queue_size=1)
        self.result_connect_publisher = rospy.Publisher('/mira_ds/result_connect', Bool, queue_size=1)
        self.result_acquire_publisher = rospy.Publisher('/mira_ds/result_acquire', Bool, queue_size=1)
        self.currentRange = 0

    def connectMiraCallback(self, message):
        result = Bool()
        if message.data:
            rospy.loginfo("<<<[Mira_DS]>>> gonna connect!")
            if self.connectMira():
                result.data = True
            else:
                result.data = False
        else:
            rospy.loginfo("<<<[Mira_DS]>>> gonna disconnect!")
            if self.disconnectMira():
                result.data = True
            else:
                result.data = False            
        self.result_connect_publisher.publish(result)

    def armLaserCallback(self, request):
        if self.armLaser():
            return TriggerResponse(
            success=True,
            message="Hey, roger that; we'll be right there!"
            )
        else:
            return TriggerResponse(
            success=False,
            message="Hey, There was a Problem!"
            )

    def disarmLaserCallback(self, request):
        if self.disarmLaser():
            return TriggerResponse(
            success=True,
            message="Hey, roger that; we'll be right there!"
            )
        else:
            return TriggerResponse(
            success=False,
            message="Hey, There was a Problem!"
            )

    def getRangeCallback(self, request):
        range = self.getRange()
        self.currentRange = range
        if range:
            return TriggerResponse(
            success=True,
            message=range
            )
        else:
            return TriggerResponse(
            success=False,
            message="Hey, There was a Problem!"
            )

    def autofocusCallback(self, request):
        if self.autofocus():
            return TriggerResponse(
                success=True,
                message="Hey, roger that; we'll be right there!"
            )
        else:
            return TriggerResponse(
                success=False,
                message="Hey, There was a Problem!"
            )

    def acquireCallback(self, message):
        rospy.loginfo("<<<[Mira_DS]>>> acquireCallback!")
        result = Bool()
        if self.acquire(message.data):
            result.data = True
        else:
            result.data = False
        self.result_acquire_publisher.publish(result)
    
    def autoAcquireCallback(self, message):
        rospy.loginfo("<<<[Mira_DS]>>> autoAcquireCallback!")
        result = Bool()
        result.data = True

        if not(self.armLaser()):
            result.data = False
            self.result_acquire_publisher.publish(result)
            return
        
        self.currentRange = self.getRange()
        if not(self.currentRange):
            result.data = False
            self.result_acquire_publisher.publish(result)
            return
        
        if not(self.autofocus()):
            result.data = False
            self.result_acquire_publisher.publish(result)
            return

        if not(self.acquire(message.data)):
            result.data = False
            self.result_acquire_publisher.publish(result)
            return

        result.data = True
        self.result_acquire_publisher.publish(result)
        return

    def connectMira(self):
        success = self.device.Connect()
        if success:
            rospy.loginfo("<<<[Mira_DS]>>> Successfully connected!")
        else:
            rospy.loginfo("<<<[Mira_DS]>>> There was a problem connecting!")
        return success

    def disconnectMira(self):
        success = self.device.Disconnect()
        if success:
            rospy.loginfo("<<<[Mira_DS]>>> Successfully disconnected!")
        else:
            rospy.loginfo("<<<[Mira_DS]>>> There was a problem disconnecting!")
        return success

    def armLaser(self):
        success = self.device.AFSOLaserOn()
        if success:
            rospy.loginfo("<<<[Mira_DS]>>> Successfully armed the laser!")
        else:
            rospy.loginfo("<<<[Mira_DS]>>> There was a problem when arming the laser!")
        return success
        
    def disarmLaser(self):
        success = self.device.AFSOLaserOff()
        if success:
            rospy.loginfo("<<<[Mira_DS]>>> Successfully disarmed the laser!")
        else:
            rospy.loginfo("<<<[Mira_DS]>>> There was a problem when disarming the laser!")
        return success

    def getRange(self):
        success = self.device.AFSOGetRange()
        if success:
            rospy.loginfo("<<<[Mira_DS]>>> Successfully got range from the laser!")
        else:
            rospy.loginfo("<<<[Mira_DS]>>> There was a problem when getting the range from the laser!")
        return success

    def autofocus(self):
        success = self.device.AFSOAutoFocus()
        if success:
            rospy.loginfo("<<<[Mira_DS]>>> Successfully set autofocus!")
        else:
            rospy.loginfo("<<<[Mira_DS]>>> There was a problem when setting the autofocus!")
        return success

    def acquire(self, xtr_process=True):
        if xtr_process:
            result = self.device.Acquire(download_func=self.device.DownloadLastSample, guid="1c06cb54-a111-46cb-8272-e57b99d1217c")
            rospy.loginfo("<<<[Mira_DS]>>> acquiring with xtr!")
        else:
            result = self.device.Acquire(download_func=self.device.DownloadLastSample, guid="921d4a81-001b-4c22-81d2-159069d2825a")
            rospy.loginfo("<<<[Mira_DS]>>> acquiring without xtr!")


        if result != False:
            #print(result.Metadata)
            message = Results()
            metaData_json = json.loads(result.Metadata)

            if "Mixtures" in metaData_json:
                metaData_Mixtures = metaData_json["Mixtures"][0]
                message.match = metaData_Mixtures["Name"] + "; Score: " + str(metaData_Mixtures["MatchScore"])
                message.hazard_level = str(metaData_Mixtures["HazardLevel"])
            else:
                message.match = 'ERROR'
                message.hazard_level = ''

            if "Correlations" in metaData_json:
                list_of_correlations = [""]
                for element in metaData_json["Correlations"]:
                    message.correlations.append(element["Name"] + "; Score: " + str(element["MatchScore"]))

            message.operating_prozedure = str(metaData_json["OperatingProcedure"]["Name"])
            message.first_wavenumber = metaData_json["FirstWavenumber"]
            message.intensities_count = metaData_json["IntensitiesCount"]
            message.range = int(self.currentRange)
            message.intensities.clear()
            message.baselines.clear()
            message.top_match_intensities.clear()
            for element in self.device.CurrentSampleData.Intensities:
                message.intensities.append(int(element))
            for element in self.device.CurrentSampleData.Baselines:
                message.baselines.append(int(element))
            for element in self.device.CurrentSampleData.TopMatchIntensity:
                message.top_match_intensities.append(int(element))

            self.result_publisher.publish(message);       
            rospy.loginfo("<<<[Mira_DS]>>> Mira Result published!")

            sleep(2.0) # TODO check why this is needed

            return True

        else:
            rospy.loginfo("<<<[Mira_DS]>>> PROBLEM acquiring. result was false!")
            return False

if __name__ == '__main__':
    rospy.init_node('mira_ds')
    rospy.loginfo("Mira Ds node started")
    miraDs = MiraDs()
    rospy.spin()
