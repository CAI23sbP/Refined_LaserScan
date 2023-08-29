
import numpy as np
import rospy, os, math
from sensor_msgs.msg import LaserScan

class CONFIG():
    th_distance = 5
    scanSkip = 1
    maxAngle = 270

class LASER_SCAN:
    def __init__(self,ns):
        rospy.init_node('subscriber_node', anonymous=True)

        self.ns = ns
        self.ns_prefix = lambda topic: os.path.join(self.ns, topic)
        self.scan_pub = rospy.Publisher(self.ns_prefix("refined_scan"),LaserScan,queue_size=1)
        self.scan_sub = rospy.Subscriber(
                    self.ns_prefix("scan"),LaserScan, self.callback_scan
            )
        self.th_distance = CONFIG.th_distance
        self.scanSkip = CONFIG.scanSkip
        self.maxAngle = CONFIG.maxAngle
        self.scan_dim = 360 #default 
        
    def callback_scan(self,msg)->None:
        refined_scans = LaserScan()
        refined_scans.header.stamp = rospy.Time.now()
        refined_scans.header.frame_id = "/base_scan"
        refined_scans.angle_increment = msg.angle_increment
        refined_scans.range_max = 5
        refined_scans.ranges, positions = LASER_SCAN.refined_scan(msg.ranges, self.th_distance, self.scanSkip, self.maxAngle, self.scan_dim, msg.range_max, msg.angle_increment)
        self.scan_pub.publish(refined_scans)
    
    @staticmethod
    def refined_scan(ranges:np.ndarray, th_distance:float, scanSkip:int ,maxAngle: int, scan_dim:int, laser_range: float, angle_increment: float):
        obst = np.empty([0,2])   # reset the obstacle set to only keep visible objects
        deg = len(ranges)   # Number of degrees - varies in Sim vs real world
        if deg ==0:
            deg = scan_dim

        discretized_ranges = np.array([])
        for index in range(0,deg-1,scanSkip):
            distance = ranges[index] 
            angle = index * angle_increment
            if (distance < self.th_distance):
                if (index%scanSkip ==0):
                    if index <= int((maxAngle/2))-int(((maxAngle/2))%scanSkip): ## right
                        obsX = round(distance*math.cos(angle),4)
                        obsY = round(distance*math.sin(angle),4)
                        obs_row = np.array([obsX, obsY])
                        self.obst = np.vstack((self.obst,obs_row))

                    elif index>= int(int((maxAngle/2)/scanSkip) + int(int(2*(maxAngle/2 % scanSkip)) + deg- maxAngle)/scanSkip)*scanSkip : ## left
                        obsX = round(distance*math.cos(angle),4)
                        obsY = round(distance*math.sin(angle),4)
                        obs_row = np.array([obsX, obsY])
                        self.obst = np.vstack((self.obst,obs_row))
                    else:
                        obsX = np.nan
                        obsY = np.nan
                        obs_row = np.array([obsX, obsY])
                        self.obst = np.vstack((self.obst, obs_row))
                else:
                    obsX = np.nan
                    obsY = np.nan
                    obs_row = np.array([obsX, obsY])
                    self.obst = np.vstack((self.obst, obs_row))
            else:
                obsX = np.nan
                obsY = np.nan
                obs_row = np.array([obsX, obsY])
                self.obst = np.vstack((self.obst, obs_row))
        discretized_ranges = np.round(np.hypot(obst[:,0],obst[:,1]),4)
        return discretized_ranges, obst
    

        
if "__main__" ==__name__:
    while not rospy.is_shutdown():
        scan = LASER_SCAN(ns ="/sim_1")
