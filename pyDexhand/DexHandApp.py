import pyDexhand.hand_class as hand_class
import time
import threading

class DexHand():
    def __init__(self,COMName,if_debug=False,timeout=0.05,report_period = 5,is_report = True):
        '''
        DexHand Initialization. The hand will open to maximum and set this position as zero position.
        Parameter(s):
            -COMName: serial port's name. On windows, it may looks like "COM9" (or "\\\\.\\COM16" when port id greater than 9). On Linux, it may looks like '/dev/ttyACM0'
            -if_debug: print some info for debugging when it is true
            -Timeout: serial port reading timeout
            -report_period: print some info for grasping every (report_period * 10ms)
            -is_report: print some info for grasping if it is true
        '''
        self._basicDexHand = hand_class.DexHandBasic(COMName,if_debug,timeout)
        self._contactFlag = False
        self._forceControlFlag = False
        self.report_period = round(report_period)
        self.is_report = is_report
        self._update_flag = False

        self._listener = threading.Thread(target=self.update_hand_data)
        self._listener.setDaemon(True)
        self._listener_cnt = 0
        self._listener.start()

        time.sleep(0.1)
        self._basicDexHand.set_home()
        while self._basicDexHand.task != 'SETZERO':
            pass
        while self._basicDexHand.task =='SETZERO':
            pass
        self._basicDexHand.set_weight_param(1.883) #1.883,0.9099

        print('DexHand has initialized successfully')


    def contact(self):
        '''
        Close Dexhand's fingers until they contact an object
        The function will not return until Dexhand has indeed contact an object 

        Parameter(s):
            None
        '''
        self._basicDexHand.approching()
        self.is_report = True
        self._basicDexHand.kestimator.__init__()
        self._update_flag = False
        cnt = 0
        while self._basicDexHand.task != 'CONTACT':
            if self._update_flag ==True:
                self._update_flag = False
                cnt +=1
                if cnt >=10:
                    self._basicDexHand.approching()
        while self._basicDexHand.task == 'CONTACT':
            pass     
        self._contactFlag = True
        self.is_report = False
        print('DexHand has contacted an object!')
        

    def grasp(self,force=5):
        '''
        Preload a force onto the object. the unit of force is Newton and the default force is 0.5.
        Before preload, DexHand must has contact an object. If not, this function will first make it contact.
        The function will not return until the desired force is reached.
        During preloading, the stiffness will be estimated but not used in force control.

        Parameter(s):
            -force: desired preload force
        '''
        if force > 20:
            print('Warning: The goal force cannot larger than 20N!')
            return
        if self._contactFlag == False:
            print('Warning: DexHand may not have contact an object. Now it is approching!')
            self.contact()
        self._contactFlag = True
        self._forceControlFlag = True
        self.is_report = True

        self._basicDexHand.preload(force)
        cnt=0
        self._update_flag = False
        while self._basicDexHand.task != 'SETFORCE':
            if self._update_flag ==True:
                self._update_flag = False
                cnt +=1
                if cnt >=10:
                    self._basicDexHand.preload(force)
        while self._basicDexHand.task == 'SETFORCE':
            pass   
        print('DexHand has contacted an object!')
        self._forceControlFlag = False
        self.is_report = False

    def forcecontrol(self,force):
        '''
        Control grasping force of DexHand. 
        before force controlling, an preloading force is recommended(and contact is necessary!).
        The function will return once the force controlling command was sent through serial port

        During preloading, the stiffness will be estimated and used in force control.

        Parameter(s):
            -force: desired grasping force
        '''

        if force > 20:
            print('Warning: The goal force cannot larger than 20N!')
            return
        self._forceControlFlag = True
        self.is_report = True
        self._basicDexHand.forcecontrol(force)


    def goto(self,pos):
        '''
        Move DexHand's gripper to appointed position. the unit is millimeter(not quite precise though)

        Parameter(s):

        '''
        if pos > 57.0:
            print('Warning: Goal position cannot be greater than 57.0!')
            return
        self.is_report = False
        self._basicDexHand.goto(pos*100)
        cnt = 0
        self._update_flag = False
        while self._basicDexHand.task != 'GOTO':
            if self._update_flag ==True:
                self._update_flag = False
                cnt +=1
                if cnt >=10:
                    self._basicDexHand.goto(pos*100)
        forcecnt = 0
        self._update_flag = False
        while self._basicDexHand.task == 'GOTO':
            if self._update_flag == True:
                self._update_flag = False
                if self.now_force <=0.01:
                    forcecnt+=1
                    if forcecnt >=5:
                        self._contactFlag = False
                        self._forceControlFlag = False
                else: 
                    forcecnt = 0
        print('DexHand has reached goal position')
        self.is_report = False


    def detacted(self):
        '''
        Open Dexhand's gripper and move the gripper to position zero
        '''
        self.goto(0)
        self._contactFlag = False   
        self._forceControlFlag = False
    

    def update_hand_data(self):
        while True:
            if self._basicDexHand.update_hand_data() == False:
                continue

            self._update_flag = True
            if self._forceControlFlag:
                self._basicDexHand.data_push_in()
            if self.is_report:
                self._listener_cnt+=1
                if self._listener_cnt == self.report_period:
                    self._listener_cnt = 0
                    self.report_data()
            
        

    def report_data(self):
        '''
        report DexHand's task, current pos, current force and goal force

        Parameter(s):
            None
        '''
        self._basicDexHand.report_data()


    @property
    def now_force(self):
        return self._basicDexHand.now_force
    
    @property
    def goal_force(self):
        return self._basicDexHand.goal_force
    
    @property
    def now_pos(self):
        return self._basicDexHand.pos
    
    @property
    def now_k(self):
        return self._basicDexHand.kestimator.k