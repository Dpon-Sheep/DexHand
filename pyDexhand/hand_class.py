import pyDexhand
import pyDexhand.kEstimator as kEstimator
import serial
import struct
task_list = ['NORMAL','STOP','START','CONTACT','SETZERO','GOTO','SETFORCE']
class DexHandBasic(serial.Serial):
    def __init__(self,COMName,if_debug=False,timeout=0.05):
        super(DexHandBasic,self).__init__(COMName,baudrate=115200,timeout=timeout)
        if self.isOpen():
            print("Opened Serial Port Successfully")
        else:
            print("Failed to Open Serial Port")
        self.if_debug = if_debug
        self.now_force = None
        self.goal_force = None
        self.pos = None
        self.task = None
        self.kestimator = kEstimator.DexHand_kEstimator()

    def _uniform_send(self,message):
        message = " ".join(message)
        send_bytes = bytes.fromhex(message)
        write_len = self.write(send_bytes)
        if self.if_debug:
            print('port has sent %d bytes'%write_len)

    def stop_gripper(self): # stop moving gripper and turn off the motor.
        message = ['00']
        self._uniform_send(message)

    def start_gripper(self): # restart the motor from turned off.
        message = ['00','01']
        self._uniform_send(message)

    def close_gripper(self,speed = 500): # move the gripper along closing direction.
        if speed > 2000:
            print('Error: goal speed cannot larger than 2000')
            return
        message = ['01','01','F4']
        message[1] = ('%02x'%(speed>>8))
        message[2] = ('%02x'%(speed&0xff))
        self._uniform_send(message)

    def open_gripper(self,speed = 500):  # move the gripper along open direction.
        if speed > 2000:
            print('Error: goal speed cannot larger than 2000')
            return
        message = ['02','01','F4']
        message[1] = ('%02x'%(speed>>8))
        message[2] = ('%02x'%(speed&0xff))
        self._uniform_send(message)

    def gripper_pending(self): # stop moving the gripper and keep it staying at present position
        message = ['01','00','00']
        self._uniform_send(message)

    def set_home(self): # set gripper's home
        message = ['03']
        self._uniform_send(message)

    def goto(self,pos): # move gripper to its home
        message = ['04','00','00','00']
        message[1] = ('%02x'%(pos>>8))
        message[2] = ('%02x'%(pos&0xff))
        self._uniform_send(message)

    def set_weight_param(self,weighttime): # set now force as zero
        message = ['05','00','00']
        weighttimedata = round(weighttime*10000)
        message[1] = ('%02x'%(weighttimedata>>8))
        message[2] = ('%02x'%(weighttimedata&0xff))
        self._uniform_send(message)

    def approching(self): # gripper will approch object and stop automately when contact with the object
        message = ['08']
        self._uniform_send(message)

    def forcecontrol(self,force): # force control
        force = round(force*100)
        message = ['06','00','32','01','00','00']
        kdata = round(self.kestimator.k*10000)
        message[1]=('%02x'%(force>>8))
        message[2]=('%02x'%(force & 0xff))
        message[4]=('%02x'%(kdata>>8))
        message[5]=('%02x'%(kdata & 0xff))
        self._uniform_send(message)
    def preload(self,force):
        force = round(force*100)
        message = ['06','03','E8','00']
        message[1]=('%02x'%(force>>8))
        message[2]=('%02x'%(force & 0xff))
        self._uniform_send(message)
    def set_pid(self,Kp,Ki,maxi):
        message = ['07','00','00','00','00','00']
        KP = round(Kp*100)
        KI = round(Ki*10000)
        message[1] = ('%02x'%(KP>>8))
        message[2] = ('%02x'%(KP&0xff))
        message[3] = ('%02x'%(KI>>8))
        message[4] = ('%02x'%(KI&0xff))
        message[5] = ('%02x'%(maxi))
        self._uniform_send(message)
    def update_hand_data(self):
        try:
            get_msg = self.read(9)
            self.task = task_list[get_msg[0]]
            self.now_force,self.goal_force,self.pos = struct.unpack('>hhi',get_msg[1:])
            self.now_force/=100
            self.goal_force/=100
            self.pos/=10000
            if self.if_debug:
                self.report_data()
            return True
        except:
            return False
    def data_push_in(self):
        self.kestimator.data_push_in(self.pos,self.now_force)

    def report_data(self):
        # print(f'{self.task}: nowforce={self.now_force} N, goalforce = {self.goal_force} N, pos = {self.pos}mm k={self.kestimator.k}')
        print('%s: nowforce = %.2f N, goalforce = %.2f N, pos = %.4f mm, k= %.4f'%(self.task,self.now_force,self.goal_force,self.pos,self.kestimator.k))
