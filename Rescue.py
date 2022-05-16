#from video_test import *
from optitrack_test import *
from math_function import *
import networkx as nx
from reference import *

nodeToCo = [[5.4, -0.2], [3.2, -3.0], [1.3, -1.7], [-2.4, -2.8], [-4, -2], [-2.1, 0.1], [-0.5, 0.3],
                    [-1.9, 1.7], [-4.0, 2.3], [-4.0, 3.5], [-2.6, 3.3], [1.6, 2.6], [2.8, 1.4], [4.4, 2.7]]
        # edgeList = [[0,1],[0, 2], [2, 3],[1,2],[3,4],[4,5],[5,6],[7,8],[6,8],[7,10],[4,9],[5,9],[9,15],[19,15],[9,19],[15,16],[19,16],[16,17],[17,18],[11,12],[12,13],[13,14],[14,15]]
edgeList = [[0, 1], [1, 2], [2, 12], [2, 3], [3, 4], [3, 5], [5, 6], [5, 7], [7, 8], [7, 10], [8, 9], [9, 10],
                    [7, 11], [11, 12], [0, 12], [0, 13]]


class Rescue:
    def __init__(self):
        self.state = 0
        # state 1: wandering; state 2: grip; state 3: back; state 4: emergency

        # connection information
        self.robot_id = 12
        self.robot_IP = '192.168.0.212'
        self.client_IP = '192.168.0.23'
        self.server_IP = "192.168.0.4"
        self.socket_control = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_video = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.streaming_client = NatNetClient()

        # waypoints information
        # self.waypoints = [[0, 0], [0, 3], [0, -3], [4, 0], [-4, 0], [4, 3], [4, -3], [-4, 3], [-4, -3]]
        self.G = nx.Graph()
        self.wayPoint = []
        self.waypointid = 0

        # P control information
        self.kp = 500
        self.kr = 30
        self.ko = 1

        # map information
        # self.grid = GridMap((8, 12), 4)
        # self.distance = 1000

        # duck information
        self.target_FLAG = False
        self.target_center = (0, 0)
        self.target_radius = 0

        #emergency information
        self.emergency_level = 0
        self.pre_x = 0
        self.pre_y = 0

    def Loop(self):
        self.ConnectToRobot()
        self.ConnectToServer()
        self.BuildGraph()
        self.state = 1
        while True:
            print(self.state)
            if self.emergency_level > 100:
                self.state = 4
            if self.state == 1:
                self.Wandering()
            elif self.state == 2:
                self.GoRescure()
            elif self.state == 3:
                self.GoBack()
            else:
                self.Emergency()
        # self.StopConnectToRobot()


    def ConnectToRobot(self):
        self.socket_control.connect((self.robot_IP, 5000))
        #self.socket_video.connect(self.robot_IP, 8000)
        print("Connection Successful!")

    def StopConnectToRobot(self):
        self.socket_control.shutdown(2)
        self.socket_video.shutdown(2)
        self.socket_control.close()
        self.socket_video.close()

    def ConnectToServer(self):
        self.streaming_client.set_client_address(self.client_IP)
        self.streaming_client.set_server_address(self.server_IP)
        self.streaming_client.set_use_multicast(True)
        self.streaming_client.rigid_body_listener = receive_rigid_body_frame

    '''
    def UpdateData(self):
        self.Ultrasound()
        self.Video()

    def Ultrasound(self):
        command = 'CMD_SONIC#1\n'
        self.socket_control.send(command.encode('utf-8'))
        data = s.recv(1024).decode('utf-8')
        self.distance = int(data.split("#")[1])

        command = 'CMD_SONIC#0\n'
        s.send(command.encode('utf-8'))

    def Video(self):
        video_connection = self.socket_video.makefile('rb')

        while True:
            stream_byte = video_connection.read(4)
            jpg_buffer = struct.unpack('<L', stream_byte[:4])
            video_jpg = connection.read(jpg_buffer[0])

            if IsValidImage4Bytes(jpg):
                img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                self.DetectDuck(img)

    def DetectDuck(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_red = np.array([26, 100, 100])
        upper_red = np.array([34, 255, 255])
        mask = cv2.inRange(gray, lower_red, upper_red)
        result = cv2.bitwise_and(image, image, mask=mask)
        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)

        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                   param1=100, param2=15,
                                   minRadius=1, maxRadius=200)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            biggest = circles[0, 0]
            for i in circles[0, :]:
                if i[2] > biggest[2]:
                    biggest = i

            center = (biggest[0], biggest[1])
            # circle center
            cv2.circle(gray, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = biggest[2]
            cv2.circle(gray, center, radius, (255, 0, 255), 3)


        # time.sleep(1)
    '''

    def Wandering(self):
        waypoint_id = random.randint(10, 13)
        print(waypoint_id)
        self.GoToPoint(0, waypoint_id)
        self.waypointid = waypoint_id
        print("aaaaaaaaaaaaaaaaaaaaaaaaaa",'2')
        self.state = 2

    def goToWayPoint(self, x1, y1, x2, y2, r):
        s = self.socket_control
        k1 = 150
        k2 = 25
        K = 5
        robot_id = self.robot_id

        if ismoving(self.pre_x, self.pre_y, positions[self.robot_id][0], positions[self.robot_id][1]) == False:
            self.emergency_level = self.emergency_level + 1
        else:
            self.emergency_level = 0
        self.pre_x = positions[self.robot_id][0]
        self.pre_y = positions[self.robot_id][1]

        if getDistance(x1, y1, x2, y2) > 0.15:
            v = -600 - getDistance(x1, y1, x2, y2) * k1
            angle = getAngle(x1, y1, x2, y2)
            if abs(angle - r) < 25:
                omega = 0
            else:
                omega = (angle - r) * k2
            u = np.array([v + omega, v - omega])
            u[u > 1500] = 1500
            u[u < -1500] = -1500
            # Send control input to the motors
            ''' if getDistance(x1, y1, x2, y2) > 0.1 and abs(omega) < 30 * k2 and u[0] > -600:
                u[0] = -600
            if getDistance(x1, y1, x2, y2) > 0.1 and abs(omega) < 30 * k2 and u[1] > -600:
                u[1] = -600
            '''
            command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (1.4 * u[0], 1.4 * u[0], u[1], u[1])
            print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
            print("gotowaypoint", x2, y2, command)
            s.send(command.encode('utf-8'))
        else:
            command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (0, 0, 0, 0)
            s.send(command.encode('utf-8'))
        time.sleep(0.2)

    def GoToPoint(self, cur_id, waypoint_id):
        self.SetWaypoints(cur_id, waypoint_id)
        is_running = self.streaming_client.run()
        robot_id = self.robot_id
        while is_running:
            if self.robot_id not in positions:
                continue

            for i in range(len(self.wayPoint)):
                if i == 0:
                    continue
                while getDistance(positions[robot_id][0], positions[robot_id][1], self.wayPoint[i][0], self.wayPoint[i][1]) > 0.2:
                    print('*******************************************************************************',
                          '          ', i)

                    self.goToWayPoint(positions[robot_id][0], positions[robot_id][1], self.wayPoint[i][0], self.wayPoint[i][1], rotations[robot_id])
                print('*******************************************************************************','          ',i)

            break

        return


    def GoBack(self):
        waypoint_id = random.randint(1, 13)
        self.GoToPoint(self.waypointid, 0)
        self.waypointid = 0
        self.state = 1

    def GoRescure(self):
        self.Servo()
        self.state = 3

    def Emergency(self):
        command = 'CMD_MOTOR#-1500#-1500#-1500#-1500\n'
        self.socket_control.send(command.encode('utf-8'))
        time.sleep(3)
        self.emergency_level = 0

        self.state = 1

    def RotateRandomly(self):
        rotate_time = random.randint(1, 20) / 10
        command = 'CMD_MOTOR#-1500#-1500#1500#1500\n'
        self.socket_control.send(command.encode('utf-8'))

    def Servo(self):
        s = self.socket_control
        command = "CMD_SERVO#0#90\n"
        s.send(command.encode('utf-8'))
        print('down')
        command = "CMD_SERVO#1#50\n"
        s.send(command.encode('utf-8'))
        print('open')
        # s.send(forward.encode('utf-8'))
        # time.sleep(1)
        # command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (0, 0, 0, 0)
        # s.send(command.encode('utf-8'))
        # while True:
        command = "CMD_SERVO#0#140\n"
        s.send(command.encode('utf-8'))
        command = "CMD_SERVO#1#-30\n"
        s.send(command.encode('utf-8'))
        time.sleep(1)
        turnRight = 'CMD_MOTOR#-1300#-1300#600#600\n'
        s.send(turnRight.encode('utf-8'))
        time.sleep(5)

        i = 1
        d1 = 50
        d2 = 105
        command = 'CMD_SERVO#%d#%d\n' % (i, d1)
        s.send(command.encode('utf-8'))
        time.sleep(0.2)
        command = 'CMD_SERVO#%d#%d\n' % (i, d1 + (d2 - d1) / 2)
        s.send(command.encode('utf-8'))
        time.sleep(0.2)
        command = 'CMD_SERVO#%d#%d\n' % (i, d2)
        s.send(command.encode('utf-8'))
        time.sleep(0.2)
        time.sleep(1)

        command = "CMD_SERVO#0#90\n"
        s.send(command.encode('utf-8'))
        time.sleep(1)

        # slowServoMove(1, 105, 50)
        # time.sleep(1)

        command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (0, 0, 0, 0)
        s.send(command.encode('utf-8'))

    def BuildGraph(self):
        self.G = nx.Graph()
        nodes = list(range(14))
        self.G.add_nodes_from(nodes)
        weightList = []
        for i in range(len(edgeList)):
            weight = getDistance(nodeToCo[edgeList[i][0]][0], nodeToCo[edgeList[i][0]][1], nodeToCo[edgeList[i][1]][0],
                                 nodeToCo[edgeList[i][1]][1])
            weightList.append((edgeList[i][0], edgeList[i][1], weight))
        print(weightList)
        self.G.add_weighted_edges_from(weightList)

    def SetWaypoints(self, cur, tar):
        self.wayPoint.clear()
        shortestPath = nx.shortest_path(self.G, cur, tar)
        for i in range(len(shortestPath)):
            self.wayPoint.append((nodeToCo[shortestPath[i]][0], nodeToCo[shortestPath[i]][1]))




rescue = Rescue()
rescue.Loop()
rescue.StopConnectToRobot()

