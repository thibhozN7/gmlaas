#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
import subprocess
class ObjectMove:
    def __init__(self):
        
        rosbag = rospy.get_param('~rosbag_value', 'False')       
        if rosbag == True:
            command = ['rosbag', 'record', '-O', 'my_bagfile.bag', '/data/pre_h_computation', '/gazebo/model_states']
            process = subprocess.Popen(command)
        

        rospy.init_node('object_move_py',anonymous=False)
        self.state_msg = ModelState()
        self.state_msg.model_name = 'robot'
        self.state_msg.reference_frame= 'world'
        self.state_msg.pose.position.z = 1  # Altezza fissa
        self.state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

        # Variabili per memorizzare gli ultimi messaggi ricevuti dai due topic
        self.latest_cmd_vel_msg = None
        self.latest_model_state_msg = None

        # Subscribers per i due topic
        rospy.Subscriber('/visual_servoing_command/velocity_ref', TwistStamped, self.cmd_vel_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_state_callback)

    def cmd_vel_callback(self, msg):
        # Memorizza l'ultimo messaggio cmd_vel ricevuto
        self.latest_cmd_vel_msg = msg.twist

        # Se entrambi i messaggi sono stati ricevuti, esegui l'aggiornamento dello stato del modello
        if self.latest_model_state_msg is not None:
            self.update_model_state()

    def model_state_callback(self, msg):
        # Memorizza l'ultimo messaggio get_model_state ricevuto
        self.latest_model_state_msg = msg

        # Se entrambi i messaggi sono stati ricevuti, esegui l'aggiornamento dello stato del modello
        if self.latest_cmd_vel_msg is not None:
            self.update_model_state()

    def update_model_state(self):
        # Recupera i dati di velocità lineare e angolare dal messaggio cmd_vel
        # Convert from the ROS convention (x-forward, y-left, z-up) to the Gazebo convention (z-forward, -x-left, -y-up)
        linear_x = self.latest_cmd_vel_msg.linear.z
        linear_y = -self.latest_cmd_vel_msg.linear.x
        linear_z = -self.latest_cmd_vel_msg.linear.y
        angular_x = self.latest_cmd_vel_msg.angular.z
        angular_y = -self.latest_cmd_vel_msg.angular.x
        angular_z = -self.latest_cmd_vel_msg.angular.y

        # Imposta le velocità lineari e angolari nel messaggio di stato
        self.state_msg.twist.linear.x = linear_x
        self.state_msg.twist.linear.y = linear_y
        self.state_msg.twist.linear.z = linear_z
        self.state_msg.twist.angular.x = angular_x
        self.state_msg.twist.angular.y = angular_y
        self.state_msg.twist.angular.z = angular_z

        # Recupera l'ultima posizione del robot dal messaggio ModelStates
        for i in range(len(self.latest_model_state_msg.name)):
            if self.latest_model_state_msg.name[i] == 'robot':
                self.state_msg.pose = self.latest_model_state_msg.pose[i]
                break

        # Pubblica il nuovo stato del modello sul topic /gazebo/set_model_state
        self.state_pub.publish(self.state_msg)

def main():
    try:
        object_move = ObjectMove()
        rospy.spin()  # Mantieni il nodo in esecuzione
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
