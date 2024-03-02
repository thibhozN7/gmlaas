#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates

class SetPose:
    def __init__(self):
        rospy.init_node('set_pose')
        self.state_msg = ModelState()
        self.state_msg.model_name = 'my_object'
        self.state_msg.reference_frame= 'world'
        self.state_msg.pose.position.z = 1  # Altezza fissa
        self.state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

        # Variabili per memorizzare gli ultimi messaggi ricevuti dai due topic
        self.latest_cmd_vel_msg = None
        self.latest_model_state_msg = None

        # Subscribers per i due topic
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_state_callback)

    def cmd_vel_callback(self, msg):
        # Memorizza l'ultimo messaggio cmd_vel ricevuto
        self.latest_cmd_vel_msg = msg

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
        linear_x = self.latest_cmd_vel_msg.linear.x
        angular_z = self.latest_cmd_vel_msg.angular.z

        # Imposta le velocità lineari e angolari nel messaggio di stato
        self.state_msg.twist.linear.x = linear_x
        self.state_msg.twist.angular.z = angular_z

        # Recupera l'ultima posizione del robot dal messaggio ModelStates
        for i in range(len(self.latest_model_state_msg.name)):
            if self.latest_model_state_msg.name[i] == 'my_object':
                self.state_msg.pose = self.latest_model_state_msg.pose[i]
                break

        # Pubblica il nuovo stato del modello sul topic /gazebo/set_model_state
        self.state_pub.publish(self.state_msg)

def main():
    try:
        set_pose = SetPose()
        rospy.spin()  # Mantieni il nodo in esecuzione
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
