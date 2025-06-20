#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sound_play.libsoundplay import SoundClient
import threading
import time

class SoundPlayer:
    def __init__(self):
        rospy.init_node('sound_player')
        rospy.loginfo("sound_player started!")

        # Parameters
        self.warning_sound_path = rospy.get_param('~warning_sound_path', '/home/u/tko_ws/src/obstacle_stopper/voice/excuesme.wav')
        self.sound_play_duration = 4.0  # Duration of the sound file in seconds

        # Initialize sound client
        self.soundhandle = SoundClient()

        # Initialize sound_playing
        self.sound_playing = False
        self.play_sound_thread = None

        # Subscriber
        rospy.Subscriber('/play_warning_sound', Bool, self.play_warning_sound_callback)

    def play_warning_sound_callback(self, msg):
        if msg.data:
            if not self.sound_playing:
                self.sound_playing = True
                self.play_sound_thread = threading.Thread(target=self.play_warning_sound)
                self.play_sound_thread.start()

    def play_warning_sound(self):
        rospy.loginfo("Playing warning sound")
        self.soundhandle.playWave(self.warning_sound_path)
        time.sleep(self.sound_play_duration)
        self.sound_playing = False
        rospy.loginfo("Finished playing warning sound")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        sound_player = SoundPlayer()
        sound_player.run()
    except rospy.ROSInterruptException:
        pass
