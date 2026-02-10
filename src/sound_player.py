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
        self.still_sound_path = rospy.get_param('~still_sound_path', '/home/u/tko_ws/src/obstacle_stopper/voice/still.wav')
        self.rotation_sound_path = rospy.get_param('~rotation_sound_path', '/home/u/tko_ws/src/obstacle_stopper/voice/rotation.wav')
        self.enterlift_sound_path = rospy.get_param('~enterlift_sound_path', '/home/u/tko_ws/src/obstacle_stopper/voice/enterlift.wav')
        self.cartdock_sound_path = rospy.get_param('~cartdock_sound_path', '/home/u/tko_ws/src/obstacle_stopper/voice/single_do.wav') # Added cartdock sound path

        self.warning_sound_play_duration = rospy.get_param('~warning_sound_play_duration', 4.0)
        self.still_sound_play_duration = rospy.get_param('~still_sound_play_duration', 7.0)
        self.rotation_sound_play_duration = rospy.get_param('~rotation_sound_play_duration', 4.0)
        self.enterlift_sound_play_duration = rospy.get_param('~enterlift_sound_play_duration', 5.0)
        self.cartdock_sound_play_duration = rospy.get_param('~cartdock_sound_play_duration', 1.0) # Added cartdock sound duration

        # Initialize sound client
        self.soundhandle = SoundClient()

        # Flags
        self.sound_playing = False
        self.run_enabled = True  # Set by /play_sound topic
        self.sound_lock = threading.Lock()
        self.play_sound_thread = None

        # Subscribers
        rospy.Subscriber('/play_warning_sound', Bool, self.play_warning_sound_callback)
        rospy.Subscriber('/play_still_sound', Bool, self.play_still_sound_callback)
        rospy.Subscriber('/play_rotation_sound', Bool, self.play_rotation_sound_callback)
        rospy.Subscriber('/play_enterlift_sound', Bool, self.play_enterlift_sound_callback)
        rospy.Subscriber('/play_sound', Bool, self.play_sound_enable_callback)
        rospy.Subscriber('/play_cartdock_sound', Bool, self.play_cartdock_sound_callback) # Added cartdock sound subscriber

    def play_sound_enable_callback(self, msg):
        self.run_enabled = msg.data
        rospy.loginfo(f"Sound player enabled: {self.run_enabled}")

    def play_warning_sound_callback(self, msg):
        if self.run_enabled and msg.data:
            self.play_sound(self.warning_sound_path, self.warning_sound_play_duration)

    def play_still_sound_callback(self, msg):
        if self.run_enabled and msg.data:
            self.play_sound(self.still_sound_path, self.still_sound_play_duration)

    def play_rotation_sound_callback(self, msg):
        if msg.data:
            self.play_sound(self.rotation_sound_path, self.rotation_sound_play_duration)

    def play_enterlift_sound_callback(self, msg):
        if msg.data:
            self.play_sound(self.enterlift_sound_path, self.enterlift_sound_play_duration)

    def play_cartdock_sound_callback(self, msg): # Added cartdock sound callback
        if msg.data:
            self.play_sound(self.cartdock_sound_path, self.cartdock_sound_play_duration)

    def play_sound(self, sound_path, sound_play_duration):
        with self.sound_lock:
            if not self.sound_playing:
                self.sound_playing = True
                self.play_sound_thread = threading.Thread(target=self._play_sound, args=(sound_path, sound_play_duration))
                self.play_sound_thread.start()

    def _play_sound(self, sound_path, sound_play_duration):
        rospy.loginfo(f"Playing sound: {sound_path}")
        self.soundhandle.playWave(sound_path)
        time.sleep(sound_play_duration)
        with self.sound_lock:
            self.sound_playing = False
        rospy.loginfo("Finished playing sound")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        sound_player = SoundPlayer()
        sound_player.run()
    except rospy.ROSInterruptException:
        pass