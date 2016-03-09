#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from edwin.msg import *
import time
import subprocess
import alsaaudio, time, audioop

class EdwinAudioDetection():
	def __init__(self):
		rospy.init_node('edwin_audio_node', anonymous=True)
		pub = rospy.Publisher('edwin_sound', String, queue_size=10)

		mic = alsaaudio.PCM(alsaaudio.PCM_CAPTURE,alsaaudio.PCM_NONBLOCK)
		# Set attributes: Mono, 8000 Hz, 16 bit little endian samples
		mic.setchannels(1)
		mic.setrate(8000)
		mic.setformat(alsaaudio.PCM_FORMAT_S16_LE)
		mic.setperiodsize(160)

	def calibrate():
		print 'calibrating'
		timer = 0
		average_list = []
		while timer < 3.0:
			l,data = self.mic.read()
			test_sound = audioop.max(data, 2)
			if test_sound != 0:
				average_list.append(test_sound) #sampling sound in 2 second interval
			timer += .01

		thresh = int(sum(average_list)/float(len(average_list))) #average volume.

		print thresh, "ready"
		return thresh

	def run(self):
		absolute_threshold = 2030

		while not rospy.is_shutdown():
			threshold = self.calibrate()
			if threshold > absolute_threshold:
				print 'Threshold too high.  Recalibrating'
				threshold = self.calibrate()

			soundbite = 0
			bite_length = 0
			peak_volume = 0

			cont = False
			while (cont == False) or peak_volume == 0:
				#only breaks out of loop when peak volume conditions are met
				l,data = self.mic.read()
				if l:
					level = audioop.max(data, 2)
					if level > threshold:
						print "Listening Session:", str(soundbite)
						if cont:
							soundbite += 1
							cont = False
						if level > peak_volume:
							peak_volume = level
					elif level < threshold: #Once hit under threshold, it will recognize next sound.
						cont = True
				time.sleep(.01)
				if peak_volume > 0:
					bite_length += .01

			self.pub.publish(str(bite_length) + ' ' + str(peak_volume))

if __name__ == '__main__':
	audio_eng = EdwinAudioDetection()
	audio_eng.run()