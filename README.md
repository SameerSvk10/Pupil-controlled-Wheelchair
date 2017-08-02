# Pupil-controlled-Wheelchair
The concept of this project is to control the wheelchair depending on pupil movements of the patients. Image processing techniques are used to detect eye movements and corresponding signals are send to Arduino to control the wheelchair motors.
I have used OpenCV's inbuilt haar cascade classifiers for face detection and eyes detection.

I took reference for pupil detection from this repository : https://gist.github.com/edfungus/67c14af0d5afaae5b18c

Control signals for wheelchair are send to Arduino using Serial communication. Arduino recieves the command and control the motors accordingly.
