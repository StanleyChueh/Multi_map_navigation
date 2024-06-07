#!/usr/bin/env python

from argparse import ArgumentParser
import cv2
import apriltag

################################################################################

def apriltag_webcam(camera_index=0,
                    detection_window_name='AprilTag'):

    '''
    Detect AprilTags from webcam feed.

    Args:   camera_index [int]: Index of the webcam to use (usually 0 for default webcam)
            detection_window_name [str]: Title of displayed (output) tag detection window
    '''

    parser = ArgumentParser(description='Detect AprilTags from webcam feed.')
    apriltag.add_arguments(parser)
    options = parser.parse_args()

    '''
    Set up a reasonable search path for the apriltag DLL.
    Either install the DLL in the appropriate system-wide
    location, or specify your own search paths as needed.
    '''

    detector = apriltag.Detector(options, searchpath=apriltag._get_dll_path())

    video = cv2.VideoCapture(camera_index)

    while(video.isOpened()):

        success, frame = video.read()
        if not success:
            break

        result, overlay = apriltag.detect_tags(frame,
                                               detector,
                                               camera_params=(3156.71852, 3129.52243, 359.097908, 239.736909),
                                               tag_size=0.0762,
                                               vizualization=3,
                                               verbose=3,
                                               annotation=True
                                              )

        cv2.imshow(detection_window_name, overlay)
        if cv2.waitKey(1) & 0xFF == ord(' '): # Press space bar to terminate
            break

################################################################################

if __name__ == '__main__':
    apriltag_webcam()
