import cv2
import cv2.data

from moviepy.editor import VideoFileClip, concatenate_videoclips

import threading


def thread1(last_frame):
    # load video file & frontal face cascade
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

    # output1 = cv2.VideoWriter("output1.mp4", 0x7634706d, fps, (1280, 720))
    frame = 0
    video1 = video
    video1.set(cv2.CAP_PROP_POS_FRAMES, frame)
    success, frame = video1.read()
    while success:
        if video1.get(cv2.CAP_PROP_POS_FRAMES) == last_frame:
            break
        resize = cv2.resize(frame, (1280, 720))
        # use cascade to detect frontal faces
        faces = face_cascade.detectMultiScale(resize, 1.4, 4)

        # use returned coordinates to draw a frame
        for (x, y, w, h) in faces:
            cv2.rectangle(resize, (x, y), (x + w, y + h), (30, 211, 198), 3)

        # write frame to empty output
        output1.write(resize)

        # read next frame to start the loop
        success, frame = video1.read()


def thread2(first_frame, last_frame):
    # load video file & frontal face cascade
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

    # output2 = cv2.VideoWriter("output2.mp4", 0x7634706d, fps, (1280, 720))
    frame = first_frame
    video2 = video
    video2.set(cv2.CAP_PROP_POS_FRAMES, frame)
    success, frame = video2.read()
    while success:
        if video2.get(cv2.CAP_PROP_POS_FRAMES) == last_frame:
            break
        resize = cv2.resize(frame, (1280, 720))
        # use cascade to detect frontal faces
        faces = face_cascade.detectMultiScale(resize, 1.4, 4)

        # use returned coordinates to draw a frame
        for (x, y, w, h) in faces:
            cv2.rectangle(resize, (x, y), (x + w, y + h), (30, 211, 198), 3)

        # write frame to empty output
        output2.write(resize)

        # read next frame to start the loop
        success, frame = video2.read()


def thread3(first_frame, last_frame):
    # load video file & frontal face cascade
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

    # output3 = cv2.VideoWriter("output3.mp4", 0x7634706d, fps, (1280, 720))
    frame = first_frame
    video3 = video
    video3.set(cv2.CAP_PROP_POS_FRAMES, frame)
    success, frame = video3.read()
    while success:
        if video3.get(cv2.CAP_PROP_POS_FRAMES) == last_frame:
            break
        resize = cv2.resize(frame, (1280, 720))
        # use cascade to detect frontal faces
        faces = face_cascade.detectMultiScale(resize, 1.4, 4)

        # use returned coordinates to draw a frame
        for (x, y, w, h) in faces:
            cv2.rectangle(resize, (x, y), (x + w, y + h), (30, 211, 198), 3)

        # write frame to empty output
        output3.write(resize)

        # read next frame to start the loop
        success, frame = video3.read()


def thread4(first_frame):
    # load video file & frontal face cascade
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

    # output4 = cv2.VideoWriter("output4.mp4", 0x7634706d, fps, (1280, 720))
    frame = first_frame
    video4 = video
    video4.set(cv2.CAP_PROP_POS_FRAMES, frame)
    success, frame = video4.read()
    while success:
        resize = cv2.resize(frame, (1280, 720))
        # use cascade to detect frontal faces
        faces = face_cascade.detectMultiScale(resize, 1.4, 4)

        # use returned coordinates to draw a frame
        for (x, y, w, h) in faces:
            cv2.rectangle(resize, (x, y), (x + w, y + h), (30, 211, 198), 3)

        # write frame to empty output
        output4.write(resize)

        # read next frame to start the loop
        success, frame = video4.read()


video = cv2.VideoCapture('video.mp4')

width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
fps = int(video.get(cv2.CAP_PROP_FPS))

initialFrame2 = int(frames/4)
initialFrame3 = int((frames/4)*2)
initialFrame4 = int((frames/4)*3)

output1 = cv2.VideoWriter("output1.mp4", 0x7634706d, fps, (1280, 720))
output2 = cv2.VideoWriter("output2.mp4", 0x7634706d, fps, (1280, 720))
output3 = cv2.VideoWriter("output3.mp4", 0x7634706d, fps, (1280, 720))
output4 = cv2.VideoWriter("output4.mp4", 0x7634706d, fps, (1280, 720))

t1 = threading.Thread(target=thread1, args=(initialFrame2, ))
t2 = threading.Thread(target=thread2, args=(initialFrame2, initialFrame3))
t3 = threading.Thread(target=thread3, args=(initialFrame3, initialFrame4))
t4 = threading.Thread(target=thread4, args=(initialFrame4,))

print(width, height, frames, fps)

t1.start()
t2.start()
t3.start()
t4.start()

t1.join()
t2.join()
t3.join()
t4.join()

clip_1 = VideoFileClip("output1.mp4")
clip_2 = VideoFileClip("output1.mp4")
clip_3 = VideoFileClip("output1.mp4")
clip_4 = VideoFileClip("output1.mp4")
final_clip = concatenate_videoclips([clip_1, clip_2, clip_3, clip_4])
final_clip.write_videofile("final.mp4")

print("A")
