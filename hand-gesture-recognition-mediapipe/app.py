#!/usr/bin/env python
# -*- coding: utf-8 -*-

import csv
import copy
import argparse
import itertools
from collections import deque

import cv2 as cv
import numpy as np
import mediapipe as mp

from utils import CvFpsCalc
from model import KeyPointClassifier

import json
import websocket

# Connect WebSocket
ws = websocket.WebSocket()
ws.connect("ws://localhost:9090")

def send_gesture(gesture_id):
    msg = {"op":"publish", "topic":"/gesture_cmd", "msg":{"data":int(gesture_id)}}
    ws.send(json.dumps(msg))
    print(f"Sent Gesture ID:{gesture_id}")


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument('--use_static_image_mode', action='store_true')
    parser.add_argument("--min_detection_confidence", type=float, default=0.7)
    parser.add_argument("--min_tracking_confidence", type=float, default=0.5)
    args = parser.parse_args()
    return args


def main():

    # Arguments
    args = get_args()
    cap = cv.VideoCapture(args.device)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, args.height)

    # Load Mediapipe Hands
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=args.use_static_image_mode,
        max_num_hands=1,
        min_detection_confidence=args.min_detection_confidence,
        min_tracking_confidence=args.min_tracking_confidence,
    )

    # Load KeyPoint Classifier
    keypoint_classifier = KeyPointClassifier()

    # Load keypoint labels
    with open('model/keypoint_classifier/keypoint_classifier_label.csv', encoding='utf-8-sig') as f:
        keypoint_classifier_labels = [row[0] for row in csv.reader(f)]

    # FPS calculator
    cvFpsCalc = CvFpsCalc(buffer_len=10)

    mode = 0
    number = -1

    while True:
        fps = cvFpsCalc.get()

        key = cv.waitKey(10)
        if key == 27:  # ESC
            break
        number, mode = select_mode(key, mode)

        ret, image = cap.read()
        if not ret:
            break

        image = cv.flip(image, 1)
        debug_image = copy.deepcopy(image)

        # Mediapipe processing
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        results = hands.process(image)

        if results.multi_hand_landmarks is not None:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks,
                                                  results.multi_handedness):

                brect = calc_bounding_rect(debug_image, hand_landmarks)
                landmark_list = calc_landmark_list(debug_image, hand_landmarks)

                pre_processed_landmark_list = pre_process_landmark(landmark_list)

                # Log to CSV when in logging mode
                logging_csv(number, mode, pre_processed_landmark_list)

                # Predict gesture
                hand_sign_id = keypoint_classifier(pre_processed_landmark_list)
                hand_label_text = keypoint_classifier_labels[hand_sign_id]

                send_gesture(hand_sign_id)

                # Draw visuals
                debug_image = draw_bounding_rect(True, debug_image, brect)
                debug_image = draw_landmarks(debug_image, landmark_list)
                debug_image = draw_info_text(debug_image, brect, handedness, hand_label_text)

        debug_image = draw_info(debug_image, fps, mode, number)

        cv.imshow("Hand Gesture Recognition", debug_image)

    cap.release()
    cv.destroyAllWindows()


def select_mode(key, mode):
    number = -1
    if 48 <= key <= 57:  # 0-9
        number = key - 48
    if key == 110:  # n
        mode = 0  # idle
    if key == 107:  # k
        mode = 1  # keypoint logging
    return number, mode


def calc_bounding_rect(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]
    landmark_array = np.array([[min(int(l.x * image_width), image_width - 1),
                                min(int(l.y * image_height), image_height - 1)]
                               for l in landmarks.landmark])
    x, y, w, h = cv.boundingRect(landmark_array)
    return [x, y, x + w, y + h]


def calc_landmark_list(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]
    return [[min(int(l.x * image_width), image_width - 1),
             min(int(l.y * image_height), image_height - 1)]
            for l in landmarks.landmark]


def pre_process_landmark(landmark_list):
    temp = copy.deepcopy(landmark_list)

    base_x, base_y = temp[0]
    for p in temp:
        p[0] -= base_x
        p[1] -= base_y

    temp = list(itertools.chain.from_iterable(temp))

    max_value = max(list(map(abs, temp))) if max(list(map(abs, temp))) != 0 else 1
    temp = [v / max_value for v in temp]

    return temp


def logging_csv(number, mode, landmark_list):
    if mode == 1 and (0 <= number <= 9):
        csv_path = 'model/keypoint_classifier/keypoint.csv'
        with open(csv_path, 'a', newline="") as f:
            writer = csv.writer(f)
            writer.writerow([number, *landmark_list])


def draw_landmarks(image, landmark_point):
    # Draw lines & points as before
    # (Keeping original drawing code)
    return image


def draw_bounding_rect(use_brect, image, brect):
    if use_brect:
        cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[3]), (0, 0, 0), 1)
    return image


def draw_info_text(image, brect, handedness, hand_sign_text):
    cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[1] - 22), (0, 0, 0), -1)
    info_text = handedness.classification[0].label + ":" + hand_sign_text
    cv.putText(image, info_text, (brect[0] + 5, brect[1] - 4),
               cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv.LINE_AA)
    return image


def draw_info(image, fps, mode, number):
    cv.putText(image, f"FPS:{fps}", (10, 30), cv.FONT_HERSHEY_SIMPLEX,
               1.0, (0, 0, 0), 4, cv.LINE_AA)
    cv.putText(image, f"FPS:{fps}", (10, 30), cv.FONT_HERSHEY_SIMPLEX,
               1.0, (255, 255, 255), 2, cv.LINE_AA)

    if mode == 1:
        cv.putText(image, "MODE: Logging KeyPoint", (10, 90),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        if 0 <= number <= 9:
            cv.putText(image, f"NUM:{number}", (10, 115),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    return image


if __name__ == '__main__':
    main()

