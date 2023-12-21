#!/usr/bin/env python

import subprocess

def play_sound(file_path):
    rospy.init_node('sound_play_text_example')
    command = f"aplay {file_path}"  # 파일 경로를 포함한 명령어 생성
    subprocess.call(command, shell=True)  # 명령어 실행

if __name__ == "__main__":
    file_to_play = "/home/wego/wego_ws/car.wav"  # 재생할 WAV 파일 경로
    play_sound(file_to_play)  # 소리 재생 함수 호출
