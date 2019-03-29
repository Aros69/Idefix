#!/usr/bin/env python3
from ev3dev2.motor import Motor, LargeMotor, OUTPUT_A, OUTPUT_D, OUTPUT_C, OUTPUT_B,SpeedRPM, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds

def main():
    tank_drive = MoveTank(OUTPUT_A,  OUTPUT_C)
    #tank_drive.on_for_seconds(SpeedRPM(175), SpeedRPM(175), 1)
    tank_drive.on_for_rotations(SpeedPercent(50),SpeedPercent(50),5)
    #tank_drive = MoveTank(OUTPUT_B,  OUTPUT_D)
    #tank_drive.on_for_rotations(SpeedPercent(-50),SpeedPercent(50),5)

if __name__ == '__main__':
    main()
