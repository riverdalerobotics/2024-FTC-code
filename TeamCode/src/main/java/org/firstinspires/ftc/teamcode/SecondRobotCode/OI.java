package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class OI {
    static Gamepad operatorController;
    static Gamepad driverController;
    public OI(Gamepad drive, Gamepad operator){
        operatorController = operator;
        driverController = drive;
    }
}
