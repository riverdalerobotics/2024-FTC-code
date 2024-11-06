package org.firstinspires.ftc.teamcode.RobotCode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class OI{
    static Gamepad operatorController;
    static Gamepad driverController;
    public OI(Gamepad firstStick, Gamepad secondStick){
        operatorController = secondStick;
        driverController = firstStick;
    }


}
