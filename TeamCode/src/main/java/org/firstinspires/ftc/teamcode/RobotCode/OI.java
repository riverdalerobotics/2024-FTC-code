package org.firstinspires.ftc.teamcode.RobotCode;

import com.fasterxml.jackson.databind.deser.std.UntypedObjectDeserializer;
import com.qualcomm.robotcore.hardware.Gamepad;

public class OI{
    static Gamepad operatorController;
    static Gamepad driverController;
    public OI(Gamepad firstStick, Gamepad secondStick){
        operatorController = secondStick;
        driverController = firstStick;
    }
    public static double speed(){
        return driverController.left_stick_y;
    }
    public static double turn(){
        return driverController.right_stick_x;
    }
    public static double moveArm(){
        return operatorController.left_stick_y;
    }
    public static double moveWrist(){
        return 0.0;
    }
    public static double moveIntake(){
        return operatorController.right_stick_y;
    }

}
