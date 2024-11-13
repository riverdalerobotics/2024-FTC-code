package org.firstinspires.ftc.teamcode.RobotCode;
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
        //todo: map this to bumpers
    }
    public static double moveIntake(){
        return operatorController.right_stick_y;
    }

    public static boolean testArmAngle(){
        return operatorController.b;
    }

}
