package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import com.qualcomm.robotcore.hardware.Gamepad;

public class OI {
    static Gamepad driverController;
    static Gamepad operatorController;
    public OI(Gamepad driver, Gamepad operator){
        driverController = driver;
        operatorController = operator;
    }
    public static double ySpeed(){
        return driverController.left_stick_y;
    }
    public static double xSpeed(){
        return driverController.left_stick_x;
    }
    public static double turnSpeed(){
        return driverController.right_stick_x;
    }
    public static boolean climbPartOne(){
        return driverController.a;
    }
    public static boolean climbPartTwo(){
        return driverController.b;
    }
    public static boolean basicScore(){
        return operatorController.a;
    }
    public static boolean basicIntake(){
        return operatorController.b;
    }
    public static boolean fancyScore(){
        return operatorController.right_bumper;
    }
    public static boolean fancyIntake(){
        return operatorController.left_bumper;
    }
    public static double moveArmForTest(){
        return operatorController.left_stick_y;
    }

}
