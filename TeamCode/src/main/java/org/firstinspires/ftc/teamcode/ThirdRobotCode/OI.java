package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import com.qualcomm.robotcore.hardware.Gamepad;

public class OI {
     Gamepad driverController;
     Gamepad operatorController;
    public OI(Gamepad driver, Gamepad operator){
        driverController = driver;
        operatorController = operator;
    }
    public  double ySpeed(){
        return driverController.left_stick_y;
    }
    public  double xSpeed(){
        return driverController.left_stick_x;
    }
    public  double turnSpeed(){
        return driverController.right_stick_x;
    }
    public  boolean climbPartOne(){
        return driverController.a;
    }
    public  boolean climbPartTwo(){
        return driverController.b;
    }
    public boolean basicScore(){
        return operatorController.a;
    }
    public  boolean basicIntake(){
        return operatorController.b;
    }
    public  boolean fancyScore(){
        return operatorController.right_bumper;
    }
    public  boolean spit(){return operatorController.left_trigger>0.3;}
    public  boolean fancyIntake(){
        return operatorController.left_bumper;
    }
    public  double moveArmForTest(){
        return operatorController.left_stick_y;
    }

}
