package org.firstinspires.ftc.teamcode.ThirdRobotCode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {
    CRServo intakeServo;
    Servo leftWrist;
    Servo rightWrist;
    ColorSensor colour;
    public IntakeSubsystem (CRServo intakeMotor, Servo leftWrist, ColorSensor colorSensor, Servo rightWrist){
        this.leftWrist = leftWrist;
        this.rightWrist = rightWrist;
        this.intakeServo = intakeMotor;
        this.colour = colorSensor;
    }
    public void pivotIntake(double angle){
        leftWrist.setPosition(angle);
        rightWrist.setPosition(-angle);
    }
    public void spinIntake(double power){
        intakeServo.setPower(power);
    }

    public char getColour(){
        char color = 'N';
        double distToWhite = Math.sqrt(Math.pow(255-colour.red(), 2)+Math.pow(255-colour.blue(), 2)+Math.pow(255-colour.green(), 2));
        double distToRed = Math.sqrt(Math.pow(255-colour.red(), 2)+Math.pow(0-colour.blue(), 2)+Math.pow(0-colour.green(), 2));
        double distToYellow = Math.sqrt(Math.pow(255-colour.red(), 2)+Math.pow(0-colour.blue(), 2)+Math.pow(255-colour.green(), 2));
        double distToBlue = Math.sqrt(Math.pow(0-colour.red(), 2)+Math.pow(255-colour.blue(), 2)+Math.pow(0-colour.green(), 2));
        double closestColour = Math.min(Math.min(distToYellow, distToWhite),Math.min(distToBlue, distToRed));
        if (closestColour == distToRed){
            color = 'r';
        }
        else if (closestColour == distToBlue){
            color = 'b';
        }

        else if (closestColour == distToYellow){
            color = 'y';
        }
        else{
            color = 'w';}
        return color;
    }
}
