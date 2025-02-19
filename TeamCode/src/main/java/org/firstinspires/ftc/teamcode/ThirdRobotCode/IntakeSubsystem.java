package org.firstinspires.ftc.teamcode.ThirdRobotCode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    CRServo intakeServo;
    Servo leftWrist;
    Servo rightWrist;
    RevColorSensorV3 colour;
    Telemetry telemetry;
    public IntakeSubsystem (CRServo intakeMotor, Servo leftWrist, RevColorSensorV3 colorSensor, Servo rightWrist, Telemetry telemetry){
        this.leftWrist = leftWrist;
        this.rightWrist = rightWrist;
        this.intakeServo = intakeMotor;
        this.colour = colorSensor;
        this.telemetry = telemetry;
        rightWrist.setDirection(Servo.Direction.REVERSE);
    }
    public void pivotIntake(double angle){
        leftWrist.setPosition(angle);
        rightWrist.setPosition(angle);
    }
    public void spinIntake(double power){
        intakeServo.setPower(power);
    }
    public static void intakeWait(int ms)
    {
        try
        {
            Thread.sleep(ms);
        }
        catch(InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }
    public char getColour(){

        char color = 'w';
        NormalizedRGBA normalizedcolours = colour.getNormalizedColors();
        double [] rgb = {colour.red(), colour.green(), colour.blue()};
        double max = Constants.IntakeConstants.max*((rgb[0]+rgb[1]+rgb[2])/3);
        double min = Constants.IntakeConstants.min*((rgb[0]+rgb[1]+rgb[2])/3);
        if(rgb[0]>rgb[1]+max&&rgb[0]>rgb[2]+max){
            color='r';
        } else if (rgb[2]>rgb[1]+max&&rgb[2]>rgb[0]+max) {
            color='b';
        } else if (rgb[1]>rgb[2]+max&&rgb[0]>rgb[2]+max&&Math.abs(rgb[0]-rgb[1])<min) {
            color = 'y';
        }
//        double distToWhite = Math.sqrt(Math.pow(min-rgb[0], 2)+Math.pow(min-rgb[1], 2)+Math.pow(min-rgb[2], 2));
//        double distToRed = Math.sqrt(Math.pow(max-rgb[0], 2)+Math.pow(min-rgb[1], 2)+Math.pow(min-rgb[2], 2));
//        double distToYellow = Math.sqrt(Math.pow(max+200-rgb[0], 2)+Math.pow(max+200-rgb[1], 2)+Math.pow(min-rgb[2], 2));
//        double distToBlue = Math.sqrt(Math.pow(min-rgb[0], 2)+Math.pow(min-rgb[1], 2)+Math.pow(max-rgb[2], 2));
//        double closestColour = Math.min(Math.min(distToYellow, distToWhite),Math.min(distToBlue, distToRed));
//        if (closestColour == distToRed){
//            color = 'r';
//        }
//        else if (closestColour == distToBlue){
//            color = 'b';
//        }
//
//        else if (closestColour == distToYellow){
//            color = 'y';
//        }
//        else{
//            color = 'w';}
//
//
        return color;
    }

    @Override
    public void periodic() {
        telemetry.addData("Color", getColour());
        double [] rgb = {colour.red(), colour.green(), colour.blue()};
        telemetry.addData("R:", rgb[0]+" G " +rgb[1]+ " B " + rgb[2]);

    }
}
