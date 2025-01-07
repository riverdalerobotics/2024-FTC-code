package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubsystem extends SubsystemBase{
    DcMotorEx armPivotMotor;


     double integralSum = 0;
     //double Kp = 0.0006; moved to constants
     //double Ki = 0;

     //double Kd = 0;
     //double Kf = 1;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    int armPivotMotorPosition;
    PIDFCoefficients values;
    MultipleTelemetry telemetry;

    /**
     * The subsystem of the arm
     * @param armPivot the arm motor, must be static and DcMotor<b>EX</b>
     */
    public ArmSubsystem(DcMotorEx armPivot, MultipleTelemetry telemetry) {
        values = new PIDFCoefficients(Constants.ArmConstants.kp, Constants.ArmConstants.ki, Constants.ArmConstants.kd, Constants.ArmConstants.kf);
        this.armPivotMotor = armPivot;
        armPivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.telemetry = telemetry;
        armPivotMotor.setDirection(DcMotor.Direction.REVERSE);
    }

//    public void pivotArm(double armPivotInput, double power) {
//
//        armPivotMotorPosition += (int) armPivotInput;
//        armPivotMotor.setPower(power);
//        armPivotMotor.setTargetPosition(armPivotMotorPosition);
//    }
// Not necessary
//    public double PIDControl(double reference, double state) {
//        double error = reference - state;
//        integralSum += error * timer.seconds();
//        double derivitave = (error - lastError) / timer.seconds();
//
//        timer.reset();
//
//        return (error * Kp) + (derivitave * Kd) + (integralSum * Ki);
//
//    }
    public double getPos(){
        return 360*armPivotMotor.getCurrentPosition()/Constants.ArmConstants.GEAR_RATIO;
    }
    public boolean isBusy(){
        return armPivotMotor.isBusy();
    }
    public void pivotArm(double angle, double speed, PIDFCoefficients pidfCoefficients){

        double rotations = angle*Constants.ArmConstants.GEAR_RATIO/360;
        armPivotMotor.setPower(speed);
        armPivotMotor.setTargetPosition((int)rotations);
        armPivotMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        armPivotMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(pidfCoefficients.p, 0 , 0, 0));
        armPivotMotor.setTargetPositionTolerance((int)(3*Constants.ArmConstants.GEAR_RATIO/360));

        armPivotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

    }
    @Override
    public void periodic(){
        super.periodic();
        telemetry.addData("Current arm position", getPos());
    }

}