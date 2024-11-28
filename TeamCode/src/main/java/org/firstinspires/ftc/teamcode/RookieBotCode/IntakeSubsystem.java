package org.firstinspires.ftc.teamcode.RookieBotCode;
import org.firstinspires.ftc.teamcode.RookieBotCode.Constants;
import com.acmerobotics.dashboard.DashboardCore;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem{

    CRServo intake;
    Servo claw;
    DcMotor intakeMotor;

    public IntakeSubsystem(CRServo intake, Servo lift, DcMotor intakeMotor){
        this.intake = intake;
        this.claw = lift;
        this.intakeMotor = intakeMotor;
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double intakeInDeg(){
        return intakeMotor.getCurrentPosition()*360*Constants.IntakeConstants.GEAR_RATIO;
    }
    public void moveArmToPos(double angle, double speed){
        intakeMotor.setPower(speed);
        intakeMotor.setTargetPosition((int)(angle/(360*Constants.IntakeConstants.GEAR_RATIO)));
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void grabWithClaw(double pos){
        claw.setPosition(pos);
    }
    public void openClaw(){
        claw.setPosition(1);
    }
    public void closeClaw(){
        claw.setPosition(0);
    }

    public void spinTake(double power){
        intake.setPower(power);
    }

    public void resetPos(){
        claw.setPosition(0);
    }
}
