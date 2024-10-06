package org.firstinspires.ftc.teamcode;

public class HelperFunctions {
    public static double pController(double kp, double setpoint, double pos){
        double error = setpoint-pos;
        double output = error * kp;
        return output;
    }

    public static double iController(double ki, double setPoint, double pos){
        double sum = 0;
        double error = setPoint - pos;
        sum += error;
        return ki*sum;
    }

    public static double dController(double kd, double setpoint, double pos){
        double error = setpoint - pos;
        double olddiff = error;
        double deltaError = error - olddiff;
        olddiff = error;
        double speed = kd*deltaError;
        return speed;
    }


    public static double piController(double kp, double ki, double setPoint, double pos){
        double speed = pController(kp, setPoint, pos) + iController(ki, setPoint, pos);
        return speed;
    }
    public static double pidController(double kp, double ki, double kd, double setPoint, double pos){
        double speed = pController(kp, setPoint, pos) + iController(ki, setPoint, pos) + dController(kd, setPoint, pos);
        return speed;
    }

}
