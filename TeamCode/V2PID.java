package org.firstinspires.ftc.teamcode.V2;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

public class V2PID {
    public double kP; //Error Proportion Constant for tuning
    public double kD; //Derivative Proportion Constant for tuning
    public double kI; //Integral Proportion Constant for tuning
    public static double power;
    public static double error;
    public static double integral=0;
    double OldError=0;
    ElapsedTime timer = new ElapsedTime();

    double CurrentTime;
    public V2PID(double kP, double kI, double kD) {
        this.kP=kP;
        this.kI=kI;
        this.kD=kD;
    }
    public double Calculate(double TP, double CP){ //TP = Target Position
            FtcDashboard dashboard= FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();

            error=(TP-CP);
            CurrentTime=timer.milliseconds();
            double dT=CurrentTime; //Change in time
            integral= (error*dT  + integral);
            if (abs(integral)>4000000){
                integral=0;
            }
            double derivative= ((error-OldError)/dT);
            power= (error*kP)+(derivative*kD)+(integral*kI);
            packet.put("Error", error);
            packet.put("Integral", integral);
            packet.put("Derivative", derivative);
            packet.put("power", power);
            packet.put("Time", dT);
            dashboard.sendTelemetryPacket(packet);

            OldError=error;
            timer.reset();
            return power;
    }

}
