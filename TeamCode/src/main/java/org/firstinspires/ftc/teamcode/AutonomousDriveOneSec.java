package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutonomousDriveOneSec extends AutonomousDriveBase {

//    private final double drivePidKp = 1;     // Tuning variable for PID.
//    private final double drivePidTi = 1.0;   // Eliminate integral error in 1 sec.
//    private final double drivePidTd = 0.1;   // Account for error in 0.1 sec.
//    // Protect against integral windup by limiting integral term.
//    private final double drivePidIntMax = 1.0;  // Limit to max speed.
//    private final double driveOutMax = 1.0;  // Motor output limited to 100%.


    @Override
    public void Run() throws InterruptedException {

            motorWheelFL.setPower(0.5);
            motorWheelFR.setPower(-0.5);
            motorWheelBL.setPower(0.5);
            motorWheelBR.setPower(-0.5);
            sleep(750);
            motorWheelFL.setPower(0);
            motorWheelFR.setPower(0);
            motorWheelBL.setPower(0);
            motorWheelBR.setPower(0);
    }
}