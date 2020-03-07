package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOP extends OpMode {

    private DcMotor motorWheelFL;
    private DcMotor motorWheelFR;
    private DcMotor motorWheelBL;
    private DcMotor motorWheelBR;
    //private Servo servoClamp;
    private Servo trayOne;
    private Servo trayTwo;
    private Servo cockAndBallTorture;
    private CRServo hentai;
    private CRServo servoWheelOne;
    private CRServo servoWheelTwo;
    private DcMotor motorLift;
    private DcMotor collectionWheelsL;
    private DcMotor collectionWheelsR;
    //private DcMotor motorTray;
    //private DcMotor motorTray2;
    private double V1;
    private double V2;
    private double V3;
    private double V4;
    private boolean trayDown;
    private boolean blockGrabbed;
    private double timer1;
    private double timer2;
    private double servoPower;
    //private double timer3;

    @Override
    public void init() {
        motorWheelFL = hardwareMap.get(DcMotor.class, "motorWheelFL");
        motorWheelFR = hardwareMap.get(DcMotor.class, "motorWheelFR");
        motorWheelBL = hardwareMap.get(DcMotor.class, "motorWheelBL");
        motorWheelBR = hardwareMap.get(DcMotor.class, "motorWheelBR");
        motorLift = hardwareMap.get(DcMotor.class, "motorLift");
        //servoClamp = hardwareMap.get(Servo.class, "servoClamp");
        trayOne = hardwareMap.get(Servo.class, "trayOne");
        trayTwo = hardwareMap.get(Servo.class, "trayTwo");
        cockAndBallTorture = hardwareMap.get(Servo.class, "cockAndBallTorture");
        hentai = hardwareMap.get(CRServo.class, "hentai");
        servoWheelOne = hardwareMap.get(CRServo.class, "servoWheelOne");
        servoWheelTwo = hardwareMap.get(CRServo.class, "servoWheelTwo");
        collectionWheelsL = hardwareMap.get(DcMotor.class, "collectionWheelsL");
        collectionWheelsR = hardwareMap.get(DcMotor.class, "collectionWheelsR");

        motorWheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorWheelFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorWheelFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorWheelFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorWheelBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorWheelBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorWheelBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorWheelBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectionWheelsR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collectionWheelsR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectionWheelsL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collectionWheelsL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorTray.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorTray2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorTray2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        trayDown = false;
        blockGrabbed = false;
        timer1 = 0;
        timer2 = 0;
        //	telemetry.addData("Status", "Initialized");
//        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
    }



        // run until the end of the match (driver presses STOP)
        @Override
        public void loop() {
//            telemetry.addData("Status", "Running");

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) - rightX;
            final double v2 = -r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) - rightX;
            final double v4 = -r * Math.cos(robotAngle) - rightX;

            double c1=v1*0.5;
            double c2=v2*0.5;
            double c3=v3*0.5;
            double c4=v4*0.5;

//            double WheelBL= motorWheelBL.getPower();
//            double WheelBR= motorWheelBR.getPower();
//            double WheelFL= motorWheelFL.getPower();
//            double WheelFR= motorWheelFR.getPower();
//
//            double errorBL= c1-WheelBL;
//            double errorBR= c2-WheelBR;
//            double errorFL= c3-WheelFL;
//            double errorFR= c4-WheelFR;
//
//            double KP= 0.8;

//            boolean PID = gamepad1.left_stick_button;
//            if(PID){
//                if(timer3 > 100) {
//                    runPID = !runPID;
//                    timer3 = 0;
//                }
//            }
//            if(runPID){
                V1=c1;
                V2=c2;
                V3=c3;
                V4=c4;
//            }
//            else{
//                V1= c1 + KP*errorBL;
//                V2= c2 + KP*errorBR;
//                V3= c3 + KP*errorFL;
//                V4= c4 + KP*errorFR;
//            }



            motorWheelBL.setPower(V1);
            motorWheelBR.setPower(V2);
            motorWheelFL.setPower(V3);
            motorWheelFR.setPower(V4);


//            telemetry.addData("FL Motor Power", motorWheelFL.getPower());
//            telemetry.addData("FR Motor Power", motorWheelFR.getPower());
//            telemetry.addData("BL Motor Power", motorWheelBL.getPower());
//            telemetry.addData("BR Motor Power", motorWheelBR.getPower());
//            telemetry.addData("v1",v1);
//            telemetry.addData("c1",c1);
//            telemetry.addData("errorBL",errorBL);
//            telemetry.addData("V1",V1);


          //  telemetry.update();

            float liftDown = this.gamepad1.left_trigger;
            float liftUp = this.gamepad1.right_trigger;
            boolean tray = this.gamepad1.right_bumper;
            boolean grab = this.gamepad1.left_bumper;
            //boolean trayDown = this.gamepad1.a;
            //boolean trayUp = this.gamepad1.b;
            motorLift.setPower(liftUp-liftDown);
            if(grab) {
                //Grab me from behind
                if(getRuntime()-timer2 > 0.5) {
                    blockGrabbed = !blockGrabbed;
                    timer2 = getRuntime();
                }
            }
            if(tray){
                if(getRuntime()-timer1 > 0.5) {
                    trayDown = !trayDown;
                    timer1 = getRuntime();
                }
            }
            if(this.gamepad1.a) {
                servoPower = 0.2;
            } else if(this.gamepad1.b) {
                servoPower = -0.2;

            } else {
                servoPower = 0;
            }
            /*
            if(close){
                servoClamp.setPosition(1);
            }
            else{
                servoClamp.setPosition(0);
            }

             */
            hentai.setPower(servoPower);
            collectionWheelsL.setPower(0.1);
            collectionWheelsR.setPower(-0.1);
            servoWheelOne.setPower(0.1);
            servoWheelTwo.setPower(-0.1);
            if(trayDown){
                trayOne.setPosition(1);
                trayTwo.setPosition(1);
                //motorTray.setPower(0.1);
                //motorTray2.setPower(-0.1);
            } else {
                trayOne.setPosition(0);
                trayTwo.setPosition(0);
                //motorTray.setPower(-0.1);
                //motorTray2.setPower(0.1);
            }
            if(blockGrabbed) {
                cockAndBallTorture.setPosition(1);

            } else {
                cockAndBallTorture.setPosition(0);
            }
            /*
            else{
                motorTray.setPower(0);
                motorTray2.setPower(0);
            }

             */
        }
    }

