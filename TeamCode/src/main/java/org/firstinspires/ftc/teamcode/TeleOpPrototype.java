package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name = "Nothing Racist", group = "Competition")
public class TeleOpPrototype extends LinearOpMode {
    public static int wormTargetPos = 0;
    private PIDController wormController = new PIDController(0.013, 0, 0);
    private DcMotor LF, RF, LB, RB;
    private DcMotor Slide, Worm;
    private Servo wrist;
    private Servo claw;
    private double speed;
    private double auxSpeed;
    public ElapsedTime elapsedTime = new ElapsedTime();
    public String wormGoTo;
    @Override
    public void runOpMode() {
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        Slide = hardwareMap.get(DcMotor.class, "wormGearMotor");
        Worm = hardwareMap.get(DcMotor.class, "LSlide");
        //WORM IS RIGHT STICK - Y || SLIDE IS LEFT STICK - Y
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        //wrist = hardwareMap.get(Servo.class,"wrist");
        //wrist.setPosition(0);
        //claw = hardwareMap.get(Servo.class,"claw");
        //claw.setPosition(0);
        //drive = new MecanumDrive_Striker(hardwareMap);
        speed = 0.4;
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int slideCurrent = Slide.getCurrentPosition();
        int wormCurrent = Worm.getCurrentPosition();
        final int wormStartPosition = wormCurrent;
        final int slideStartPosition = slideCurrent;
        //int auxmotoCurrent = auxMotor.getCurrentPosition();
        //auxMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //auxMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        /*
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        */
        double tester = 0.0;
        boolean wristDirection = true;
        double Drive = 0.0;
        double strafe = 0.0;
        double rotate = 0.0;
        //RUNWITHOUTENCODER means THE ENCODER STILL WORKS it's just there is more velocity control
        Gamepad currentDriverOneGamepad = new Gamepad();
        Gamepad previousDriverOneGamepad = new Gamepad();
        Gamepad currentDriverTwoGamepad = new Gamepad();
        Gamepad previousDriverTwoGamepad = new Gamepad();
        waitForStart();
        Worm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //elapsedTime.reset();
        //============================================================================================================================================================
        //=============================================================================================================================================================
        //=============================================================================================================================================================
        //=============================================================================================================================================================
        //=============================================================================================================================================================
        //=============================================================================================================================================================
        //==============================================================================================================================================================
        //==============================================================================================================================================================
        //==============================================================================================================================================================
        //==============================================================================================================================================================
        //Basic Driving (Some Motor Designations)
        //==============================================================================================================================================================
        //==============================================================================================================================================================
        wrist.setPosition(0.5);
        while (opModeIsActive()) {
            gamepad1.setLedColor(0.0, 0.0, 100.0, 1000000000);
            gamepad2.setLedColor(0.0, 0.0, 100.0, 1000000000);
            slideCurrent = Slide.getCurrentPosition();
            wormCurrent = Worm.getCurrentPosition();
            Drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x * 1.1;
            rotate = gamepad1.right_stick_x;
            try {
                previousDriverOneGamepad.copy(currentDriverOneGamepad);
                currentDriverOneGamepad.copy(gamepad1);
                previousDriverTwoGamepad.copy(currentDriverTwoGamepad);
                currentDriverTwoGamepad.copy(gamepad2);
            } catch (Error ignored) {}
            double max;
            double axial = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double leftFrontPower = axial - lateral + yaw;
            double leftBackPower = axial + lateral + yaw;
            //-
            double rightFrontPower = axial - lateral - yaw;
            double rightBackPower = axial + lateral - yaw;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            LF.setPower(leftFrontPower * speed);
            RF.setPower(rightFrontPower * speed);
            LB.setPower(leftBackPower * speed);
            RB.setPower(-rightBackPower * speed);
            //============================================================================================================================================================
            //=============================================================================================================================================================
            //=============================================================================================================================================================
            //=============================================================================================================================================================
            //=============================================================================================================================================================
            //=============================================================================================================================================================
            //==============================================================================================================================================================
            //==============================================================================================================================================================
            //==============================================================================================================================================================
            //==============================================================================================================================================================
            //Misc. Controls
            //==============================================================================================================================================================
            //==============================================================================================================================================================
            if (gamepad1.right_trigger > 0) {
                speed = 0.9 * gamepad1.right_trigger;
            } else {
                speed = 0.4;
            }
            //============================================================================================================================================================
            //=============================================================================================================================================================
            //=============================================================================================================================================================
            //=============================================================================================================================================================
            //=============================================================================================================================================================
            //=============================================================================================================================================================
            //==============================================================================================================================================================
            //==============================================================================================================================================================
            //==============================================================================================================================================================
            //==============================================================================================================================================================
            //Aux. Controls
            //==============================================================================================================================================================
            //==============================================================================================================================================================
            //wristDirection = true; == clockwise
            //wristDirection = false; == counterclockwise
            if (currentDriverTwoGamepad.circle && !previousDriverTwoGamepad.circle) {
                if (claw.getPosition() == 0) {
                    claw.setPosition(0.4);
                } else {
                    claw.setPosition(0);
                }
            }
            //0.4 = | \
            //0.8 = scoop
            //0.2 = ???ty
            /*
            if gamepad
            */
            //change here if too fast or slow
            //Slide.setTargetPosition();
            //sets to sepcific sposition (also need power and run to position)
            /*
            if Slide.getCurrentPosition(too far +/-)
            {
            Slide.setTargetPosition(far enough);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(medium speed);
            }
            else
            {
            */
            if (gamepad2.right_trigger > 0) {
                auxSpeed = 0.9;
            } else {
                auxSpeed = 0.4;
            }
            if (currentDriverTwoGamepad.dpad_down && !previousDriverTwoGamepad.dpad_down) {
                wormTargetPos = 0;
            } else if (currentDriverTwoGamepad.dpad_up && !previousDriverTwoGamepad.dpad_up) {
                wormTargetPos = 2675;
            } else if (currentDriverTwoGamepad.dpad_left && !previousDriverTwoGamepad.dpad_left) {
                wormTargetPos = 3000;
            } else if (currentDriverTwoGamepad.dpad_right && !previousDriverTwoGamepad.dpad_right) {
                wormTargetPos = 3400;
            }

            if (gamepad2.right_stick_y != 0) {
                Worm.setPower(gamepad2.right_stick_y * auxSpeed);
            } else {
                Worm.setPower(wormController.calculate(Worm.getCurrentPosition(), wormTargetPos));
            }

            //else {
            // Worm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //}
            Slide.setPower(gamepad2.left_stick_y * auxSpeed);
            telemetry.addData("GAMEPAD 2 . RIGHT STICK Y : ", gamepad2.right_stick_y);
            telemetry.addData("LB : ", LB.getPower());
            telemetry.addData("RF : ", RF.getPower());
            telemetry.addData("RB : ", RB.getPower());
            telemetry.addData("DRIVE", Drive);
            telemetry.addData("STRAFE", strafe);
            telemetry.addData("ROTATE", rotate);
            telemetry.addData("ROTATE", Worm.getCurrentPosition());
            telemetry.addData("ROTATE TARGET", Worm.getTargetPosition());
            telemetry.addData("SLIDE CURRENT", Slide.getCurrentPosition());
            telemetry.addData("SLIDE TARGET", Slide.getTargetPosition());
            telemetry.addData("CLAW", claw.getPosition());
            telemetry.addData("WRIST", wrist.getPosition());
            telemetry.update();
        }
    }
}
//============================================================================================================================================================
//=============================================================================================================================================================
//=============================================================================================================================================================
//=============================================================================================================================================================
//=============================================================================================================================================================
//=============================================================================================================================================================