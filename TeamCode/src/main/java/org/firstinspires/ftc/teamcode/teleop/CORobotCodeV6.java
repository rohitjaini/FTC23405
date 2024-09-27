package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.kdrobot.DriveTrainSpeed;
import org.firstinspires.ftc.teamcode.kdrobot.KDRobot;

@TeleOp
public class CORobotCodeV6 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        KDRobot robot = new KDRobot();
        robot.init(hardwareMap, telemetry);
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor rightLinearSlide = hardwareMap.get(DcMotor.class, "RightLS");
        DcMotor leftLinearSlide = hardwareMap.get(DcMotor.class, "LeftLS");
        Servo armHingeOne = hardwareMap.get(Servo.class, "aHO");
        Servo armHingeTwo = hardwareMap.get(Servo.class, "aHT");
        Servo clawOne = hardwareMap.get(Servo.class, "cO");
        Servo clawTwo = hardwareMap.get(Servo.class, "cT");
        DcMotor armBase = hardwareMap.get(DcMotor.class, "aB");
        armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_x; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_y * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
//Linear Slide movement
            if (gamepad2.x) {
                rightLinearSlide.setPower(1);
                leftLinearSlide.setPower(1);
                if (gamepad2.b) {
                    rightLinearSlide.setTargetPosition(1);
                    leftLinearSlide.setTargetPosition(1);
                    if (gamepad2.a) {
                        rightLinearSlide.setTargetPosition(0);
                        leftLinearSlide.setTargetPosition(0);
                        // Linear Slide movement End

                        if (gamepad1.right_bumper) {
                            sleep(250);
                            robot.changeDriveTrainSpeed(DriveTrainSpeed.INCREASE);
                        }
                        if (gamepad1.left_bumper) {
                            sleep(250);
                            robot.changeDriveTrainSpeed(DriveTrainSpeed.DECREASE);
                if (gamepad1.y) {
                    armBase.setTargetPosition(0);
                    if (gamepad1.a) {
                        armBase.setTargetPosition(-1);

                        if (gamepad1.left_trigger == 1) {
                            clawOne.setPosition(1);
                            clawTwo.setPosition(1);
                            if (gamepad1.right_trigger == 1) {
                                clawOne.setPosition(0);
                                clawTwo.setPosition(0);

                                if (gamepad1.x) {
                                    armHingeOne.setPosition(-0.5);
                                    armHingeTwo.setPosition(-0.5);
                                    if (gamepad1.a) {
                                        armHingeOne.setPosition(1);
                                        armHingeTwo.setPosition(1);
                                        if (gamepad1.y) {
                                            armBase.setTargetPosition(0);
                                        }
                                    }
                                }
                            }
                            }
                    }
                }









                        }
                    }
                }
            }
        }
    }
}

