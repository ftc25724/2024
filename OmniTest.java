
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.GyroSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Robot: Omni Tank", group="Robot")

public class OmniTest extends LinearOpMode {

    // Declare OpMode members for each of the motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontL = null;
    private DcMotor backL = null;
    private DcMotor frontR = null;
    private DcMotor backR = null;
    private DcMotor slider = null;
    private DcMotor armm = null;
    private Servo graby = null;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontL  = hardwareMap.get(DcMotor.class, "flm");
        backL  = hardwareMap.get(DcMotor.class, "blm");
        frontR = hardwareMap.get(DcMotor.class, "frm");
        backR = hardwareMap.get(DcMotor.class, "brm");
        slider = hardwareMap.get(DcMotor.class, "slide");
        armm = hardwareMap.get(DcMotor.class, "arm");
        graby = hardwareMap.get(Servo.class, "grab");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        
        int armposition = 0;
        
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        graby.setPosition(0);
        
        
        //RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        //RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        //RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        //.initialize(new IMU.Parameters(orientationOnRobot));


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontL.setDirection(DcMotor.Direction.REVERSE);
        backL.setDirection(DcMotor.Direction.REVERSE);
        frontR.setDirection(DcMotor.Direction.FORWARD);
        backR.setDirection(DcMotor.Direction.FORWARD);
        
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max = 0;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);            
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x * 1.1;
            double lateralx = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
            double axialy = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);


            double denominator = Math.max(Math.abs(axialy) + Math.abs(lateralx) + Math.abs(yaw), 1);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            /*
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            */
            
            double leftFrontPower = (axialy + lateralx + yaw) / denominator;
            double leftBackPower = (axialy - lateralx + yaw) / denominator;
            double rightFrontPower = (axialy - lateralx - yaw) / denominator;
            double rightBackPower = (axialy + lateralx - yaw) / denominator;
            
            lateralx = lateralx * 1.1;

            
            double sliderPower = gamepad2.left_stick_y;
            double armPower = gamepad2.right_stick_y;
            
            if (gamepad1.options) {
                imu.resetYaw();
            }
            
            if (gamepad2.right_trigger > 0){
                slider.setTargetPosition(-4520);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderPower = -gamepad2.right_trigger;
                armm.setTargetPosition(-10000);
                armm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPower = -gamepad2.right_trigger;
            }
            else if (gamepad2.left_trigger > 0){
                slider.setTargetPosition(0);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderPower = gamepad2.left_trigger;
                armm.setTargetPosition(-1762);
                armm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPower = gamepad2.left_trigger - 0.15;
            }
            else if (gamepad2.left_stick_y < -0.05){
                slider.setTargetPosition(-4520);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad2.left_stick_y >= 0.05){
                slider.setTargetPosition(0);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad2.right_stick_y < -0.05){
                armm.setTargetPosition(-10000);
                armm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad2.right_stick_y >= 0.05){
                armm.setTargetPosition(-1770);
                armm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else{
                armPower = 0;
                sliderPower = 0;
                
            }
            
            
            
            //1860
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(max, Math.abs(leftFrontPower));
            max = Math.max(max, Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            
            else if (gamepad2.right_bumper){
                graby.setPosition(0.42);
                
            }
            /*
            else if (gamepad2.x){
                graby.setPosition(0);
                
            }
            */
            else if (gamepad2.left_bumper){
                graby.setPosition(0.30);
                
            }


            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            // Send calculated power to wheels
            frontL.setPower(leftFrontPower);
            frontR.setPower(rightFrontPower);
            backL.setPower(leftBackPower);
            backR.setPower(rightBackPower);
            slider.setPower(sliderPower);
            armm.setPower(armPower);
            
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("slide", slider.getCurrentPosition());
            telemetry.addData("arm", armm.getCurrentPosition());
            telemetry.addData("grab", graby.getPosition());
            telemetry.addData("Axial  ", "%4.2f", axial);
            telemetry.addData("lateral  ", "%4.2f", lateral);
            telemetry.addData("yaw  ", "%4.2f", yaw);
            telemetry.addData("up/down", gamepad2.right_stick_y);
            telemetry.addData("out  ", "%4.2f", gamepad2.left_stick_y);
            telemetry.addData("rt", gamepad2.right_trigger);
            telemetry.addData("lt", gamepad2.left_trigger);
            telemetry.update();
        }
    }}
    