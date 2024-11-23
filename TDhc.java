// /*package org.firstinspires.ftc.teamcode;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.util.Range;
//  
// @TeleOp(name="Robot: Teleop Tank", group="Robot")
// 
// public class TDhc extends OpMode{
// 
//     /* Declare OpMode members. */
// /*    public DcMotor  leftDrive   = null;
//     public DcMotor  rightDrive  = null;
//     public DcMotor  armMotor  = null;
//     
//     double left = 0;
//     double right = 0;
//     double arm_position = 0;
//     double base_position = 155;
//     double dump_position = -70;
//     double bloop_position = 22;
//     /*
//      * Code to run ONCE when the driver hits INIT
//      */
// /*    @Override
//     public void init() {
//         // Define and Initialize Motors
//         leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
//         rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
//         armMotor  = hardwareMap.get(DcMotor.class, "arm_drive");
//         armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         
// 
//         // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
//         // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
//         // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//         leftDrive.setDirection(DcMotor.Direction.REVERSE);
//         rightDrive.setDirection(DcMotor.Direction.FORWARD);
// 
//         // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
//         leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// 
//         // Define and initialize ALL installed servos.
//         // arm  = hardwareMap.get(CRServo.class, "arm");
//         // rightClaw = hardwareMap.gect(Servo.class, "right_hand");
//         // armMotor.setDirection(DcMotor.Direction.REVERSE);
//         
//         armMotor.setTargetPosition((int)base_position);
//         armMotor.setPower(0.2);
//         armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         
//         // rightClaw.setPosition(MID_SERVO);
// 
//         // Send telemetry message to signify robot waiting;
//         telemetry.addData(">", "Robot Ready.  Press Play.");
//     }
// 
//     /*
//      * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
//      */
// /*    @Override
//     public void init_loop() {
//     }
// 
//     /*
//      * Code to run ONCE when the driver hits PLAY
//      */
// /*    @Override
//     public void start() {
//         left = 0;
//         right = 0;
//     }
// 
//     /*
//      * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
//      */
// /*    @Override
//     public void loop() {
//         
// 
//         // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
//         left = -gamepad1.left_stick_y;
//         right = -gamepad1.right_stick_y;
// 
//         leftDrive.setPower(left);
//         rightDrive.setPower(right);
// 
//         if (gamepad1.dpad_up){
//             leftDrive.setDirection(DcMotor.Direction.REVERSE);
//             rightDrive.setDirection(DcMotor.Direction.FORWARD);
//             leftDrive.setPower(.80);
//             rightDrive.setPower(.80);
//         }
//         
//         if (gamepad1.dpad_down){
//             leftDrive.setDirection(DcMotor.Direction.REVERSE);
//             rightDrive.setDirection(DcMotor.Direction.FORWARD);
//             leftDrive.setPower(-.80);
//             rightDrive.setPower(-.80);
//         }
//         
//         if (gamepad1.left_bumper){
//          armMotor.setTargetPosition(250);
//          armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         }
//         
//         armMotor.setPower(0.1);
//         arm_position = base_position + gamepad1.left_trigger * bloop_position + gamepad1.right_trigger * dump_position;
//         armMotor.setTargetPosition((int)(arm_position / 360 * 537.7));
//         // armMotor.setPower(arm_position);
// 
//         // Send telemetry message to signify robot running;
//         telemetry.addData("arm",  "%.2f", arm_position);
//         telemetry.addData("left",  "%.2f", left);
//         telemetry.addData("right", "%.2f", right);
//     }
// 
//     /*
//      * Code to run ONCE after the driver hits STOP
//      */
// /*    @Override
//     public void stop() {
//     }
// }*/