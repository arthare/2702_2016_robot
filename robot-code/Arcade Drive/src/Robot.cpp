#include "WPILib.h"
#include "Interfaces.h"
#include "constants.h"
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
int receivePacket();
struct data
{
	int x;
	int y;
};
/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
class Robot: public SampleRobot
{
	RobotDrive myRobot; // robot drive system
	Joystick driveStick;
	Joystick opStick;

	Solenoid highShifter;
	Solenoid lowShifter;

	Compressor compressor;

	Encoder rightencoder;
	Encoder leftencoder;

	Talon shooter;
	Talon turret;

//	DigitalInput rightencA;
//	DigitalInput rightencB;
//	DigitalInput leftencA;
//	DigitalInput leftencB;

	double angleSetpoint = 0.0;
	const double pGain = .006; //propotional turning constant
	Gyro* gyro;
	std::thread networkThread;

	//interfaces
	//SolenoidInterface* shiftersolenoid;



public:
	Robot() :
			myRobot(DRIVE_RIGHT_PWM, DRIVE_LEFT_PWM),	// initialize the RobotDrive to use motor controllers on ports 0 and 1
			driveStick(DRIVER_JOYSTICK),
			opStick(OP_JOYSTICK),
			highShifter(HIGH_SHIFT_SOLENOID),
			lowShifter(LOW_SHIFT_SOLENOID),
//			rightencA(RIGHT_ENCODER_A_DIO),
//			rightencB(RIGHT_ENCODER_B_DIO),
//			leftencA(LEFT_ENCODER_A_DIO),
//			leftencB(LEFT_ENCODER_B_DIO),
			leftencoder(LEFT_ENCODER_A_DIO, LEFT_ENCODER_B_DIO),
			rightencoder(RIGHT_ENCODER_A_DIO, RIGHT_ENCODER_B_DIO),
			compressor(COMPRESSOR),
			shooter(SHOOTER),
			turret(TURRET),
			networkThread(receivePacket)

	{
		gyro = new AnalogGyro(GYRO_ANALOG_CHANNEL);
		myRobot.SetExpiration(0.1);

		myRobot.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kRearRightMotor, true);

		compressor.Start();
	}
	void Autonomous()
	{
		int start=leftencoder.GetRaw();
		double turningValue;
		while(IsAutonomous() && IsEnabled() && leftencoder.GetRaw()<start+6000)
		{
			turningValue = (angleSetpoint - gyro->GetAngle()) * pGain;
			printf("in autonomous \n");
			Wait(0.005);
			myRobot.ArcadeDrive(-0.7, turningValue);
		}
	}
	/**
	 * Runs the motors with arcade steering.
	 */
	void OperatorControl()
	{
		int printcnt = 0;

		while (IsOperatorControl() && IsEnabled())
		{
			printf("hi i, \n");
			myRobot.ArcadeDrive(driveStick); // drive with arcade style (use right stick)
			if( driveStick.GetRawButton( 2 ))
			{
				highShifter.Set( 1 );
			}
			else
			{
				highShifter.Set( 0 );
			}
			if( driveStick.GetRawButton( 3 ))
			{
				lowShifter.Set( 1 );
			}
			else
			{
				lowShifter.Set( 0 );
			}
			Wait(0.005);// wait for a motor update time

			if( ++printcnt >1000 )
			{
				printf("%d %d\n",leftencoder.GetRaw(), rightencoder.GetRaw());
				printcnt = 0;
			}
			if(driveStick.GetRawButton(SHOOTER_BUTTON))
			{
				shooter.Set(1.0);
			}
			else
			{
				shooter.Set(0);

			}

				turret.Set(opStick.GetX());

		}
	}
	void drive2()
	{
		double turningValue;

		while (IsOperatorControl() && IsEnabled())
		{
			turningValue = (angleSetpoint - gyro->GetAngle()) * pGain;
			if (driveStick.GetY() <= 0) {
				//forwards
				myRobot.Drive(driveStick.GetY(), turningValue);
			} else {
				//backwards
				myRobot.Drive(driveStick.GetY(), -turningValue);
			}
		}
	}

};


int receivePacket()
{

    int iResult = 0;

    int RecvSocket;
    sockaddr_in RecvAddr;

    unsigned short Port = 2702;

    char RecvBuf[1024];
    int BufLen = 1024;

    sockaddr_in SenderAddr;
    socklen_t SenderAddrSize = sizeof (SenderAddr);

    //-----------------------------------------------
    // Create a receiver socket to receive datagrams
    RecvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (RecvSocket < 0) {
        return 1;
    }
    //-----------------------------------------------
    // Bind the socket to any address and the specified port.
    RecvAddr.sin_family = AF_INET;
    RecvAddr.sin_port = htons(Port);
    RecvAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    iResult = bind(RecvSocket,(const sockaddr*) & RecvAddr, sizeof (RecvAddr));
    if (iResult != 0) {
        return 1;
    }
    //-----------------------------------------------
    // Call the recvfrom function to receive datagrams
    // on the bound socket.
    while(true)
    {
    	data dataRecv;
    	iResult = recvfrom(RecvSocket,& dataRecv, sizeof(dataRecv), 0, (sockaddr*)& SenderAddr, &SenderAddrSize);
    	printf("got %d %d \n",dataRecv.x ,dataRecv.y);
    }
    //-----------------------------------------------
    // Close the socket when finished receiving datagrams
    iResult = close(RecvSocket);
    if (iResult <0) {

        return 1;
    }

    //-----------------------------------------------
    // Clean up and exit.
    return 0;
}

START_ROBOT_CLASS(Robot)

