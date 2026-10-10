/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Vy Nguyen                                        */
/*    Created:      Tue Sep 27 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// MotorFL              motor         16              
// MotorFR              motor         15              
// MotorBL              motor         12              
// MotorBR              motor         14              
// MotorML              motor         11              
// MotorMR              motor         13              
// IntakeMotor          motor         6               
// Controller1          controller                    
// Catapult             motor         1               
// String               digital_out   H               
// Optical              optical       10              
// LimitSwitchB         limit         B               
// RotationSensor       rotation      17              
// InertialSensor       inertial      18              
// BandBooster          digital_out   A               
// String2              digital_out   F               
// LimitSwitchAuto      limit         C               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include <string>
#include <sstream>
#include <list>

using namespace vex;\
competition Competition;

double Auto;

int AutoCode = 0;
double AutoCodeTime[100];
double AutoCodeError[100];
double StartingPos;
double InitialTime;


double cmove;
double TrackingCircum = 8.63938;
double totalmoved;

double chead;
double MotorValues;
double shead;
double constant = 2;

bool Skills;

  int filenumber = 1;
  char filename[100];
  int brainclear = 0;

void MotorStop()
{
  MotorBL.spin(fwd,0,pct);
  MotorML.spin(fwd,0,pct);
  MotorFL.spin(fwd,0,pct);
  MotorBR.spin(fwd,0,pct);
  MotorMR.spin(fwd,0,pct);
  MotorFR.spin(fwd,0,pct);

  MotorBL.setBrake(hold);
  MotorML.setBrake(hold);
  MotorFL.setBrake(hold);
  MotorBR.setBrake(hold);
  MotorMR.setBrake(hold);
  MotorFR.setBrake(hold);
}

void AllMotorSpin(double Lmotor, double Rmotor)
{
    MotorBL.spin(fwd,Lmotor,pct);
    MotorML.spin(fwd,Lmotor,pct);
    MotorFL.spin(fwd,Lmotor,pct);
    MotorBR.spin(fwd,Rmotor,pct);
    MotorMR.spin(fwd,Rmotor,pct);
    MotorFR.spin(fwd,Rmotor,pct);
}


double smove;
double DirectionFix;
double TotalTurn = 0;
double LeftError;
double RightError;

double TimeList[300];
double ManipulatedList[300];

void MoveForwards(double inches, bool forwards,int speed)
  {
    RotationSensor.resetPosition();
    smove = RotationSensor.position(rev) * TrackingCircum;
    DirectionFix = 1;
    MotorValues = speed;
    LeftError = 0;
    RightError = 0;
    if (forwards == true)
      {
        DirectionFix = 1;
      }
        else
          {
            DirectionFix = -1;
          }   
     /* if (inches > 30)
        {
          MotorValues = 10;
          while (((RotationSensor.position(rev) - (smove*DirectionFix)) * TrackingCircum) <  (6*DirectionFix))
            {
              MotorBL.spin(fwd,MotorValues + LeftError,pct);
              MotorML.spin(fwd,MotorValues + LeftError,pct);
              MotorFL.spin(fwd,MotorValues + LeftError,pct);
              MotorBR.spin(fwd,MotorValues + RightError,pct);
              MotorMR.spin(fwd,MotorValues + RightError,pct);
              MotorFR.spin(fwd,MotorValues + RightError,pct);
              MotorValues = (((((RotationSensor.position(rev)*TrackingCircum) -(smove * DirectionFix) )/6 )*40)+10);
              printf("Motor Value %f\n",MotorValues);
              //LeftError = 0.1*(TotalTurn - InertialSensor.rotation());
              //RightError = -0.1*(TotalTurn - InertialSensor.rotation());
              printf("Right Error %f\n",RightError);
              printf("Left Error %f\n",LeftError);
              if(RightError > 0.25)
                {
                  RightError = 0.25;
                  LeftError = -0.25;
                }
            }
        } */

    while((inches > (smove - (RotationSensor.position(rev) * TrackingCircum))  ) and ((MotorValues > (constant+0.3) )) )
      {
        //double a = -0.00020289; //v1
        //double c = 2.44289;     // 

        //double a = -0.000212076;  //v2
        //double c = 2.52331;       //25/50 -> 25/60

        //double a = -0.00021699;   //v3
        //double c = 2.56587;       //12.5/10 -> 12.5/20

        //double a = -0.000221904; //v4
        //double c = 2.60843; //12.5/20 -> 12.5/30

        double a = -0.000240918; //v5
        double c = 2.77397; //12.5/30 -> 12.5/50 ||| 25/60 -> 25/70

        MotorValues = ( speed * ( 1 - ( ((RotationSensor.position(rev)*TrackingCircum * DirectionFix) - smove ) / (inches) ) ) ) ;
        MotorValues = (a*pow(MotorValues,3.0) + c * MotorValues) + (constant * DirectionFix);
        //MotorValues = 1.1 * MotorValues;
        //LeftError = -0.5*(TotalTurn - InertialSensor.rotation()) * DirectionFix;
        //RightError = 0.5*(TotalTurn - InertialSensor.rotation()) * DirectionFix;
        if(RightError > 0.25)
          {
            RightError = 0.25;
            LeftError = -0.25;
          }
              //printf("Right Error %f\n",RightError);
              //printf("Left Error %f\n",LeftError);
      //  printf("Motor Value: %f\n",MotorValues);
    MotorValues = MotorValues * DirectionFix;
    MotorBL.spin(fwd,MotorValues + LeftError,pct);
    MotorML.spin(fwd,MotorValues + LeftError,pct);
    MotorFL.spin(fwd,MotorValues + LeftError,pct);
    MotorBR.spin(fwd,MotorValues + RightError,pct);
    MotorMR.spin(fwd,MotorValues + RightError,pct);
    MotorFR.spin(fwd,MotorValues + RightError,pct);
        MotorValues = MotorValues * DirectionFix;
        if (LimitSwitchB.pressing())
          {
            Catapult.spin(fwd,0,pct);
            Catapult.setBrake(hold);
          }
      }
      MotorStop();
      //printf("Heading change %f\n",InertialSensor.heading());
  }

void Turn(int TurnValue,int speed,bool left)
  {
//    InertialSensor.setHeading(180,deg);
    TotalTurn = TotalTurn + TurnValue;
    MotorValues = 1;
    //printf("shead: %f\n",shead);

for(int i = 0; i < 5;i++)
  {


    while( !(TotalTurn == round(InertialSensor.rotation(deg) ) ) and !(round(MotorValues) == 0) )
      {
        MotorValues = ( speed * ( 1 - ( ( InertialSensor.rotation(deg) - (TotalTurn-TurnValue)) / TurnValue ) ) );
    MotorValues = round(MotorValues);
    //printf("Velocity %f\n", MotorValues);
    //printf("Inertial %f\n",std::abs(InertialSensor.rotation(deg)));
    //printf("Total Turn %f\n",TotalTurn);
    if(left)
      {
    MotorBL.spin(fwd,MotorValues,pct);
    MotorML.spin(fwd,MotorValues,pct);
    MotorFL.spin(fwd,MotorValues,pct);
    MotorBR.spin(fwd,MotorValues*-1,pct);
    MotorMR.spin(fwd,MotorValues*-1,pct);
    MotorFR.spin(fwd,MotorValues*-1,pct);
      }
        else
          {
    MotorBL.spin(reverse,MotorValues,pct);
    MotorML.spin(reverse,MotorValues,pct);
    MotorFL.spin(reverse,MotorValues,pct);
    MotorBR.spin(reverse,MotorValues*-1,pct);
    MotorMR.spin(reverse,MotorValues*-1,pct);
    MotorFR.spin(reverse,MotorValues*-1,pct);
          }

              if (LimitSwitchB.pressing())
                {
                  Catapult.spin(fwd,0,pct);
                  Catapult.setBrake(hold);
                }

      }
      MotorStop();
      wait(0.1,sec);
  }
  }

void Coasting()
  {
    MotorFL.setBrake(coast);
    MotorML.setBrake(coast);
    MotorBL.setBrake(coast);
    MotorFR.setBrake(coast);
    MotorMR.setBrake(coast);
    MotorBR.setBrake(coast);
  }

void ControllerPrint()
  {
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.clearScreen();
  }


void Driving() //Driver control, puts onto a cubic function
  {
      double joystickL = ((0.0000530602 * pow(Controller1.Axis3.position(),3.0)) + 0.469292*Controller1.Axis3.position()) + ( (0.0000530602 * pow(Controller1.Axis1.position(),3.0)) + 0.469292*Controller1.Axis1.position());
      double joystickR = ((0.0000530602 * pow(Controller1.Axis3.position(),3.0)) + 0.469292*Controller1.Axis3.position()) - ( (0.0000530602 * pow(Controller1.Axis1.position(),3.0)) + 0.469292*Controller1.Axis1.position());
      //printf("Left %f\n",joystickL);
      //printf("Right %f\n",joystickR);
      MotorFR.spin(fwd,joystickR, pct); //moves all the motors forwards
      MotorFL.spin(fwd,joystickL, pct);
      MotorMR.spin(fwd,joystickR, pct);
      MotorML.spin(fwd,joystickL, pct);
      MotorBR.spin(fwd,joystickR, pct);
      MotorBL.spin(fwd,joystickL, pct);
  }


void Rollerspin(int colour, int intakespeed) //spins roller during Autonomous using optical sensor
  {
    waitUntil(Optical.isNearObject());
    wait(0.25,sec);
    if (colour == 1) //1 for red
      {
        while(!(Optical.hue() < 90 or Controller1.ButtonA.pressing()))
          {
            //intakespeed = intakespeed + 0.01;
            IntakeMotor.spin(fwd,intakespeed,pct);
          }
        IntakeMotor.spin(fwd,0,pct);
      }
        else if (colour == 0) //0 for blue
        
          {
            while(!(Optical.hue() > 90 or Controller1.ButtonA.pressing()))
              {
                //intakespeed = intakespeed + 0.01;
                IntakeMotor.spin(fwd,intakespeed,pct);
              }
            IntakeMotor.spin(fwd,0,pct);
          }
  }
double stime;
int rumbling;
  void usercontrol(void)
{
  Coasting(); //makes sure all the motors are coasting and not holding during driver
  BandBooster.set(true);//turns off the band booster
  int intakespeed = 100;
 // int temptimer = 0;
  Optical.setLight(ledState::on);
  Optical.setLightPower(100,pct);
  rumbling = 0;
  stime = Brain.timer(sec);
  double BandBoosterTimer = Brain.timer(sec);


  while(true){ //forever loop for drive train

      Driving(); //drive train
      if (Skills) //during skills, the controller will rumble at important times
        {
          if(stime == 0)
            {
              stime = Brain.timer(sec);
            }
              else if (stime + 15 > Brain.timer(sec) and rumbling == 0)
                {
                  Controller1.rumble("-");
                  rumbling++;
                }
                  else if (stime + 30 > Brain.timer(sec) and rumbling == 1)
                    {
                      Controller1.rumble("--");
                      rumbling++;
                    }
                      else if (stime + 45 > Brain.timer(sec) and rumbling == 2)
                        {
                          Controller1.rumble("-");
                          rumbling++;
                        }
                          else if (stime + 55 > Brain.timer(sec) and rumbling == 3)
                            {
                              Controller1.rumble(".");
                              rumbling++;
                            }
                              else if (stime + 56 > Brain.timer(sec) and rumbling == 4)
                                {
                                  Controller1.rumble(".");
                                  rumbling++;
                                }
                                  else if (stime + 57 > Brain.timer(sec) and rumbling == 5)
                                    {
                                      Controller1.rumble(".");
                                      rumbling++;
                                    }
                                      else if (stime + 58 > Brain.timer(sec) and rumbling == 6)
                                        {
                                          Controller1.rumble(".");
                                          rumbling++;
                                        }
                                          else if (stime + 59 > Brain.timer(sec) and rumbling == 7)
                                            {
                                              Controller1.rumble(".");
                                              rumbling++;
                                            }
                                              else if (stime + 60 > Brain.timer(sec) and rumbling == 7)
                                                {
                                                  Controller1.rumble("---");
                                                  rumbling++;
                                                }
        }
if(Controller1.ButtonUp.pressing())
  {
    BandBooster.set(true); //incase the band booster is somehow off or doesnt work, you can manualy remove the bands
  }
    else
      {
        BandBooster.set(false);
      }
    
    if(Controller1.ButtonB.pressing() and Controller1.ButtonDown.pressing())
      {
       String2.set(true); 
      }

 
    if(Controller1.ButtonRight.pressing() and Controller1.ButtonY.pressing()) //makes sure that you have to press 2 buttons at the same time to prevent early expansion
      {
        String.set(true); //shoots expansion
      }

    if(Controller1.ButtonL2.pressing()) //when button l2 pressed spin intake
      {
        IntakeMotor.spin(fwd,intakespeed,pct);
      }
        else if(Controller1.ButtonL1.pressing()) //when button l1 pressed stop intake
          {
            IntakeMotor.spin(fwd,-intakespeed,pct);
          }
            else if(!Controller1.ButtonY.pressing())
              {
                IntakeMotor.spin(fwd,0,pct);
              }

    if(Controller1.ButtonUp.pressing()) //increases and decreases intake speed
      {
        intakespeed = 100;
      }
        else if(Controller1.ButtonDown.pressing())
          {
            intakespeed = 60;
          }
    if(Controller1.ButtonB.pressing()) //Catapult band booster disengage
      {
        BandBooster.set(false);
      }

    if(Controller1.ButtonR2.pressing() or Controller1.ButtonA.pressing()) //Catapult firing
      {
        Catapult.spin(fwd,80,pct);
      }
    if(!LimitSwitchB.pressing()) //pulls catapult back until detector detects the arm
      {
        Catapult.spin(fwd,80,pct);
      }
        else if (!(Controller1.ButtonA.pressing() or Controller1.ButtonR2.pressing()))
          {
            Catapult.spin(fwd,0,pct);
            Catapult.setBrake(hold);
          }



    if ((BandBoosterTimer + 2) > Brain.timer(sec))
      {
        BandBooster.set(false);
      }

      /*if(pulton == 1) //wait time code for future reference
      {
        if(temptimer == 0)
          {
           temptimer = Brain.timer(sec);
          }
            else if (temptimer <= (Brain.timer(sec) - 1.8))
              { /
                Catapult.spin(fwd,0,pct);
                Catapult.setBrake(brake);
                temptimer = 0;
                pulton = 0;
              }
      }*/

  }
    }
double test;

void CataFire() //fires the catapult
  {
    Catapult.spin(fwd,80,pct);
    waitUntil(LimitSwitchB.pressing());
    wait(0.5,sec);
    waitUntil(LimitSwitchB.pressing());
    Catapult.spin(fwd,0,pct);
    Catapult.setBrake(hold);
  }

double waittime = 0.5;
void autonomous(void)
{
  Optical.setLight(ledState::on);
  Optical.brightness(100);
  TotalTurn = 0;


if(Skills)
  {

if (Auto == 1) //skills prog autonomous
  {

            RotationSensor.setPosition(0,rev);
            InertialSensor.setRotation(0, deg);
            waittime = 0;
            int rollerspeed = 100;

            IntakeMotor.spin(fwd,50,pct);

              ControllerPrint();
              Controller1.Screen.print("Catapult prep");
            Catapult.spin(fwd,80,pct);
            waitUntil(LimitSwitchB.pressing());
            Catapult.spin(fwd,0,pct);
            Catapult.setBrake(hold);

              ControllerPrint();
              Controller1.Screen.print("Spinning 1st Roller");
            AllMotorSpin(10, 10);
            Rollerspin(0,rollerspeed); //spins untill it detects blue
              ControllerPrint();
              Controller1.Screen.print("1st Roller Spun");
            MotorStop();
            wait(waittime,sec);

              ControllerPrint();
              Controller1.Screen.print("Moving away from 1st Roller");
            MoveForwards(16, false, 30);
              ControllerPrint();
              Controller1.Screen.print("Turning towards 2nd Roller");
            wait(waittime,sec);
            Turn(88,30,true); //
              ControllerPrint();
              Controller1.Screen.print("Moving towards 2nd Roller");
            IntakeMotor.spin(fwd,100,pct);
            MoveForwards(13, true, 10);
              ControllerPrint();
              Controller1.Screen.print("2nd Roller Spinning");
            AllMotorSpin(10, 10);
            //intakespeed = 100;
            Rollerspin(0,rollerspeed);
              ControllerPrint();
              Controller1.Screen.print("2nd Roller Spun");
            MotorStop();
            wait(waittime,sec);
              ControllerPrint();
              Controller1.Screen.print("Move Away from 2nd Roller");
            MoveForwards(6, false, 20);
            IntakeMotor.spin(fwd,0,pct);
            wait(waittime,sec);
            wait(0.25,sec);
              ControllerPrint();
              Controller1.Screen.print("Turning towards Red high Goal");
            Turn(-90, -40, true);
            wait(waittime,sec);
              ControllerPrint();
              Controller1.Screen.print("Moving towards Red high Goal");
            MoveForwards(50, false, 45);//1.5
            MotorStop();
            wait(waittime,sec);
            wait(waittime,sec);
            Turn(9,8,true);
              ControllerPrint();
              Controller1.Screen.print("Shooting into Red high Goal");
            CataFire();
            wait(waittime,sec);
            Turn(-16,-8,true);
            wait(waittime,sec);
              ControllerPrint();
              Controller1.Screen.print("Moving backwards from Red high goal");
            MoveForwards(30, true, 25);
              ControllerPrint();
              Controller1.Screen.print("Turning towards 3 discs");
            Turn(-128,-65,true);
              ControllerPrint();
              Controller1.Screen.print("Intaking 3 discs");
            IntakeMotor.spin(fwd,100,pct);
            MoveForwards(55, true, 30);
            IntakeMotor.spin(fwd,-100,pct);
            wait(0.5,sec);
            IntakeMotor.spin(fwd,100,pct);
              ControllerPrint();
              Controller1.Screen.print("Turning to move towards Red high goal");
            Turn(-46,-30,true);
              ControllerPrint();
              Controller1.Screen.print("Moving to line up with Red high goal");
            MoveForwards(32, true, 25);
              ControllerPrint();
              Controller1.Screen.print("Turning to line up with Red high goal");
            Turn(90,30,true);
             //ControllerPrint();
            //  Controller1.Screen.print("Creeping towards Red high goal");
           // MoveForwards(10, false, 8);
            //Turn(-5,-5,true);
              ControllerPrint();
              Controller1.Screen.print("Shooting Cata");
            CataFire();
              ControllerPrint();
              Controller1.Screen.print("Moving towards Roller 3");
            Turn(8,5,true);
            MoveForwards(42, true, 26);
              ControllerPrint();
              Controller1.Screen.print("Turning towards Roller 3");
            Turn(-100,-45,true);
              ControllerPrint();
              Controller1.Screen.print("Spinning Roller 3");
            AllMotorSpin(10, 10);
            IntakeMotor.spin(fwd,100,pct);
            Rollerspin(0, rollerspeed);
            IntakeMotor.spin(fwd,0,pct);
              ControllerPrint();
              Controller1.Screen.print("Moving away from Roller 3");
            MoveForwards(20, false, 25);
              ControllerPrint();
              Controller1.Screen.print("Turning towards Roller 4");
            IntakeMotor.spin(fwd,100,pct);
            Turn(97,35,true);
            MoveForwards(6, true, 20);
              ControllerPrint();
              Controller1.Screen.print("Spinning Roller 4");
            AllMotorSpin(10, 10);
            IntakeMotor.spin(fwd,100,pct);
            Rollerspin(0, rollerspeed);
              ControllerPrint();
              Controller1.Screen.print("Moving away Roller 4");
            MoveForwards(5,false, 8);
              ControllerPrint();
              Controller1.Screen.print("Linning up for Expansion");
            Turn(-90,-35,true);
            MoveForwards(16.5, true, 30);
            Turn(45,20,true);
            MoveForwards(3, false, 5);
              ControllerPrint();
              Controller1.Screen.print("Firing Expansion");
            String.set(true);
            String2.set(true);

            



  }
    else if (Auto == 2) //Skills Safe Autonomous, incase the main prog doesnt work as intended or is not finished, this will ensure a few points
      {
        RotationSensor.setPosition(0,rev);
        InertialSensor.setRotation(0, deg);

        IntakeMotor.spin(fwd,100,pct);

          ControllerPrint();
          Controller1.Screen.print("Catapult prep");
        Catapult.spin(fwd,80,pct);
        waitUntil(LimitSwitchB.pressing());
        Catapult.spin(fwd,0,pct);
        Catapult.setBrake(hold);

          ControllerPrint();
          Controller1.Screen.print("Spinning 1st Roller");
        AllMotorSpin(5, 5);
        wait(1,sec);
        Rollerspin(0,100); //spins untill it detects blue
          ControllerPrint();
          Controller1.Screen.print("1st Roller Spun");
        MotorStop();
        wait(waittime,sec);

          ControllerPrint();
          Controller1.Screen.print("Moving away from 1st Roller");
        MoveForwards(16.5, false, 20);
          ControllerPrint();
          Controller1.Screen.print("Turning towards 2nd Roller");
        wait(waittime,sec);
        Turn(90,25,true); //
          ControllerPrint();
          Controller1.Screen.print("Moving towards 2nd Roller");
          ControllerPrint();
          Controller1.Screen.print("2nd Roller Spinning");
        AllMotorSpin(10, 10);
        IntakeMotor.spin(fwd,100,pct);
      //intakespeed = 100;
        Rollerspin(0,100);
          ControllerPrint();
          Controller1.Screen.print("2nd Roller Spun");
        MotorStop();
        wait(waittime,sec);
          ControllerPrint();
          Controller1.Screen.print("Move Away from 2nd Roller");
        MoveForwards(8, false, 20);
        wait(waittime,sec);
          ControllerPrint();
          Controller1.Screen.print("Turning towards Red high Goal");
        Turn(-90, -25, true);
        wait(waittime,sec);
          ControllerPrint();
          Controller1.Screen.print("Moving towards Red high Goal");
        MoveForwards(57, false, 35);
        MotorStop();
        wait(waittime,sec);
        Turn(7,10,true);
        wait(waittime,sec);
          ControllerPrint();
          Controller1.Screen.print("Shooting into Red high Goal");
        CataFire();
        wait(waittime,sec);
        Turn(348,50,true);
        wait(waittime,sec);
          ControllerPrint();
          Controller1.Screen.print("Moving Back towards 1st Roller");
        MoveForwards(69, true, 35);
          ControllerPrint();
          Controller1.Screen.print("Turning Towards Middle");
        Turn(40,25,true);
          ControllerPrint();
          Controller1.Screen.print("Shooting Expansion");
        wait(waittime,sec);
        MoveForwards(10, false, 10);
        String.set(true);

      }
  }
  else
    {
        if (Auto == 1) //Red Auto 1
    { 
      AllMotorSpin(10, 10);
      Rollerspin(0,-80);
      MotorStop();
      wait(0.25,sec);

      MoveForwards(1, false, 25);

      wait(0.25,sec);
      Turn(90,25,true);
      wait(0.25,sec);
      MoveForwards(12, false, 10);
      wait(0.25,sec);
      CataFire();
      Coasting(); 

    }
      else if (Auto == 2) //Red Auto 2
        { 
          MoveForwards(12, true, 10);

          CataFire();

          MoveForwards(17, true, 10);

          wait(0.25,sec);
          Turn(90,25,true);
          wait(0.25,sec);
          
          AllMotorSpin(10, 10);
          Rollerspin(0, -80);
          MotorStop();
          Coasting();
        }
          else if(Auto == 3) //Red Auto 3
            {
              waittime = 0.25;
              //double msectime = 10;
              Catapult.spin(fwd,50,pct);
              waitUntil(LimitSwitchB.pressing());
              Catapult.spin(fwd,0,pct);
              AllMotorSpin(10, 10);
              Rollerspin(0,-80);
              MotorStop();
              wait(waittime,sec); //spins first roller

              MoveForwards(8, false, 10);//moves away from roller
              double Aiming = -7;
              Turn(Aiming,-3,true); //aims towards the high goal
              wait(waittime,sec);
              MoveForwards(0.5, true, 2);
              CataFire();
              Catapult.spin(fwd,80,pct);
              IntakeMotor.spin(fwd,100,pct);
              Turn(-120,-60,true);
              MoveForwards(42, true, 25);
              IntakeMotor.spin(reverse,100,pct);
              wait(0.5,sec);
              IntakeMotor.spin(fwd,100,pct);
              Turn(90,40,true);
              IntakeMotor.spin(reverse,100,pct);
              MoveForwards(7, false, 10);
              CataFire(); //shoots into high goal
              /*wait(msectime,msec);
              IntakeMotor.spin(reverse,100,pct);
              Turn(-132-Aiming,-60,true); //turns towards 3 disc stack
              wait(msectime,msec);
              MoveForwards(30, true, 40);
              IntakeMotor.spin(fwd,100,pct);
              MoveForwards(20, true, 20); //intakes 3 discs
              wait(msectime,msec);
              Turn(90,40,true); //turns towards high goal
              wait(waittime,sec);
              CataFire(); //shoots into high goal */
            }
              else if (Auto == 4) //Red Auto 4, long match point
                { 
                  MoveForwards(22,true, 35); //move towards roller
                  Turn(91,45,true); //turn towards roller
                  IntakeMotor.spin(reverse,100,pct);
                  AllMotorSpin(25, 25);
                  Rollerspin(0, 80); //spin roller
                  MoveForwards(7, false, 15); //move away from roller and closer high goal
                  Turn(10,5,true); //turn towards high goal
                  Catapult.spin(fwd,700,pct);//catapult shoots
                  wait(0.25,sec);
                  IntakeMotor.spin(fwd,100,pct);
                  Turn(120,56,true); //turns towards 3 discs
                  MoveForwards(58, true, 50);//intakes 3 discs
                  Turn(-85,-40,true); //turns towards high goal
                  IntakeMotor.spin(fwd,0,pct);
                  MoveForwards(8, false, 10); //moves towards high goal
                  CataFire(); //shoots into high goal
                }            
  else if (Auto == 5) //Blue Auto 1
    {
      AllMotorSpin(10, 10);
      Rollerspin(1,-50);
      MotorStop();
      wait(0.25,sec);

      MoveForwards(1, false, 25);

      wait(0.25,sec);
      Turn(90,25,true);
      wait(0.25,sec);
      MoveForwards(12, false, 10);
      wait(0.25,sec);
      CataFire();
      Coasting(); 


    }
      else if (Auto == 6) //Blue Auto 2
        {
          MoveForwards(12, true, 10);

          CataFire();

          MoveForwards(17, true, 10);

          wait(0.25,sec);
          Turn(90,25,true);
          wait(0.25,sec);
          
          AllMotorSpin(10, 10);
          Rollerspin(1, -50);
          MotorStop();
          Coasting();
        }
          else if (Auto == 7) //Blue Win point Auto 3
            {
              waittime = 0.25;
              //double msectime = 10;
              Catapult.spin(fwd,50,pct);
              waitUntil(LimitSwitchB.pressing());
              Catapult.spin(fwd,0,pct);
              AllMotorSpin(10, 10);
              Rollerspin(1,-80);
              MotorStop();
              wait(waittime,sec); //spins first roller

              MoveForwards(8, false, 10);//moves away from roller
              double Aiming = -7;
              Turn(Aiming,-3,true); //aims towards the high goal
              wait(waittime,sec);
              MoveForwards(0.5, true, 2);
              CataFire();
              Catapult.spin(fwd,80,pct);
              IntakeMotor.spin(fwd,100,pct);
              Turn(-120,-60,true);
              MoveForwards(42, true, 25);
              IntakeMotor.spin(reverse,100,pct);
              wait(0.5,sec);
              IntakeMotor.spin(fwd,100,pct);
              Turn(90,40,true);
              IntakeMotor.spin(reverse,100,pct);
              MoveForwards(7, false, 10);
              CataFire(); //shoots into high goal
             
            }
              else if (Auto == 8) // Test auto for PID or other testing
                { 
                  MoveForwards(22,true, 35); //move towards roller
                  Turn(91,45,true); //turn towards roller
                  IntakeMotor.spin(reverse,100,pct);
                  AllMotorSpin(25, 25);
                  Rollerspin(1, -80); //spin roller
                  MoveForwards(7, false, 15); //move away from roller and closer high goal
                  Turn(10,5,true); //turn towards high goal
                  Catapult.spin(fwd,70,pct);//catapult shoots
                  wait(0.25,sec);
                  IntakeMotor.spin(fwd,100,pct);
                  Turn(120,56,true); //turns towards 3 discs
                  MoveForwards(58, true, 50);//intakes 3 discs
                  Turn(-85,-40,true); //turns towards high goal
                  IntakeMotor.spin(fwd,0,pct);
                  MoveForwards(8, false, 10); //moves towards high goal
                  CataFire(); //shoots into high goal
                  /*
                    printf("Iterations:%d\n",AutoCode);
                    printf("--------Time Taken--------\n");
                    int AutoCode2 = AutoCode;
                    while(AutoCode2 > 0)
                      {
                        printf("%f\n",AutoCodeTime[AutoCode2]);
                        AutoCode2 = AutoCode2 - 1;
                      }
                    printf("--------Error--------\n");
                    while(AutoCode > 0)
                      {
                        printf("%f\n",AutoCodeError[AutoCode]);
                        AutoCode = AutoCode - 1;
                      }
                    printf("--------Fin--------\n");
                  while(!Controller1.ButtonA.pressing())
                  {
                    StartingPos = InertialSensor.rotation(deg);
                    InitialTime = Brain.timer(sec);
                    Turn(90,35,true);
                    wait(1,sec);
                    AutoCodeError[AutoCode] = (InertialSensor.rotation(deg))-(StartingPos);
                    AutoCodeTime[AutoCode] = (Brain.timer(sec) - (InitialTime+1));
                    AutoCode++;

                    StartingPos = InertialSensor.rotation();
                    InitialTime = Brain.timer(sec);
                    Turn(90,35,true);
                    wait(1,sec);
                    AutoCodeError[AutoCode] = InertialSensor.rotation(deg)-(StartingPos);
                    AutoCodeTime[AutoCode] = (Brain.timer(sec) - (InitialTime+1));
                    AutoCode++;

                  }
                  printf("Iterations:%d\n",AutoCode);
                  printf("--------Time Taken--------\n");
                  AutoCode2 = AutoCode;
                  while(AutoCode2 > 0)
                    {
                      printf("%f\n",AutoCodeTime[AutoCode2]);
                      AutoCode2 = AutoCode2 - 1;
                    }
                  printf("--------Error--------");
                  while(AutoCode > 0)
                    {
                      printf("%f\n",AutoCodeError[AutoCode]);
                      AutoCode = AutoCode - 1;
                    }
                  printf("--------Fin--------\n");

                  */
                  /*
                    printf("Iterations:%d\n",AutoCode);
                    printf("--------Time Taken--------\n");
                    int AutoCode2 = AutoCode;
                    while(AutoCode2 > 0)
                      {
                        printf("%f\n",AutoCodeTime[AutoCode2]);
                        AutoCode2 = AutoCode2 - 1;
                      }
                    printf("--------Error--------\n");
                    while(AutoCode > 0)
                      {
                        printf("%f\n",AutoCodeError[AutoCode]);
                        AutoCode = AutoCode - 1;
                      }
                    printf("--------Fin--------\n");
                  while(!Controller1.ButtonA.pressing())
                  {
                    StartingPos = RotationSensor.position(rev) * TrackingCircum;
                    InitialTime = Brain.timer(sec);
                    MoveForwards(48, false, 50);
                    wait(1,sec);
                    AutoCodeError[AutoCode] = (RotationSensor.position(rev) *  TrackingCircum)-(StartingPos);
                    AutoCodeTime[AutoCode] = (Brain.timer(sec) - (InitialTime+1));
                    AutoCode++;

                    StartingPos = RotationSensor.position(rev) * TrackingCircum;
                    InitialTime = Brain.timer(sec);
                    MoveForwards(48, true, 50);
                    wait(1,sec);
                    AutoCodeError[AutoCode] = (RotationSensor.position(rev) *  TrackingCircum)-(StartingPos);
                    AutoCodeTime[AutoCode] = (Brain.timer(sec) - (InitialTime+1));
                    AutoCode++;

                  }
                  printf("Iterations:%d\n",AutoCode);
                  printf("--------Time Taken--------\n");
                  AutoCode2 = AutoCode;
                  while(AutoCode2 > 0)
                    {
                      printf("%f\n",AutoCodeTime[AutoCode2]);
                      AutoCode2 = AutoCode2 - 1;
                    }
                  printf("--------Error--------");
                  while(AutoCode > 0)
                    {
                      printf("%f\n",AutoCodeError[AutoCode]);
                      AutoCode = AutoCode - 1;
                    }
                  printf("--------Fin--------\n");
                  */
                }

    }



}

int i2;

void AutoSelection(int Mod)
  {
      Brain.Screen.drawImageFromFile("AutoSelector2.png",0,0); 
      wait(0.5,sec);
      waitUntil(Brain.Screen.pressing());

      if(Brain.Screen.xPosition() > 240)
        {
          if(Brain.Screen.yPosition() > 136)
            {
              Auto = 4 + Mod;
            }
            else
              {
                Auto = 2 + Mod;
              }
        }
          else
            {
              if(Brain.Screen.yPosition() > 136)
                {
                  Auto = 3 + Mod;
                }
                  else
                    {
                      Auto = 1 + Mod;
                    }
            }

  }


void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();


  InertialSensor.calibrate();
  Brain.Screen.printAt(50, 50, false, "Calibrating..."); //Calibrates the Inertia sensor so heading is accurate
  waitUntil(!InertialSensor.isCalibrating());

  Brain.Screen.drawImageFromFile("MatchTypeSelector2.png",0,0); //starts the GUI that sets up the right Autonomous for matches or skills
  waitUntil(Brain.Screen.pressing());
  Brain.Screen.clearScreen();
  if(Brain.Screen.xPosition() > 240 ) //detects what side of the screen got pushed
    {
      Skills = false;
      Brain.Screen.drawImageFromFile("TeamSelector.png", 0, 0); //select either blue or red side for matches
      wait(0.5,sec);
      waitUntil(Brain.Screen.pressing());
      Brain.Screen.clearScreen();
        if(Brain.Screen.xPosition() > 240 )
          {
            AutoSelection(0);
            Brain.Screen.clearScreen();
            Brain.Screen.drawImageFromFile("FerociousFoxLogoRed.png",0,0); //logo for Red side
            Brain.Screen.drawImageFromFile("RedFox2.png",380,172); 
            Controller1.Screen.setCursor(0, 0);
            Controller1.Screen.print("Auto %f",Auto); //prints the auto on controller to make sure the right one was selected
          }
            else
              {
                AutoSelection(4);
                Brain.Screen.clearScreen();
                Brain.Screen.drawImageFromFile("BlueShark2.png",380,172); 
                Brain.Screen.drawImageFromFile("FerociousFoxLogoBlue.png",0,0); //logo for Blue side
                Controller1.Screen.setCursor(0, 0);
                Controller1.Screen.print("Auto %f",Auto);
                if (Auto == 8)
                  {
                    while(true)
                      {
                        Brain.Screen.clearScreen();
                        Brain.Screen.drawImageFromFile("Youareanidiot1.png",58,0); //Brain demonstation video
                        wait(0.5,sec);
                        Brain.Screen.setFillColor(white);
                        Brain.Screen.drawRectangle(0, 0, 480, 272);
                        Brain.Screen.drawImageFromFile("idiot2.png", 57, 0);
                        wait(0.5,sec);
                      }

                  }
              }

    }
      else
        {
          Skills = true;
          Brain.Screen.clearScreen();
      Brain.Screen.drawImageFromFile("SkillsSelector.png",0,0); //auto selector for skills
      wait(0.5,sec);
      waitUntil(Brain.Screen.pressing());

      if(Brain.Screen.xPosition() > 240)
        {
          if(Brain.Screen.yPosition() > 136)
            {
              Auto = 4;
            }
            else
              {
                Auto = 2;
                  Controller1.Screen.setCursor(0, 0);
                  Controller1.Screen.print("Auto %f",Auto);
              }
        }
          else
            {
              if(Brain.Screen.yPosition() > 136)
                {
                  Auto = 3;
                }
                  else
                    {
                      Auto = 1;
                    }
            }
            Controller1.Screen.setCursor(0, 0);
            Controller1.Screen.print("Auto %f",Auto);
            Brain.Screen.clearScreen();
            Brain.Screen.drawImageFromFile("BlueShark2.png",480,172);
            Brain.Screen.drawImageFromFile("FerociousFoxLogoBlue.png",0,0); 
        }
  
while(true)
  {
    wait(1,sec);
  }
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();
 Brain.Screen.drawImageFromFile("FerociousFoxLogoRed.png",127/2,0);  
  // Prevent main from exiting with an infinite loop.
while(true)
  {
    wait(1,sec);
  }
//    printf("Rotation Sensor: %f\n",RotationSensor.position(rev));
//    printf("Inertia Sensor: %f\n", InertialSensor.heading());
  }
