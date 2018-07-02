package leJOS_exclusive;          
                                  
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException; 
                                           
import ev3.exercises.library.Logging;    
//import gyroboy.GyroBoyApp2; 
import lejos.hardware.Button;

//public class Segoway { 
                                                                                                                                                                                             
import lejos.hardware.Sound; // TODO: Visual count-down only, no sound? Or some sort of sound interface and container for Sound class (can't implement interface on static methods)? 
import lejos.hardware.motor.UnregulatedMotor;        
import lejos.hardware.port.MotorPort; 
import lejos.hardware.port.SensorPort;           
import lejos.hardware.sensor.EV3GyroSensor;        
import lejos.robotics.EncoderMotor; 
import lejos.robotics.Gyroscope;                 
//import lejos.robotics.GyroscopeAdaptor;       
import lejos.robotics.SampleProvider;    
import lejos.utility.Delay;       
             
    /**                                                                                                        
     * <p>This class balances a two-wheeled Segway-like robot. It works with almost any construction          
     * (tall or short) such as the <a href="http://www.laurensvalk.com/nxt-2_0-only/anyway">Anyway</a> or      
     * the <a href="http://www.hitechnic.com/blog/gyro-sensor/htway/">HTWay</a>. Wheel diameter is the most 
     * important construction variable, which is specified in the constructor.</p>     
     *                                       
     * <p>To start the robot balancing:                                   
     * <li>1. Run the program. You will be prompted to lay it down.                                                                              
     * <li>2. Lay it down (orientation doesn't matter). When it detects it is not moving it will automatically calibrate the gyro sensor. 
     * <li>3. When the beeping begins, stand it up so it is vertically balanced.                
     * <li>4. When the beeping stops, let go and it will begin balancing on its own.</p> 
     *                                                                                               
     * <p>Alternately you can lean the robot against a wall and run the program. After the gyro         
     * calibration, the robot backs up against the wall until it falls forward. When it detects the   
     * forward fall, it start the balance loop.</p>        
     *                                                                                                
     * <p>NOTE: In order to make the robot move and navigate, use the SegowayPilot class.</p>  
     *                                                                                                                                   
     * <p><i>This code is based on the <a href="http://www.hitechnic.com/blog/gyro-sensor/htway/">HTWay</a> by HiTechnic and the program from BB.</i></p> 
     *              
     * @author This is the work of GitHub user 71104 - Alberto La Rocca    
     *         
     */                                                                                               
    public class Segoway extends Thread                                                                                                                                         
        {    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////        
        // TODO: Thread should be a private inner class.   //public class Segoway implements Runnable                                                               //             
                                                                                                                                                                    //                                
        // Motors and gyro:                                                                                                                                         //                                  
        //private EV3GyroSensor gyro;                                                                                                                               //                              
        //private GyroSensor gyro;                                                                                                                                  //                               
        private static Gyroscope gyro;                                                                                                                              //           
        protected EncoderMotor left_motor;                                                                                                                          //                        
        protected EncoderMotor right_motor;                                                                                                                         //                             
                                                                                                                                                                    //                               
        //=====================================================================                                                                                     //                              
        // Balancing constants                                                                                                                                      //                        
        //                                                                                                                                                          //                                
        // These are the constants used to maintain balance.                                                                                                        //                               
        //=====================================================================                                                                                     //                           
                                                                                                                                                                    //      
        /**                                                                                                                                                         //                                           
         * Loop wait time.  WAIT_TIME is the time in ms passed to the Wait command.                                                                                 //                                
         * NOTE: Balance control loop only takes 1.128 ms in leJOS NXJ.                                                                                             //                                              
         */                                                                                                                                                         //                                              
        //private static final int WAIT_TIME = 7; // originally 8                                                                                                   //                                             
        //private static final int WAIT_TIME = 6; // originally 8                                                                                                   //                                    
        //private static final int WAIT_TIME = 7 ; // originally 8                                                                                                  //                                        
        //private static final int WAIT_TIME = 8 ; // originally 8                                                                                                  //                                              
        private static final int WAIT_TIME = 5 ; // originally 8                                                                                                    //                                              
                                                                                                                                                                    //                                           
        // These are the main four balance constants, only the gyro                                                                                                 //                                               
        // constants are relative to the wheel size.  KPOS and KSPEED                                                                                               //                                            
        // are self-relative to the wheel size.                                                                                                                     //                                              
        //private static final double KGYROANGLE = 7.5;//8;                                                                                                         //                                              
        //private static final double KGYROANGLE = 7.7;//8;                                                                                                         //                                                   
        //private static final double KGYROANGLE = 8.4;//8;                                                                                                         //                 
        private static final double KGYROANGLE = 9;                                                                                                                 //           
        //private static final double KGYROSPEED = 2.0;//2;                                                                                                         //           
        private static final double KGYROSPEED = 3;                                                                                                                 //           
        private static final double KPOS = 0.08;//0.1;                                                                                                              //       
        private static final double KSPEED = 0.2;                                                                                                                   //           
                                                                                                                                                                    //             
        //double KGYROANGLE2 = 9; double KGYROSPEED2 = 3; double KPOS2 = 0.08; double KSPEED2 = 0.2;    double KDRIVE2 = 0.2;                                       //        
                                                                                                                                                                    //            
        /**                                                                                                                                                         //        
         * This constant aids in drive control. When the robot starts moving because of user control,                                                               //        
         * this constant helps get the robot leaning in the right direction.  Similarly, it helps                                                                   //       
         * bring robot to a stop when stopping.                                                                                                                     //        
         */                                                                                                                                                         //        
        private static final double KDRIVE = -0.02;                                                                                                                 //           
        //private static final double KDRIVE = 0.0;                                                                                                                 //           
                                                                                                                                                                    //      
        /**                                                                                                                                                         //          
         * Power differential used for steering based on difference of target steering and actual motor difference.                                                 //          
         */                                                                                                                                                         //          
        private static final double KSTEER = 0.25;                                                                                                                  //           
                                                                                                                                                                    //          
        /**                                                                                                                                                         //        
         * Gyro offset control                                                                                                                                      //      
         * The gyro sensor will drift with time.  This constant is used in a simple long term averaging                                                             //      
         * to adjust for this drift. Every time through the loop, the current gyro sensor value is                                                                  //    
         * averaged into the gyro offset weighted according to this constant.                                                                                       //   
         */                                                                                                                                                         //        
        //private static final double EMAOFFSET = 0.0005;    //    Original    ////////////////////////////////////////                                             //        
                                                                                                                                                                    //     
        /**                                                                                                                                                         //    
         * If robot power is saturated (over +/- 100) for over this time limit then                                                                                 //   
         * robot must have fallen.  In milliseconds.                                                                                                                //      
         */                                                                                                                                                         //         
        private static final double TIME_FALL_LIMIT = 500; // originally 1000                                                                                       //         
                                                                                                                                                                    //         
        //---------------------------------------------------------------------                                                                                     //             
                                                                                                                                                                    //                
        /**                                                                                                                                                         //                
         * This constant is in degrees/second for maximum speed.  Note that position                                                                                //         
         * and speed are measured as the sum of the two motors, in other words, 600                                                                                 //             
         * would actually be 300 degrees/second for each motor.                                                                                                     //        
         */                                                                                                                                                         //      
        //private static final double CONTROL_SPEED  = 600.0;    //    Segoway original                                                                             //       
        //private static final double CONTROL_SPEED  = 0.0;                                                                                                         //       
        private static final double CONTROL_SPEED  = -600.0;                                                                                                        //       
                                                                                                                                                                    //         
        //=====================================================================                                                                                     //          
        // Global variables                                                                                                                                         //             
        //=====================================================================                                                                                     //           
                                                                                                                                                                    //           
        // These two xxControlDrive variables are used to control the movement of the robot. Both                                                                   //              
        // are in degrees/second:                                                                                                                                   //                    
        /**                                                                                                                                                         //                
         * motorControlDrive is the target speed for the sum of the two motors                                                                                      //              
         * in degrees per second.                                                                                                                                   //           
         */                                                                                                                                                         //           
        private double motorControlDrive = 0.0;    //    Segoway original                                                                                           //         
                                                                                                                                                                    // 
        /**                                                                                                                                                         //             
         * motorControlSteer is the target change in difference for two motors                                                                                      //              
         * in degrees per second.                                                                                                                                   //              
         */                                                                                                                                                         //            
        private double motorControlSteer = 0.0;                                                                                                                     //                   
                                                                                                                                                                    //                 
        /**                                                                                                                                                         //                    
         * This global contains the target motor differential, essentially, which                                                                                   //                      
         * way the robot should be pointing.  This value is updated every time through                                                                              //               
         * the balance loop based on motorControlSteer.                                                                                                             //           
         */                                                                                                                                                         //        
        private double motorDiffTarget = 0.0;                                                                                                                       //      
                                                                                                                                                                    //   
        /**                                                                                                                                                         //     
         * Time that robot first starts to balance.  Used to calculate tInterval.                                                                                   // 
         */                                                                                                                                                         // 
        private long tCalcStart;                                                                                                                                    //  
                                                                                                                                                                    // 
        /**                                                                                                                                                         //                       
         * tInterval is the time, in seconds, for each iteration of the balance loop.                                                                               //                        
         */                                                                                                                                                         //  
        private double tInterval;                                                                                                                                   // 
                                                                                                                                                                    // 
        /**                                                                                                                                                         // 
         * ratioWheel stores the relative wheel size compared to a standard NXT 1.0 wheel.                                                                          //           
         * RCX 2.0 wheel has ratio of 0.7 while large RCX wheel is 1.4.                                                                                             // 
         */                                                                                                                                                         // 
        private double ratioWheel;                                                                                                                                  // 
                                                                                                                                                                    //  
        // Gyro globals                                                                                                                                             //            
        private double gOffset;                                                                                                                                     //             
        private double gAngleGlobal = 0;                                                                                                                            //      
        long cLoop = 0;                                                                                                                                             //       
                                                                                                                                                                    //  
        // Motor globals                                                                                                                                            //  
        private double motorPos = 0;                                                                                                                                //      
        private long mrcSum = 0, mrcSumPrev;                                                                                                                        //                      
        private long motorDiff;                                                                                                                                     //             
        private long mrcDeltaP3 = 0;                                                                                                                                //           
        private long mrcDeltaP2 = 0;                                                                                                                                //      
        private long mrcDeltaP1 = 0;                                                                                                                                //      
        private static UnregulatedMotor leftMotor = new UnregulatedMotor(MotorPort.D);                                                                              //        
        private static UnregulatedMotor rightMotor = new UnregulatedMotor(MotorPort.A);                                                                             //   
        private EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S2);                                                                                        //   
        double[] Duration = new double[10];                                                                                                                         //   
        double testv[] = new double[10];                                                                                                                            //  
        int TempIndex;                                                                                                                                              //  
        double Time;                                                                                                                                                //  
        long MyTime;                                                                                                                                                //  
        long mytime;                                                                                                                                                //  
        long now1;                                                                                                                                                  //                
        long now2;                                                                                                                                                  //            
        //double KGYROANGLE2;      double KGYROSPEED2;      double KPOS2;         double KSPEED2;                                                                   //                      
        long mrcLeft, mrcRight, mrcDelta;    //    Segoway original                                                                                                 //           
                                                                                                                                                                    //                                           
        int loopCount = 1;            // postpone activation of the motors until dt in the loop is stable            // From GyroBoy                                //                    
                                                                                                                                                                    //      
        //int TestInterval = (int) 1E0;    //          1;                                                                                                           //                                       
        //int TestInterval = (int) 1E1;    //         10;                                                                                                           //                                    
        //int TestInterval = (int) 1E2;    //        100;                                                                                                           //                                        
        //int TestInterval = (int) 1E3;    //       1000;                                                                                                           //                                                    
        int TestInterval = (int) 1E4;    //      10000;                                                                                                             //                                 
        //int TestInterval = (int) 1E5;    //     100000;                                                                                                           //                            
        //int TestInterval = (int) 1E6;    //    1000000;                                                                                                           //                               
                                                                                                                                                                    //             
        BufferedWriter bw = null;                                                                                                                                   //                 
        //boolean Debug = false;                                                                                                                                    //                                 
        boolean Debug = true;                                                                                                                                       //                                 
        double RunTimeMS;                                                                                                                                           //          
        double RunTimeS;                                                                                                                                            //             
        double AverageMS;         //    1E6   ms                                                                                                                    //                   
        double AverageS;    //    1E6   ms                                                                                                                          //               
        double KGYROANGLE2 = 9; double KGYROSPEED2 = 3; double KPOS2 = 0.08; double KSPEED2 = 0.2;    double KDRIVE2 = 0.2;                                         //        
                                                                                                                                                                    //         
        double[] TestIntervalA      =  new double [TestInterval] ;                                                                                                  //           
        int   [] loopCountA         =  new int    [TestInterval] ;                                                                                                  //                            
        double[] gyroAngleA         =  new double [TestInterval] ;                                                                                                  //            
        double[] gyroSpeedA         =  new double [TestInterval] ;                                                                                                  //            
        double[] motorPosA          =  new double [TestInterval] ;                                                                                                  //               
        double[] motorSpeedA        =  new double [TestInterval] ;                                                                                                  //               
        double[] gAngleGlobalA      =  new double [TestInterval] ;                                                                                                  //           
        double[] mrcDeltaA          =  new double [TestInterval] ;                                                                                                  //              
        double[] mrcDeltaP1A        =  new double [TestInterval] ;                                                                                                  //             
        double[] mrcDeltaP2A        =  new double [TestInterval] ;                                                                                                  //            
        double[] mrcDeltaP3A        =  new double [TestInterval] ;                                                                                                  //             
        double[] powerA             =  new double [TestInterval] ;                                                                                                  //               
        double[] powerLeftA         =  new double [TestInterval] ;                                                                                                  //             
        double[] powerRightA        =  new double [TestInterval] ;                                                                                                  //            
        double[] mrcLeftA           =  new double [TestInterval] ;                                                                                                  //             
        double[] mrcRightA          =  new double [TestInterval] ;                                                                                                  //              
        double[] tCalcStartA        =  new double [TestInterval] ;                                                                                                  //           
        double[] tIntervalA         =  new double [TestInterval] ;                                                                                                  //          
        double[] tMotorPosOKA       =  new double [TestInterval] ;                                                                                                  //          
        double[] RunTimeMSA         =  new double [TestInterval] ;                                                                                                  //           
        double[] RunTimeSA          =  new double [TestInterval] ;                                                                                                  //           
        double[] motorControlDriveA =  new double [TestInterval] ;    //    target speed in degrees per second                                                      //           
                                                                                                                                                                   //              
public static void main(String[] args) {    /////////////////////////////////////////////////////////////////////////////                                          //                          
        long now2;                                                                                                     //                                          //                          
        long now21;                                                                                                    //                                          //                     
        System.out.println();                                                                                          //                                         //                                  
        System.out.println("6/18/2018");                                                                               //                                         //                                    
        System.out.println( "IIIIIIIIII\r\n" +                                                                         //                                         //                                    
                            "HHHHHHHHHH\r\n" +                                                                         //                                         //                             
                            "AAAAATTTTT\r\n" +                                                                         //                                        //                            
                            "ATATATATAT");                                                                             //                                        //                     
                                                                                                                       //                                        //                         
        System.out.println();                                                                                          //                                        //                                                 
        System.out.printf("%20s\t\tleJOS PROJECT JavaSE-1.7 5/21/2018\n", "Project:");                                 //                                        //                       
        System.out.printf("%20s\tSegoway\n", "Eclipse name:");                                                         //                                       //                           
        System.out.println();                                                                                          //                                       //                       
        //now = System.nanoTime();                                               //     6000000                        //                                       //                       
        try                                                                                                            //                                       //             
            {    ////////////////////////////////////////////////////////////////////////////////////////////////////  //                                       //       
            Logging.setup(Segoway.class.getPackage(), false);                                                      //  //                                       //
            //Logging.log("Loop    gyroSpeed            gyroAngle");                                               //  //                                       //        
            //Logging.log("Starting Segoway Logging");                                                             //  //                                       //       
            //Logging.log("%5d,  \t%10.10f,    \t%10.10f\n", cLoop, gyroSpeed, gyroAngle);                         //  //                                       //     
            //           <13>05:03:29:357 Segoway.body(Segoway.java:435):     1,      0.0000000000,        0.0000000000//                                       //                                                      
            //Logging.log("     Loop                             Angle                       AngleMultiplyer       //  //          SpeedMultiplier                                Power              Interval");           
            }    ////////////////////////////////////////////////////////////////////////////////////////////////////  //                                       //     
        catch (IOException e)                                                                                          //                                       //     
           {    /////////////////////                                                                                  //                                       //           
           e.printStackTrace();    //                                                                                  //                                       //
           }    /////////////////////                                                                                  //                                       // 
                                                                                                                       //                                       //
        System.out.println("4");                                                                                       //                                       //       
        System.out.println("3");                                                                                       //                                       //       
        System.out.println("2");                                                                                       //                                       //       
        System.out.println("1");                                                                                       //                                       //       
                                                                                                                       //                                       //          
        // feed back loop                                                                                              //                                       //         
        //long StartTime = System.currentTimeMillis();  //    1E6 From StatementTimeTest                               //                                       //          
        long StartTime = System.currentTimeMillis();             //    1E9                                             //                                       //          
                                                                                                                       //                                       //       
        //while (Button.ESCAPE.isUp())                                                                                 //                                       //              
        //while (loopCount < TestInterval)                                                                             //                                       //
        //tCalcStart = System.currentTimeMillis();                                                                     //                                       //
        //now = System.nanoTime(); //     6000000                                                                      //                                       //
        //tCalcStart = System.currentTimeMillis();                                                                     //                                       //
        //MyTime = System.nanoTime();                                                                                  //                                       //
        //mytime = System.nanoTime();                                                                                  //                                       //
        //Delay.msDelay(  60 - (now-lastTimeStep)/1000000);             //   10                                        //                                       //
        //Delay.msDelay(  60);                                                                                         //                                       //
        //Logging.log("Starting Segoway Logging");                                                                     //                                      //   
        //SampleProvider gyrosampler = new EV3GyroSensor(LocalEV3.get().getPort("S2")).getRateMode();                  //                                      //
        //SampleProvider gyrosampler = new EV3GyroSensor(SensorPort.S2).getRateMode();    //    Original               //                                      //
        //float gyrofreq = gyrosampler.sampleSize();                                                                   //                                      //
                                                                                                                       //                                      //
        //EncoderMotor left = new NXTMotor(MotorPort.A);                                                               //                                      //
        //EncoderMotor right= new NXTMotor(MotorPort.D);                                                               //                                      //
        //EV3GyroSensor gyro = new EV3GyroSensor(LocalEV3.get().getPort("S2"));                                        //                                      //  
        //Gyroscope gyro = new GyroscopeAdaptor(gyrosampler, gyrofreq);                                                //                                      //
        double wheelDiameter = 5.6; // cm                                                                              //                                      //
                                                                                                                       //                                     //
        Segoway segway = new Segoway(leftMotor, rightMotor, gyro, wheelDiameter);                                      //                                     //  
        //lejos.robotics.navigation.Ballbot bb = new Ballbot(xMotor, xGyro, yMotor, yGyro, rollerDiameter)             //                                     // 
        Thread t1 = new Thread(segway);                                                                                //                                     //
        t1.start();                                                                                                    //                                     //
        }    ////////////////////////////////////////////////////////////////////////////////////////////////////////////                                     //    
                                                                                                                                                              //
        /**                                                                                                                                                  //
         * Creates an instance of the Segoway, prompts the user to lay Segoway flat for gyro calibration,                                                    //
         * then begins self-balancing thread. Wheel diameter is used in balancing equations.                                                                 // 
         *                                                                                                                                                   //
         *  <li>NXT 1.0 wheels = 5.6 cm                                                                                                                      //
         *  <li>NXT 2.0 wheels = 4.32 cm                                                                                                                     //             
         *  <li>RCX "motorcycle" wheels = 8.16 cm                                                                                                           //     
         *                                                                                                                                                  //             
         * @param left The left motor. An unregulated motor.                                                                                               //      
         * @param right The right motor. An unregulated motor.                                                                                            // 
         * @param gyro A HiTechnic gyro sensor                                                                                                            //                            
         * @param wheelDiameter diameter of wheel, preferably use cm (printed on side of LEGO tires in mm)                                               //                   
         */                                                                                                                                              //                      
        //public Segoway(EncoderMotor left, EncoderMotor right, GyroSensor gyro, double wheelDiameter) {                                                //                        
        //public Segoway(EncoderMotor left, EncoderMotor right, EV3GyroSensor gyro, double wheelDiameter) {                                             //                   
        public Segoway(EncoderMotor left, EncoderMotor right, Gyroscope gyro, double wheelDiameter) {    ////////                                       //                 
        this.left_motor = left;                                                                                //                                       //
        this.right_motor = right;                                                                              //                                       //
        // Optional code to accept BasicMotor: this.right_motor = (NXTMotor)right;                             //                                       //                    
        this.gyro = gyro;                                                                                      //                                       //                   
        this.ratioWheel = wheelDiameter/5.6; // Original algorithm was tuned for 5.6 cm NXT 1.0 wheels.        //                                       //                         
                                                                                                               //                                       //                            
        // Took out 50 ms delay here.                                                                          //                                       //                        
                                                                                                               //                                       //                                
        // Get the initial gyro offset                                                                         //                                       //                                              
        gyroSensor.reset();                                                                                    //                                       //                                                 
                                                                                                               //                                       //                                                   
        // Play warning beep sequence before balance starts                                                    //                                       //                                            
                                                                                                               //                                       //                                                
        // Start balance thread                                                                                //                                       //                                               
        //            this.setDaemon(true);                                                                    //                                       //                                                
        //            this.start();                                                                            //                                       //                                            
        }    ////////////////////////////////////////////////////////////////////////////////////////////////////                                       //                                               
                                                                                                                                                        // 
        /**                                                                                                                                             //                        
         * Warn user the Segoway is about to start balancing.                                                                                           //               
         */                                                                                                                                             //                    
        private void startBeeps()                                                                                                                       //                              
            {    /////////////////////////////////////////////////////////////////                                                                      //                                     
                                                                                //                                                                      //                             
            System.out.printf("Balance in " );                                  //                                                                      //                                         
                                                                                //                                                                      //                             
            // Play warning beep sequence to indicate balance about to start    //                                                                      //                           
            for (int c=4; c>0; c--)                                             //                                                                      //                           
                {   ///////////////////////////////                             //                                                                      //                       
                System.out.print(c + " ");       //                             //                                                                      //                
                  Sound.playTone(440,100);       //                             //                                                                      //                 
                try                              //                             //                                                                      //                              
                    {    ////////////////////    //                             //                                                                      //                            
                    Thread.sleep(1000);    //    //                             //                                                                      //                        
                    }    ////////////////////    //                             //                                                                      //                         
                catch (InterruptedException e)   //                             //                                                                      //                              
                    {    ////                    //                             //                                                                      //                            
                           //                    //                             //                                                                      //                           
                    }    ////                    //                             //                                                                      //                        
                                                 //                             //                                                                      //                         
                }    //////////////////////////////                             //                                                                      //                           
            System.out.println("GO");                                           //                                                                      //                            
            //System.out.println();                                             //                                                                      //                               
            Sound.playTone(540,100);                                            //                                                                      //                                  
            }   //////////////////////////////////////////////////////////////////                                                                      //                            
                                                                                                                                                        //                    
        /**                                                                                                                                             //                   
         * Get the data from the gyro.                                                                                                                  //                           
         * Fills the pass by reference gyroSpeed and gyroAngle based on updated information from the Gyro Sensor.                                       //                                
         * Maintains an automatically adjusted gyro offset as well as the integrated gyro angle.                                                        //                         
         *                                                                                                                                              //             
         */                                                                                                                                             //                
        private void updateGyroData()                                                                                                                   //      
            {    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////               //                
            // TODO: The GyroSensor class actually rebaselines for drift ever 5 seconds. This not needed? Or is this method better?    //               //         
            // Some of this fine tuning may actually interfere with fine-tuning happening in the hardcoded dIMU and GyroScope code.    //               //              
            //float gyroRaw = 0;                                                                                                       //               //            
                                                                                                                                       //               //            
            SampleProvider gyroReader = gyroSensor.getRateMode();   //    GyroBoy                                                      //               //              
            float[] sample = new float[gyroReader.sampleSize()];    //    GyroBoy                                                      //               //             
            //gyroRaw = gyro.getAngularVelocity();    //    NXT/NXJ statement ?    //    Original                                      //               //             
            gyroSensor.fetchSample(sample, 0);                                                                                         //               //            
            gyroSpeed = -sample[0]; // invert sign to undo negation in class EV3GyroSensor                                             //               //               
            //gOffset = EMAOFFSET * gyroRaw + (1-EMAOFFSET) * gOffset;             //    Original                                      //               //              
            //gyroSpeed = gyroRaw - gOffset; // Angular velocity (degrees/sec)     //    Original                                      //               //              
                                                                                                                                       //               //           
            gAngleGlobal += gyroSpeed*tInterval;                                                                                       //               //           
            gyroAngle = gAngleGlobal; // Absolute angle (degrees)                                                                      //               //               
            //gyroAngle = ((ga1 + ga2 + ga3 + ga4) / 4.0);       From GyroBoy                                                          //               //           
            //ga4 = ga3;                                         From GyroBoy                                                          //               //          
            //ga3 = ga2;                                         From GyroBoy                                                          //               //         
            //ga2 = ga1;                                         From GyroBoy                                                          //               //           
            }    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////               //
                                                                                                                                                        //               
        /**                                                                                                                                             //
         * Keeps track of wheel position with both motors.                                                                                              //
         */                                                                                                                                             //                
        private void updateMotorData() {    /////////////////////////////////////////////                                                               //                                     
            //long mrcLeft, mrcRight, mrcDelta;    //    Segoway original              //                                                               //                
                                                                                       //                                                               //                     
            // Keep track of motor position and speed                                  //                                                               //                              
            mrcLeft  = left_motor. getTachoCount();                                    //                                                               //                                   
            mrcRight = right_motor.getTachoCount();                                    //                                                               //                                       
                                                                                       //                                                               //                    
            // Maintain previous mrcSum so that delta can be calculated and get        //                                                               //                      
            // new mrcSum and Diff values                                              //                                                               //                       
            mrcSumPrev = mrcSum;                                                       //                                                               //                           
            mrcSum = mrcLeft + mrcRight;                                               //                                                               //                            
            motorDiff = mrcLeft - mrcRight;                                            //                                                               //                           
                                                                                       //                                                               //                          
            // mrcDetla is the change int sum of the motor encoders, update            //                                                               //                         
            // motorPos based on this detla                                            //                                                               //                          
            mrcDelta = mrcSum - mrcSumPrev;                                            //                                                               //                          
            motorPos += mrcDelta;                                                      //                                                                //                         
                                                                                       //                                                                //                          
            // motorSpeed is based on the average of the last four delta's.            //                                                                //                        
            motorSpeed = (mrcDelta+mrcDeltaP1+mrcDeltaP2+mrcDeltaP3)/(4*tInterval);    //                                                                  //                           
                                                                                       //                                                                  //                       
            // Shift the latest mrcDelta into the previous three saved delta values    //                                                                  //                     
            mrcDeltaP3 = mrcDeltaP2;                                                   //                                                                  //                           
            mrcDeltaP2 = mrcDeltaP1;                                                   //                                                                   //                       
            mrcDeltaP1 = mrcDelta;                                                     //                                                                    //                       
        }    ////////////////////////////////////////////////////////////////////////////                                                                    //                         
                                                                                                                                                             //                            
        /**                                                                                                                                                  //                        
         * Global variables used to control the amount of power to apply to each wheel.                                                                      //                         
         * Updated by the steerControl() method.                                                                                                             //                            
         */                                                                                                                                                  //                             
        private int powerLeft, powerRight; // originally local variables                                                                                     //                           
                                                                                                                                                             //                             
        /**                                                                                                                                                  //                                
         * This function determines the left and right motor power that should                                                                               //                                 
         * be used based on the balance power and the steering control.                                                                                      //                                  
         */                                                                                                                                                   //                             
        private void steerControl(int power)                                                                                                                  //                              
            {    ///////////////////////////////////////////////////////////////////                                                                          //                                                  
            int powerSteer;                                                       //                                                                          //                           
                                                                                  //                                                                          //                           
            // Update the target motor difference based on the user steering      //                                                                          //                             
            // control value.                                                     //                                                                          //                            
            motorDiffTarget += motorControlSteer * tInterval;                     //                                                                          //                             
                                                                                  //                                                                          //                            
            // Determine the proportionate power differential to be used based    //                                                                          //                             
            // on the difference between the target motor difference and the      //                                                                          //                              
            // actual motor difference.                                           //                                                                           //                             
            powerSteer = (int)(KSTEER * (motorDiffTarget - motorDiff));           //                                                                           //                              
                                                                                  //                                                                           //                             
            // Apply the power steering value with the main power value to        //                                                                           //                             
            // get the left and right power values.                               //                                                                           //                              
            powerLeft = power + powerSteer;                                       //                                                                           //                               
            powerRight = power - powerSteer;                                      //                                                                            //                                    
                                                                                  //                                                                            //                               
            // Limit the power to motor power range -100 to 100                   //                                                                            //                             
            if (powerLeft > 100)   powerLeft = 100;                               //                                                                            //                             
            if (powerLeft < -100)  powerLeft = -100;                              //                                                                            //                               
                                                                                  //                                                                            //                               
            // Limit the power to motor power range -100 to 100                   //                                                                            //                              
            if (powerRight > 100)  powerRight = 100;                              //                                                                            //                            
            if (powerRight < -100) powerRight = -100;                             //                                                                            //                            
                                                                                  //                                                                            //                            
            //Logging.log("     Loop    gyroSpeed            gyroAngle");         //                                                                            //                   
            //Logging.log("%5d,   \t\t%10.10f,   \t\t%10.10f,   \t\t\t%10.10f,   \t\t\t%10d,\t\t\t%10.10f\n", cLoop, 10*gyroAngle, testv[TempIndex]*gyroAngle, KGYROSPEED*gyroSpeed, powerLeft, 1000*tInterval);          
            }    ///////////////////////////////////////////////////////////////////                                                                            //                
                                                                                                                                                                //          
        /**                                                                                                                                                     //         
         * Calculate the interval time from one iteration of the loop to the next.                                                                              //
         * Note that first time through, cLoop is 0, and has not gone through                                                                                   //
         * the body of the loop yet.  Use it to save the start time.                                                                                            //
         * After the first iteration, take the average time and convert it to                                                                                   //        
         * seconds for use as interval time.                                                                                                                    //   
         */                                                                                                                                                     //
        private void calcInterval(long cLoop)                                                                                                                   //
            {    ////////////////////////////////////////////////////////////////////////////////                                                               //                   
            if (cLoop == 0)                                                                    //                                                               //    
                {    /////////////////////////////////////////////////////////                 //                                                               //         
                // First time through, set an initial tInterval time and    //                 //                                                               //      
                // record start time                                        //                 //                                                                //
                tInterval = 0.0055;                                         //                 //                                                                 //
                tCalcStart = System.currentTimeMillis();                    //                 //                                                                 //
                }    /////////////////////////////////////////////////////////                 //                                                                 //        
            else                                                                               //                                                                 //
                {    //////////////////////////////////////////////////////////////////////    //                                                                 //                                                    
                // Take average over number of times through the loop and                //    //                                                                 //
                // use for interval time.                                                //    //                                                                 //
                tInterval = (System.currentTimeMillis() - tCalcStart)/(cLoop*1000.0);    //    //                                                                 //    
                //System.out.println(tInterval);                                         //    //                                                                  //  
                //System.out.printf("Cycle time is %3.0f ms\n", tInterval*1000);         //    //                                                                  //   
                }    //////////////////////////////////////////////////////////////////////    //                                                                  //
                                                                                               //                                                                  //
            }    ////////////////////////////////////////////////////////////////////////////////                                                                  //
                                                                                                                                                                   //
        private double gyroSpeed, gyroAngle; // originally local variables                                                                                         //
        private double motorSpeed; // originally local variable                                                                                                     //
                                                                                                                                                                    //
        //---------------------------------------------------------------------                                                                                     //            
        //                                                                                                                                                          //  
        // This is the main balance thread for the robot.                                                                                                           // 
        //                                                                                                                                                          //
        // Robot is assumed to start leaning on a wall.  The first thing it                                                                                         //
        // does is take multiple samples of the gyro sensor to establish and                                                                                        // 
        // initial gyro offset.                                                                                                                                     //
        //                                                                                                                                                           //
        // After an initial gyro offset is established, the robot backs up                                                                                           //
        // against the wall until it falls forward, when it detects the                                                                                              //
        // forward fall, it start the balance loop.                                                                                                                  //
        //                                                                                                                                                           //
        // The main state variables are:                                                                                                                             //              
        // gyroAngle  This is the angle of the robot, it is the results of                                                                                           //         
        //            integrating on the gyro value.                                                                                                                  //                  
        //            Units: degrees                                                                                                                                  //    
        // gyroSpeed  The value from the Gyro Sensor after offset subtracted                                                                                          //      
        //            Units: degrees/second                                                                                                                           //
        // motorPos   This is the motor position used for balancing.                                                                                                  //
        //            Note that this variable has two sources of input:                                                                                               //        
        //             Change in motor position based on the sum of                                                                                                   //
        //             MotorRotationCount of the two motors,                                                                                                          //  
        //            and,                                                                                                                                             //          
        //             forced movement based on user driving the robot .                                                                                               //      
        //            Units: degrees (sum of the two motors)                                                                                                           //       
        // motorSpeed This is the speed of the wheels of the robot based  on the                                                                                       //   
        //            motor encoders.                                                                                                                                  //
        //            Units: degrees/second (sum of the two motors)                                                                                                    //
        //                                                                                                                                                             //         
        // From these state variables, the power to the motors is determined                                                                                            //        
        // by this linear equation:                                                                                                                                      //
        //     power = KGYROSPEED * gyro +                                                                                                                               //
        //             KGYROANGLE * gyroAngle +                                                                                                                          //     
        //             KPOS       * motorPos +                                                                                                                           //
        //             KSPEED     * motorSpeed;                                                                                                                          //           
        //                                                                                                                                                               //           
        public void body()                                                                                                                                               //                
            {    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////       //                                        
            int power;                                                                                                                                           //        //                       
            long tMotorPosOK;                                                                                                                                    //          //                           
                                                                                                                                                                 //          //                              
            try                                                                                                                                                  //          //                                           
                {    /////////////////////////////////////////////////////                                                                                       //          //                                            
                //Specify the file name and path here                   //                                                                                       //          //                                          
                File file = new File("myfile.txt");                     //                                                                                       //          //                                          
                /* This logic will make sure that the file              //                                                                                       //          //                                          
                 * gets created if it is not present at the             //                                                                                       //          //                                           
                 * specified location*/                                 //                                                                                       //          //                                             
                if (!file.exists())                                     //                                                                                       //          //                                                   
                    {    ///////////////////////                        //                                                                                       //          //                                              
                    file.createNewFile();    //                         //                                                                                       //          //                                             
                    }    ///////////////////////                        //                                                                                       //           //             
                FileWriter fw = new FileWriter(file);                   //                                                                                       //           //              
                bw = new BufferedWriter(fw);                            //                                                                                       //           //            
                System.out.println("File written Successfully");        //                                                                                       //           //                    
                }    /////////////////////////////////////////////////////                                                                                       //           //                
            catch (IOException ioe)                                                                                                                              //           //                    
                {    ///////////////////////                                                                                                                     //           //           
                ioe.printStackTrace();    //                                                                                                                     //           //                   
                }    ///////////////////////                                                                                                                     //           //                                    
             finally                                                                                                                                             //           //                  
                {    ////                                                                                                                                        //           //                     
                }    ////                                                                                                                                        //           //                  
                                                                                                                                                                 //            //                    
            //try {                                                                                                                                              //             //                                                                 
            //    //bw.write("File written Successfully");                                                                                                       //             //                                      
            //    //bw.newLine();                                                                                                                                //             //                     
            //    }                                                                                                                                              //              //                
            //catch (IOException e1)                                                                                                                             //              //             
            //    {                                                                                                                                              //              //                
            //    // TODO Auto-generated catch block                                                                                                             //              //                
            //    e1.printStackTrace();                                                                                                                          //               //                  
            //    }                               //                                                                                                             //               //                  
            //now = System.nanoTime();                                               //     6000000                                                              //                //                           
                                                                                                                                                                 //                //                  
            startBeeps();                                                                                                                                        //                //                       
            //System.out.println();                                                                                                                              //                  //                      
                                                                                                                                                                 //                   //                 
            tMotorPosOK = System.currentTimeMillis();                                                                                                            //                   //              
                                                                                                                                                                 //                    //               
            // Reset the motors to make sure we start at a zero position                                                                                         //                     //            
            left_motor.resetTachoCount();                                                                                                                        //                      //        
            right_motor.resetTachoCount();                                                                                                                       //                      //      
                                                                                                                                                                  //                       //     
            System.out.println("4");                                                                                                                               //                       //        
            System.out.println("3");                                                                                                                                //                       //        
            System.out.println("2");                                                                                                                                 //                       //        
            System.out.println("1");                                                                                                                                  //                        //        
            System.out.printf("Balancing..................              \n");                                                                                          //                        //                      
                                                                                                                                                                        //                        //           
            // feed back loop                                                                                                                                            //                         //          
            //long StartTime = System.currentTimeMillis();  //    1E6 From StatementTimeTest                                                                              //                         //           
            long StartTime = System.currentTimeMillis();             //    1E9                                                                                             //                         //           
                                                                                                                                                                            //                         //        
            //while (Button.ESCAPE.isUp())                                                                                                                                   //                         //                                 
            //while (loopCount < TestInterval)                                                                                                                                //                         //     
            // NOTE: This balance control loop only takes 1.128 MS to execute each loop in leJOS NXJ.                                                                          //                         //     
            while (Button.ESCAPE.isUp())                                                                                                                                        //                          //      
                {    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    //                          //                               
                calcInterval(cLoop++);                                                                                                                                     //    //                           //   
                                                                                                                                                                           //    //                            //   
                updateGyroData();                                                                                                                                          //    //                             //   
                                                                                                                                                                           //    //                              //  
                updateMotorData();                                                                                                                                         //    //                              //   
                                                                                                                                                                           //    //                               //  
                // Apply the drive control value to the motor position to get robot to move.                                                                               //    //                               //  
                //motorPos -= motorControlDrive * tInterval;    //    Segoway original                                                                                     //    //                                //   
                //motorPos = 0;    //    Bad idea                                                                                                                          //    //                                //   
                                                                                                                                                                           //    //                                //  
                double xfactor;                                                                                                                                            //    //                                //  
                // This is the main balancing equation                                                                                                                     //    //                                 //  
                //power = (int)( (KGYROSPEED              * gyroSpeed +                 // Deg/Sec from Gyro sensor                                                        //    //                                 //  
                //                testv[TempIndex]        * gyroAngle)  / ratioWheel +  // Deg from integral of gyro                                                       //    //                                  //  
                //                KPOS                       * motorPos +                  // From MotorRotaionCount of both motors                                        //    //                                  //  
                //                KDRIVE                     * motorControlDrive +         // To improve start/stop performance                                            //    //                                  //  
                //                KSPEED                     * motorSpeed              );  // Motor speed in Deg/Sec                                                       //    //                                   //  
                //KGYROANGLE = 7.5;      KGYROSPEED = 1.15;    KPOS = 0.07;     KSPEED = 0.1; KDRIVE = -0.02;   //    Segoway orginal                                      //    //                                   //                                   
                //KGYROANGLE2            KGYROSPEED2           KPOS2            KSPEED2=     KDRIVE2=                                                                      //    //                                   //                                                                  
                //   = KGYROANGLE;        =KGYROSPEED;           =KPOS;            KSPEED;          KDRIVE;                                                                //    //                                   //                                                    
                                                                                                                                                                           //    //                                   //                            
                //KGYROANGLE2 = 7.5;     KGYROSPEED2=1.15;     KPOS2=0.07;      KSPEED2=0.1; KDRIVE2=-0.02;   //    -8    3sec                                             //    //                                   //                                   
                //KGYROANGLE2 = 8;       KGYROSPEED2=1.15;     KPOS2=0.07;      KSPEED2=0.1; KDRIVE2=-0.02;   //    -7    7sec                                             //    //                                   //                                   
                //KGYROANGLE2 = 9;       KGYROSPEED2=1.15;     KPOS2=0.07;      KSPEED2=0.1; KDRIVE2=-0.02;   //    -6   14sec                                             //    //                                   //                                   
                //KGYROANGLE2 = 10;      KGYROSPEED2=1.15;     KPOS2=0.07;      KSPEED2=0.1; KDRIVE2=-0.02;   //    -5    7sec                                             //    //                                   //                                   
                //KGYROANGLE2 = 11;      KGYROSPEED2=1.15;     KPOS2=0.07;      KSPEED2=0.1; KDRIVE2=-0.02;   //    -4    6sec                                             //    //                                   //                                   
                //KGYROANGLE2 = 12;      KGYROSPEED2=1.15;     KPOS2=0.07;      KSPEED2=0.1; KDRIVE2=-0.02;   //    -3    8sec                                             //    //                                   //                                   
                //KGYROANGLE2 = 13;      KGYROSPEED2=1.15;     KPOS2=0.07;      KSPEED2=0.1; KDRIVE2=-0.02;   //    -2   12sec                                             //    //                                   //                                   
                //KGYROANGLE2 = 14;      KGYROSPEED2=1.15;     KPOS2=0.07;      KSPEED2=0.1; KDRIVE2=-0.02;   //    -1   12sec                                             //    //                                   //                                   
                //KGYROANGLE2 = 15;      KGYROSPEED2=1.15;     KPOS2=0.07;      KSPEED2=0.1; KDRIVE2=-0.02;   //     0   13sec                                             //    //                                   //                                   
                //KGYROANGLE2 = 20;      KGYROSPEED2=0;        KPOS2=0;         KSPEED2=0;   KDRIVE2=-0.02;   //     1    2sec                                             //    //                                   // 
                //KGYROANGLE2 = 15;      KGYROSPEED2=0;        KPOS2=0;         KSPEED2=0;   KDRIVE2=-0.02;   //     2    2sec                                             //    //                                   //
                //KGYROANGLE2 = 17;      KGYROSPEED2=0;        KPOS2=0;         KSPEED2=0;   KDRIVE2=-0.02;   //     3    2sec                                             //    //                                    //
                //KGYROANGLE2 = 18;      KGYROSPEED2=0;        KPOS2=0;         KSPEED2=0;   KDRIVE2=-0.02;   //     4    2sec                                             //    //                                    //
                //KGYROANGLE2 = 19;      KGYROSPEED2=0;        KPOS2=0;         KSPEED2=0;   KDRIVE2=-0.02;   //     5    2sec                                             //    //                                    //
                //KGYROANGLE2 = 19;      KGYROSPEED2=1;        KPOS2=0;         KSPEED2=0;   KDRIVE2=-0.02;   //     6    2sec                                             //    //                                    //
                //KGYROANGLE2 = 19;      KGYROSPEED2=2;        KPOS2=0;         KSPEED2=0;   KDRIVE2=-0.02;   //     7    2sec                                             //    //                                     //
                //KGYROANGLE2 = 20;      KGYROSPEED2=2;        KPOS2=0;         KSPEED2=0;   KDRIVE2=-0.02;   //     8    2sec                                             //    //                                     //
                //KGYROANGLE2 = 21;      KGYROSPEED2=2;        KPOS2=0;         KSPEED2=0;   KDRIVE2=-0.02;   //     9    2sec                                             //    //                                     //
                //KGYROANGLE2 = 22;      KGYROSPEED2=2;        KPOS2=0;         KSPEED2=0;   KDRIVE2=-0.02;   //    10    2sec                                             //    //                                     // 
                //KGYROANGLE2 = 23;      KGYROSPEED2=2;        KPOS2=0;         KSPEED2=0;   KDRIVE2=-0.02;   //    11    2sec                                             //    //                                     // 
                //KGYROANGLE2 = 24;      KGYROSPEED2=2;        KPOS2=0;         KSPEED2=0;   KDRIVE2=-0.02;   //    12    2sec                                             //    //                                     //
                //KGYROANGLE2 = 19;      KGYROSPEED2=1.5;      KPOS2=0.1;       KSPEED2=0.1; KDRIVE2=-0.02;   //    13   11sec Balances                                    //    //                                     //    
                //KGYROANGLE2 = 19;      KGYROSPEED2=1.4;      KPOS2=0.1;       KSPEED2=0.1; KDRIVE2=-0.02;   //    14    5sec Better than 13                              //    //                                     //  
                //KGYROANGLE2 = 18;      KGYROSPEED2=1.4;      KPOS2=0.1;       KSPEED2=0.1; KDRIVE2=-0.02;   //    15   11sec                                             //    //                                      //
                //KGYROANGLE2 = 18;      KGYROSPEED2=1.4;      KPOS2=0.2;       KSPEED2=0.1; KDRIVE2=-0.02;   //    16    8sec Best ever                                   //    //                                      //     
                //KGYROANGLE2 = 18;      KGYROSPEED2=1.4;      KPOS2=0.21;      KSPEED2=0.1; KDRIVE2=-0.02;   //    17    6sec Walks backwards                             //    //                                      //    
                //KGYROANGLE2 = 18;      KGYROSPEED2=1.4;      KPOS2=0.21;      KSPEED2=0.2; KDRIVE2=-0.02;   //    18    2sec Falls backwards quickly                     //    //                                      //     
                //KGYROANGLE2 = 18;      KGYROSPEED2=1.4;      KPOS2=0.22;      KSPEED2=0.1; KDRIVE2=-0.02;   //    19    6sec Falls backwards faster than 17, not as fast as 18 //                                      //                                   
                //KGYROANGLE2 = 18;      KGYROSPEED2=1.5;      KPOS2=0.21;      KSPEED2=0.1; KDRIVE2=-0.02;   //    20    6sec Walks backwards, Fell backwards, lasted longer than 18                                    //                                         
                //KGYROANGLE2 = 18;      KGYROSPEED2=1.4;      KPOS2=0.2;       KSPEED2=0.1; KDRIVE2=-0.02;   //    16    6sec Best ever, again, walks backwards           //    //                                      //                           
                //KGYROANGLE2 = 19;      KGYROSPEED2=1.4;      KPOS2=1.0;       KSPEED2=0.1; KDRIVE2=-0.02;   //    21    6sec Walks backwards, better than 16             //    //                                      //                               
                //KGYROANGLE2 = 20;      KGYROSPEED2=1.0;      KPOS2=1.0;       KSPEED2=0.1; KDRIVE2=-0.02;   //    22    1sec                                             //    //                                      // 
                //KGYROANGLE2 = 21;      KGYROSPEED2=0.5;      KPOS2=1.0;       KSPEED2=0.1; KDRIVE2=-0.02;   //    23    1sec Fell backwards quickly                      //    //                                      // 
                //KGYROANGLE2 = 22;      KGYROSPEED2=0.5;      KPOS2=1.0;       KSPEED2=0.1; KDRIVE2=-0.02;   //    24    1sec                                             //    //                                      // 
                //KGYROANGLE2 = 23;      KGYROSPEED2=0.5;      KPOS2=0.5;       KSPEED2=0.1; KDRIVE2=-0.02;   //    25    2sec Ran longer than 24, 3 sec//                 //    //                                      // 
                //KGYROANGLE2 = 24;      KGYROSPEED2=0.5;      KPOS2=0.5;       KSPEED2=0.1; KDRIVE2=-0.02;   //    26   3 sec                                             //    //                                      // 
                //KGYROANGLE2 = 23;      KGYROSPEED2=0.5;      KPOS2=0.5;       KSPEED2=0.1; KDRIVE2=-0.02;   //    27 3sec                                                //    //                                      // 
                KGYROANGLE2 = 18;      KGYROSPEED2=1.4;      KPOS2=0.2;       KSPEED2=0.1; KDRIVE2=-0.02;   //    16   7sec Best ever, again, walks backwards              //    //                                      //                  
                                                                                                                                                                           //    //                                      //                  
                power = (int)(                                                                                                                                             //    //                                      //                 
                               ( KGYROANGLE2                * gyroAngle         +                         // Deg/Sec from Gyro sensor                                      //    //                                      // 
                                 KGYROSPEED2                  * gyroSpeed               )  / ratioWheel +      // Deg from integral of gyro                                //    //                                       // 
                                 KPOS2                       * motorPos          +                          // From MotorRotaionCount of both motors                       //    //                                       // 
                                 KSPEED2                     * motorSpeed +                                 // To improve start/stop performance                           //    //                                       // 
                                 KDRIVE                     * motorControlDrive                          // Motor speed in Deg/Sec                                         //    //                                       //   
                                                                                 );                                                                                        //    //                                       //            
                                                                                                                                                                           //    //                                       //      
                                                                                                                                                                           //    //                                       //  
                if (Math.abs(power) < 100)                                                                                                                                 //    //                                        //    
                    tMotorPosOK = System.currentTimeMillis();                                                                                                              //    //                                          //   
                                                                                                                                                                           //    //                                          // 
                steerControl(power); // Movement control. Not used for balancing.                                                                                          //    //                                          // 
                                                                                                                                                                           //    //                                          // 
                // Apply the power values to the motors                                                                                                                    //    //                                          // 
                // NOTE: It would be easier/faster to use MotorPort.controlMotorById(), but it needs to be public.                                                         //    //                                           //  
                left_motor.setPower(Math.abs(powerLeft));                                                                                                                  //    //                                           // 
                right_motor.setPower(Math.abs(powerRight));                                                                                                                //    //                                           // 
                                                                                                                                                                           //    //                                           // 
                if(powerLeft > 0) left_motor.forward();                                                                                                                    //    //                                           //       
                else left_motor.backward();                                                                                                                                //    //                                           //         
                                                                                                                                                                           //    //                                           //       
                if(powerRight > 0) right_motor.forward();                                                                                                                  //    //                                           //  
                else right_motor.backward();                                                                                                                               //    //                                           // 
                                                                                                                                                                           //    //                                           // 
                try {Thread.sleep(WAIT_TIME);} catch (InterruptedException e) {}                                                                                           //    //                                           // 
                
                RunTimeMS = System.currentTimeMillis() - StartTime;                                                                                                        //    //                                           //                                                            
                RunTimeS  = RunTimeMS*1E-3;                                                                                                                                //    //                                           //                                                            
                                                                                                                                                                           //    //                                           //                        
                if (Debug)                                                                                                                                                 //    //                                           //               
                    {    //////////////////////////////////////////////////////////////////////////                                                                        //    //                                           // 
                    if (loopCount <= TestInterval)                                               //                                                                        //    //                                           //                 
                        {    ////////////////////////////////////////////////////////////////    //                                                                        //    //                                           //        
                        //System.out.println();                                            //    //                                                                        //    //                                           //                                
                        //System.out.println(loopCount);                                   //    //                                                                        //    //                                           //                    
                          TestIntervalA       [loopCount-1]  =  TestInterval          ;    //    //                                                                        //    //                                           //                   
                          loopCountA          [loopCount-1]  =  loopCount             ;    //    //                                                                        //    //                                           //             
                          gyroAngleA          [loopCount-1]  =  gyroAngle             ;    //    //                                                                        //    //                                           //                     
                          gyroSpeedA          [loopCount-1]  =  gyroSpeed             ;    //    //                                                                        //    //                                           //        
                          motorPosA           [loopCount-1]  =  motorPos              ;    //    //                                                                        //    //                                           //               
                          motorSpeedA         [loopCount-1]  =  motorSpeed            ;    //    //                                                                        //    //                                           //                 
                          gAngleGlobalA       [loopCount-1]  =  gAngleGlobal          ;    //    //                                                                        //    //                                           //                    
                          mrcDeltaA           [loopCount-1]  =  mrcDelta              ;    //    //                                                                        //    //                                           //                    
                          mrcDeltaP1A         [loopCount-1]  =  mrcDeltaP1            ;    //    //                                                                        //    //                                           //                            
                          mrcDeltaP2A         [loopCount-1]  =  mrcDeltaP2            ;    //    //                                                                        //    //                                           //                
                          mrcDeltaP3A         [loopCount-1]  =  mrcDeltaP3            ;    //    //                                                                        //    //                                           //                   
                          powerA              [loopCount-1]  =  power                 ;    //    //                                                                        //    //                                           //                     
                          powerLeftA          [loopCount-1]  =  powerLeft             ;    //    //                                                                        //    //                                           //                  
                          powerRightA         [loopCount-1]  =  powerRight            ;    //    //                                                                        //    //                                           //               
                          mrcLeftA            [loopCount-1]  =  mrcLeft               ;    //    //                                                                        //    //                                           //                   
                          mrcRightA           [loopCount-1]  =  mrcRight              ;    //    //                                                                        //    //                                           //                   
                          tCalcStartA         [loopCount-1]  =  tCalcStart            ;    //    //                                                                        //    //                                           //                 
                          tIntervalA          [loopCount-1]  =  tInterval             ;    //    //                                                                        //    //                                           //                    
                          tMotorPosOKA        [loopCount-1]  =  tMotorPosOK           ;    //    //                                                                        //    //                                           //                        
                          RunTimeMSA          [loopCount-1]  =  RunTimeMS             ;    //    //                                                                        //    //                                           //                 
                          RunTimeSA           [loopCount-1]  =  RunTimeS              ;    //    //                                                                        //    //                                           //                 
                          motorControlDriveA  [loopCount-1]  =  motorControlDrive     ;    //    //                                                                        //    //                                           //                 
                        }    ////    if (loopCount <= TestInterval)  ////////////////////////    //                                                                        //    //                                           //      
                                                                                                 //                                                                        //    //                                           //                        
                    }    ///////////////    if (Debug)    /////////////////////////////////////////                                                                        //    //                                           //                      
                                                                                                                                                                           //    //                                           //                 
                // Check if robot has fallen by detecting that motorPos is being limited                                                                                   //    //                                           // 
                // for an extended amount of time.                                                                                                                         //    //                                           // 
                if ((System.currentTimeMillis() - tMotorPosOK) > TIME_FALL_LIMIT) break;                                                                                   //    //                                           //   
                                                                                                                                                                           //    //                                           // 
                //System.out.printf("Loop Count is %10d    gyro angle is %10.5f\n", loopCount, gyroAngle);                                                                 //    //                                           //  
                //Delay.msDelay(8);                                                                                                                                        //    //                                           //                
                loopCount++;                                                                                                                                               //    //                                           //              
            } //////////////////////////////////////    while (Button.ESCAPE.isUp())    /////////////////////////////////////////////////////////////////////////////////////    //                                           // 
                                                                                                                                                                                //                                           //     
            loopCount--;                                                                                                                                                       //                                           //              
            AverageMS = RunTimeMS / loopCount;         //    1E6   ms                                                                                                         //                                           //                                                                                           
            AverageS  = RunTimeS  / loopCount;    //    1E6   ms                                                                                                             //                                           //                                                                                           
            System.out.println("Oops... I fell    ");                                                                                                                       //                                           //    
            System.out.println();                                                                                                                                          //                                           //               
            System.out.printf("After %10d iterations and %10.0f seconds the average loop time is %5.2f ms\n", loopCount, RunTimeS, AverageMS);                            //                                           //                                                      
                                                                                                                                                                         //                                           //               
            left_motor.flt();                                                                                                                                           //                                           //   
            right_motor.flt();                                                                                                                                         //                                           //                 
                                                                                                                                                                      //                                           //         
            //Sound.beepSequenceUp();                                                                                                                                //                                           //  
            //System.out.print("tInt ms:");                                                                                                                         //                                           //                       
            //System.out.println((int)tInterval*1000);                                                                                                             //                                           //      
            //System.out.println(tInterval*1000);                                                                                                                 //                                           //  
            //System.out.println((int)(tInterval*1000));                                                                                                         //                                           //  
            //System.out.printf("Cycle time is %3.1f ms\n", tInterval*1000);                                                                                     //                                           //  
            //System.out.printf("Iterations is %10d", loopCount-1);                                                                                              //                                           //          
            //System.out.println("Exiting the body");                                                                                                            //                                           //   
                                                                                                                                                                 //                                           //                     
            //bw.write(String.format("%5d,   %10d,   %10d,   %10.10f,   %1010f,\t%10.10f", loopCount, i, loopCountA[i], gyroAngleA[i], powerA[i], tIntervalA[i]));                                            //              
                                                                                                                                                                 //                                           //                    
            try                                                                                                                                                  //                                           //                      
                {    /////////////////////////////////////////////////////////////////////////////                                                               //                                           //                                                                                             
                //          loopCount,  i, j, loopCountA[j], KGYROANGLE2*gyroAngleA[j],  KGYROSPEED2*gyroSpeedA[j],   KPOS2*motorPosA[j] ,  KSPEED2*motorSpeedA[j], powerA[j], powerLeftA[j], tIntervalA[j], RunTimeMSA[j], RunTimeSA[j], motorControlDriveA[j]  ));                   
                bw.write("   loopCount, i, j, loopCountA[j],      gyroAngleA[j],              gyroSpeedA[j],              motorPosA[j],         motorSpeedA[j],     powerA[j], PowerLeftA[j], tIntervalA[j], RunTimeMSA[j], RunTimeSA[j], motorControlDriveA[j]  ");                            
                bw.newLine();                                                                   //                                                               //                                           //                                                                        
                }    /////////////////////////////////////////////////////////////////////////////                                                               //                                           //                                                                                             
            catch (IOException e1)                                                                                                                               //                                           //                          
                {    ///////////////////////////////////                                                                                                         //                                           //                 
                // TODO Auto-generated catch block    //                                                                                                         //                                           //                   
                e1.printStackTrace();                 //                                                                                                         //                                           //                    
                }    ///////////////////////////////////                                                                                                         //                                           //                                                                                        
                                                                                                                                                                 //                                           //                             
            for (int i=1; i<=loopCount; i++)                                                                                                                     //                                           //                                                 
                {    ///////////////////////////////////////////////////////////////////////////////////                                                         //                                           //          
                int j = i-1;                                                                          //                                                         //                                           //                                                       
                if (i <= loopCount)                                                                   //                                                         //                                           //                                                                                              
                    {    /////////////////////////////////////////////////////////////////////////    //                                                         //                                           //                                                               
                    //System.out.println(dtA[i]);                                               //    //                                                         //                                           //   
                    //Total = Total + dtA[i]*1000;                                              //    //                                                         //                                           //               
                    //Average = Total/(i+1);                                                    //    //                                                         //                                           //                         
                    //System.out.printf("%5d:%5d %10.2f ms   \tAverage loop time is %5.2f ms\n", i+1, loopCount, dtA[i]*1000, Average);                          //                                           //         
                    //System.out.printf("Power %3.0f\t\tmSpd %3.0f\t\tmPos %.0f\t\tgSpd %.0f\tgAng = %.0f\n", pwr, mSpd, mPos, gSpd, gAng);                      //                                           //    
                    try                                                                         //    //                                                         //                                           //                
                        {    ///////////////////////////////////////////////////////////////    //    //                                                         //                                           //                                                      
                        //bw.write(String.format("Power %3.0f\t\tmSpd %3.0f\t\tmPos %.0f\t\tgSpd %.0f\tgAng = %.0f\n", pwrA[loopCount], mSpdA[loopCount], mPosA[loopCount], gSpdA[loopCount], gAngA[loopCount]));                                                
                        //Logging.log("%5d,   \t\t%10.10f,   \t\t%10.10f,   \t\t\t%10.10f,   \t\t\t%10d,\t\t\t%10.10f\n", cLoop, 10*gyroAngle, testv[TempIndex]*gyroAngle, KGYROSPEED*gyroSpeed, powerLeft, 1000*tInterval);                                                                                                                                                  
                        //                                                                                                                                loopCount  i  j  loopCountA[j]  gyroAngleA[j]     gyroSpeedA[j]  motorPosA[j]  motorSpeedA[j]  powerA[j]    tIntervalA[j]    PowerLeftA[j] RunTimeMSA[j]   RunTimeSA[j] motorControlDriveA[j]");              
                        bw.write(String.format("   %5d,   %10d,  %10d,      %10d,              %10.10f,                        %10.10f,                %10.10f,               %10.10f,            %10.10f ,     %10.10f,       %10.10f,      %10.10f,        %10.10f,      %10.10f     ",                                                  
                                                loopCount,  i,     j,     loopCountA[j],   KGYROANGLE2*gyroAngleA[j],  KGYROSPEED2*gyroSpeedA[j],   KPOS2*motorPosA[j] ,  KSPEED2*motorSpeedA[j], powerA[j], powerLeftA[j], tIntervalA[j], RunTimeMSA[j], RunTimeSA[j], motorControlDriveA[j]));                               //             
                        bw.newLine();                                                     //    //    //                                                         //                                           //                                                          
                        }    ///////////////////////////////////////////////////////////////    //    //                                                         //                                           //                                                         
                    catch (IOException e)                                                       //    //                                                         //                                           //                                                         
                        {    ///////////////////////////////////                                //    //                                                         //                                           //                                                                               
                        // TODO Auto-generated catch block    //                                //    //                                                         //                                           //                                                            
                        e.printStackTrace();                  //                                //    //                                                         //                                           //                                                                
                        }      /////////////////////////////////                                //    //                                                         //                                           //                                                                     
                                                                                                //    //                                                         //                                           //                           
                    }    ///////////////////////    if (j <= loopCount)    ///////////////////////    //                                                         //                                          //                       
                                                                                                      //                                                         //                                          //                 
            }    ////////////////////////////    for (int j=1; j<=loopCount; j++)    ///////////////////                                                         //                                          //                  
                //Logging.log("     Loop                             Angle                       AngleMultiplyer                     SpeedMultiplier                                Power              Interval");           
                                                                                                                                                                 //                                          //                         
            return;                                                                                                                                              //                                          // 
            } /////////////////////////////////////    END OF BALANCING THREAD CODE    ////////////////////////////////////////////////////////////////////////////                                          //                                                                                     
                                                                                                                                                                                                             // 
        /**                                                                                                                                                                                                 // 
         * This method allows the robot to move forward/backward and make in-spot rotations as                                                                                                              //     
         * well as arcs by varying the power to each wheel. This method does not actually                                                                                                                  //    
         * apply direct power to the wheels. Control is filtered through to each wheel, allowing the robot to                                                                                              //  
         * drive forward/backward and make turns. Higher values are faster. Negative values cause the wheel                                                                                                // 
         * to rotate backwards. Values between -200 and 200 are good. If values are too high it can make the                                                                                               // 
         * robot balance unstable.                                                                                                                                                                        // 
         *                                                                                                                                                                                               // 
         * @param left_wheel  The relative control power to the left  wheel. -200 to 200 are good numbers.                                                                                               //        
         * @param right_wheel The relative control power to the right wheel. -200 to 200 are good numbers.                                                                                              // 
         */                                                                                                                                                                                             // 
                                                                                                                                                                                                       //   
        public void wheelDriver(int left_wheel, int right_wheel)                                                                                                                                       // 
            {    //////////////////////////////////////////////////////////////////////////////                                                                                                        //                                                                                                           
            // Set control Drive and Steer.  Both these values are in motor degree/second    //                                                                                                       //                                                                                             
            motorControlDrive = (left_wheel + right_wheel) * CONTROL_SPEED / 200.0;          //                                                                                                       //                                                                            
            motorControlSteer = (left_wheel - right_wheel) * CONTROL_SPEED / 200.0;          //                                                                                                       //                                                                        
            }          /////////////////////////////////////////////////////////////////////////                                                                                                       //                                                        
                                                                                                                                                                                                      //                                            
        public void run()                                                                                                                                                                             //                                                            
            {    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////                                                                     //                                                   
            double[] Duration = new double[10];                                                                                //                                                                     //                                      
            double Searchstart = 0;                                                                                            //                                                                    //                                                             
            //anArray = new int[10];                                                                                           //                                                                    //                                                     
            //Duration = new int[10];                                                                                          //                                                                    //                                                    
                                                                                                                               //                                                                   //                                            
            System.out.println("System.nanoTime()");                                                                           //                                                                  //                                            
            now1 = System.nanoTime();  //                                                                                      //                                                                  //                                             
            System.out.println(now1);                                                                                          //                                                                  //                                   
            //Delay.msDelay(60);                                                                                               //                                                                 //                                     
            now2 = System.nanoTime();  //                                                                                      //                                                                 //                           
            System.out.println(now2);                                                                                          //                                                                 //                                
            System.out.println((now2-now1)/1000000);                                                                           //                                                                 //                                  
            System.out.println((now2-now1)*1E-6);                                                                              //                                                                // 
            System.out.println(" ms");                                                                                         //                                                                //                                       
            System.out.printf("Delay.msDelay(60) time is %3.0f ms\n", (now2-now1)/1000000.0);                                  //                                                                //                                  
            System.out.println();                                                                                              //                                                                //                 
                                                                                                                               //                                                                //                     
            System.out.println("System.currentTimeMillis()");                                                                  //                                                                //      
            now1 = System.currentTimeMillis();                                                                                 //                                                                //                   
            System.out.println(now1);                                                                                          //                                                                //               
            Delay.msDelay(  60);                                                                                               //                                                               //   
            now2 = System.currentTimeMillis();                                                                                 //                                                               //                 
            System.out.println(now2);                                                                                          //                                                               //          
            System.out.println(now2-now1);                                                                                     //                                                              //        
            System.out.println(" ms");                                                                                         //                                                              //          
            System.out.printf("Delay.msDelay(60) time is %3.0f ms\n", 1.0*(now2-now1));                                        //                                                             //    
            System.out.println();                                                                                              //                                                            // 
                                                                                                                               //                                                            //        
            System.out.println("6/17/2018");                                                                                   //                                                            //   
                                                                                                                               //                                                            //     
            //double gyroangle;      double gyrospeed;      double kpos;         double kspeed;                                //                                                            //  
            KGYROANGLE2 = 9;     KGYROSPEED2=0;      KPOS2=0;         KSPEED2=0;                                               //                                                            //     
            //KGYROSPEED2 = 0;                                                                                                 //                                                           // 
            //KPOS2 = 0.0;                                                                                                     //                                                           // 
            //KSPEED2 = 0;                                                                                                     //                                                           // 
                                                                                                                               //                                                           //         
            for (TempIndex=0; TempIndex<1; TempIndex++)                                                                        //                                                           //             
                {    //////////////////////////////////////////////////////////////////////////////////////////////////////    //                                                           //                                                            
                System.out.printf("Run %5d\t%5d percent\n", TempIndex+1, (95+TempIndex));                                //    //                                                           //                                                      
                //System.out.println(KGYROANGLE);                                                                        //    //                                                          //    
                //testv[7] = 1;                                                                                          //    //                                                          //                                                                           
                //testv[8] = 3;                                                                                          //    //                                                          //                                                     
                //testv[9] = KGYROANGLE;                                                                                 //    //                                                          //                                                 
                //testv[TempIndex] = (95.0+TempIndex)/100.0*KGYROANGLE;                                                  //    //                                                          //                         
                testv[TempIndex] = KGYROANGLE;                                                                           //    //                                                          //                                                
                //System.out.println(TempIndex);                                                                         //    //                                                          // 
                //System.out.println(testv[TempIndex]);                                                                  //    //                                                         // 
                Searchstart = System.currentTimeMillis();                                                                //    //                                                         //                                      
                body();                                                                                                  //    //                                                         //                           
                Duration[TempIndex] = System.currentTimeMillis() - Searchstart;                                          //    //                                                         //                    
                //System.out.println("Out of loop");                                                                     //    //                                                         //                                                      
                //System.out.printf("Duration of run %5d is %6.0f seconds !\n", TempIndex+1, Duration[TempIndex]/1000);  //    //                                                         //   
                System.out.println("");                                                                                  //    //                                                         //               
                }    //////////////////////////////////////////////////////////////////////////////////////////////////////    //                                                         //                                                                         
                                                                                                                               //                                                        // 
            // Finding the largest element                                                                                     //                                                        //                       
            int index = 0;                                                                                                     //                                                       // 
            double max = 0;                                                                                                    //                                                       //  
            for (int j=0; j<Duration.length; j++)                                                                              //                                                      // 
                {    /////////////////////////////                                                                             //                                                     //   
                if (Duration[j] > max)          //                                                                             //                                                     //            
                    {    ///////////////////    //                                                                             //                                                     //          
                    max = Duration[j];    //    //                                                                             //                                                    // 
                    index = j;            //    //                                                                             //                                                   // 
                    }    ///////////////////    //                                                                             //                                                   //  
                                                //                                                                             //                                                   //       
                }    /////////////////////////////                                                                             //                                                   //                                           
            now1 = System.nanoTime();                                               //     6000000                             //                                                   //            
            //System.out.printf("The Best setting of KGYROANGLE is %5.1f !", testv[index]);                                    //                                                  // 
            try                                                                                                                //                                                  //            
            {    ////////////                                                                                                  //                                                  //       
            if(bw!=null)   //                                                                                                  //                                                  //       
            bw.close();    //                                                                                                  //                                                  //       
            }    ////////////                                                                                                  //                                                  //       
        catch(Exception ex)                                                                                                    //                                                  //       
            {    //////////////////////////////////////////////////////////////                                                //                                                  //       
            System.out.println("Error in closing the BufferedWriter"+ex);    //                                                //                                                  //       
            }    //////////////////////////////////////////////////////////////                                                //                                                  //       
                                                                                                                               //                                                  //       
        Sound.beepSequenceUp();                                                                                                //                                                  // 
            }    ////////////////////////////////// public void run()  //////////////////////////////////////////////////////////                                                  // 
                                                                                                                                                                                   //                                                            
}    ////////////////////////////////    public class Segoway extends Thread    /////////////////////////////////////////////////////////////////////////////////////////////////////                  

