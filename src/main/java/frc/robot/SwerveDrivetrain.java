// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;

/** Represents a swerve drive style drivetrain. */
class SwerveVehicleClass
{
    // constants
    double D2R = Math.PI/180.0 ;  // conversions between degrees and radians
    double R2D = 180.0/Math.PI ;
    
    // values that never change
    double wheel_diameter ; // meters
    double[][] origin_pt = new double[4][2] ; // x,y coord of module point in vehicle coord sys
                                          // module 0 is in +x,+y quadrant
                                          // module 1 is in -x,+y quadrant
                                          // module 2 is in -x,-y quadrant
                                          // module 3 is in +x,-y quadrant
    // vehicle state values
    double ang_rad  ; // angle from xF to xV (will come from IMU sensor)
    double[] vel_pt0_m_per_sec = new double[2] ; // desired linear velocity of the vehicle origin point, meters/sec
    double ang_vel_deg_per_sec ;  // desired angular velocity of the vehicle chassis, deg/sec
    
    // These are the current values for the four modules.
    double[] current_steering_angles_deg = new double[4] ;
    double[] current_wheel_angular_vel_deg_per_sec = new double[4] ;
    
    // These values are updated when calc_swerve_steering_and_speed() is called.
    // They represent the steering angles and positive rotation speeds for each
    // of the four modules.
    double[] calculated_steering_angles_deg = new double[4] ;
    double[] calculated_wheel_angular_vel_deg_per_sec = new double[4] ;
    
    // These are updated when recommend_swerve_steering_and_speed() is called.
    // This decides whether the calculated values are best or, based on the
    // current steering and wheel speed values, the steering angle should be
    // pointed 180 degrees in the opposite direction and the wheel speed made
    // negative.
    double[] recommended_steering_angles_deg = new double[4] ;
    double[] recommended_wheel_angular_vel_deg_per_sec = new double[4] ;

    // methods
    void assign_veh_values_for_square_vehicle(double d, double wheel_d)
    {
        // set the wheel diameter
        this.wheel_diameter = wheel_d ;
        
        // define the locations of the four swerve modules in the xv, yv coord sys
        
        this.origin_pt[0][0] =  d ;   // module 0 is in the +x,+y quadrant
        this.origin_pt[0][1] =  d ;
       
        this.origin_pt[1][0] = -d ;   // module 1 is in the -x,+y quadrant
        this.origin_pt[1][1] =  d ;
        
        this.origin_pt[2][0] = -d ;   // module 2 is in the -x,-y quadrant
        this.origin_pt[2][1] = -d ;
  
        this.origin_pt[3][0] =  d ;   // module 3 is in the +x,-y quadrant
        this.origin_pt[3][1] = -d ;
    }
    
    void initialize_steering_angles_and_wheel_speeds(double[] steering_ang_deg,
                                          double[] wheel_speeds_deg_per_sec)
    {
        int i ;
        for (i=0 ; i<4 ; ++i)
        {
            this.current_steering_angles_deg[i] = steering_ang_deg[i] ;
            this.current_wheel_angular_vel_deg_per_sec[i] = wheel_speeds_deg_per_sec[i] ;
        }
    }
    
    void calc_swerve_steering_and_speed()
        // This method uses the current values for:
        //        this.ang_rad,
        //        this.vel_pt0_m_per_sec
        //        this.ang_vel_deg_per_sec
        // and calculates and updates the values for:
        //        this.desired_steering_angles_deg  (a length 4 array of doubles)
        //        this.desired_wheel_angular_vel_deg_per_sec (a length 4 array of doubles)
    {
        int i ;
        double sv, cv ;
    
        cv = Math.cos(this.ang_rad) ;
        sv = Math.sin(this.ang_rad) ;
        
        // calculate the vehicle velocity in the vehicle coord sys
        double[] vO_in_veh = new double [2] ;
        vO_in_veh[0] =  cv * this.vel_pt0_m_per_sec[0] + sv * this.vel_pt0_m_per_sec[1] ;
        vO_in_veh[1] = -sv * this.vel_pt0_m_per_sec[0] + cv * this.vel_pt0_m_per_sec[1] ;
        
        double[] v_i = new double[2] ;
        double speed ;
        
        for (i=0 ; i<4 ; ++i)
        {
            v_i[0] = vO_in_veh[0] - this.ang_vel_deg_per_sec*this.D2R * this.origin_pt[i][1] ;
            v_i[1] = vO_in_veh[1] + this.ang_vel_deg_per_sec*this.D2R * this.origin_pt[i][0] ;
        
            speed = Math.sqrt(v_i[0]*v_i[0] + v_i[1]*v_i[1]) ;
            this.calculated_steering_angles_deg[i] = Math.atan2(v_i[1], v_i[0]) * this.R2D ;
            this.calculated_wheel_angular_vel_deg_per_sec[i] = this.R2D* speed / (this.wheel_diameter/2.0) ;       
        }    
    }
    
    void recommend_swerve_steering_and_speed()
    {
        // Compare the calculated steering angles with the current steering angles
        // and make a recommendation.  Maybe its better to steer 180 degrees opposite
        // from the recommended value and then spin the wheel in the negative direction.

        int i ;
        double ang_diff_deg ;
        
        for (i=0 ; i<4 ; ++i)
        {
            ang_diff_deg = this.calculated_steering_angles_deg[i] 
                         - this.current_steering_angles_deg[i] ;
            
            // bound the difference between -180 and +180 degrees
            while (ang_diff_deg < 180.0)
                ang_diff_deg += 360.0 ;
            while (ang_diff_deg > 180.0)
                ang_diff_deg -= 360.0 ;

            if (Math.abs(ang_diff_deg) < 90.0)
            {
                // Don't switch direction from calculated.
                this.recommended_steering_angles_deg[i] =
                       this.current_steering_angles_deg[i] + ang_diff_deg ;
                this.recommended_wheel_angular_vel_deg_per_sec[i] = 
                       this.calculated_wheel_angular_vel_deg_per_sec[i] ;
            }
            else
            {
                // Will add 180 to calculated steering angle and
                // rotate the wheel in the negative direction.
                this.recommended_steering_angles_deg[i] =
                       this.current_steering_angles_deg[i] + ang_diff_deg - 180.0 ;  // or +180.0
                this.recommended_wheel_angular_vel_deg_per_sec[i] = 
                     - this.calculated_wheel_angular_vel_deg_per_sec[i] ;
                while(this.recommended_steering_angles_deg[i] - this.current_steering_angles_deg[i] > 180.0)
                    this.recommended_steering_angles_deg[i] -= 360.0 ;
                while(this.recommended_steering_angles_deg[i] - this.current_steering_angles_deg[i] < -180.0)
                    this.recommended_steering_angles_deg[i] += 360.0 ;
            }
        }
        
    }
    
    void print_results()
    {
        int i ;
        System.out.println("------------------") ;
        System.out.println("\nVehicle Velocity Inputs (from joystick input):") ;
        System.out.printf("\tvelocity of point 0 = %.4f, %.4f m/sec\n",
                           this.vel_pt0_m_per_sec[0], this.vel_pt0_m_per_sec[1]) ;
        System.out.printf("\tangular velocity = %.2f deg/sec\n", this.ang_vel_deg_per_sec) ;
        System.out.printf("vehicle ang = %.2f degrees (from IMU sensor)\n\n", this.ang_rad*this.R2D) ;
        
        System.out.println("\t\t\t\tmodule 0\tmodule 1\tmodule 2\tmodule3") ;
        System.out.printf("current steering ang") ;
        for (i=0 ; i<4 ; ++i)
        {
            System.out.printf("\t\t%.2f", this.current_steering_angles_deg[i]) ;  
        }
        System.out.printf("\tdeg\n") ;
        
        System.out.printf("calculated steering ang") ;
        for (i=0 ; i<4 ; ++i)
        {
            System.out.printf("\t\t%.2f", this.calculated_steering_angles_deg[i]) ;  
        }
        System.out.printf("\tdeg\n") ;
        
        System.out.printf("recommend steering ang") ;
        for (i=0 ; i<4 ; ++i)
        {
            System.out.printf("\t\t%.2f", this.recommended_steering_angles_deg[i]) ;  
        }
        System.out.printf("\tdeg\n\n") ;
        
        System.out.printf("current wheel speed") ;
        for (i=0 ; i<4 ; ++i)
        {
            System.out.printf("\t\t%.1f", this.current_wheel_angular_vel_deg_per_sec[i]) ;  
        }
        System.out.printf("\tdeg/sec\n") ;
        
        System.out.printf("calculated wheel speed") ;
        for (i=0 ; i<4 ; ++i)
        {
            System.out.printf("\t\t%.1f", this.calculated_wheel_angular_vel_deg_per_sec[i]) ;  
        }
        System.out.printf("\tdeg/sec\n") ;
        
        System.out.printf("recommend wheel speed") ;
        for (i=0 ; i<4 ; ++i)
        {
            System.out.printf("\t\t%.1f", this.recommended_wheel_angular_vel_deg_per_sec[i]) ;  
        }
        System.out.printf("\tdeg/sec\n") ;

    }
}

class main {
    public static void main( String args[] ) {
        
        SwerveVehicleClass veh = new SwerveVehicleClass();
        veh.assign_veh_values_for_square_vehicle(0.3302, 0.1016) ;
        
        double[] initial_steering_deg = {42.5, 33.6, 18.1, 21.9} ;
        double[] initial_wheel_speed_deg_per_sec = {1.56, 11.3, 4.2, 10.4} ;
        veh.initialize_steering_angles_and_wheel_speeds(initial_steering_deg,
                                            initial_wheel_speed_deg_per_sec) ;
        
        // Example Case 1:
        // set the current vehicle angle
        veh.ang_rad = 30.0 * veh.D2R ;
        
        // set the desired velocity (linear and angular for the vehicle)
        veh.vel_pt0_m_per_sec[0] =  0.5 ;
        veh.vel_pt0_m_per_sec[1] =  1.1 ;
        veh.ang_vel_deg_per_sec  = 20.0 ;
        
        veh.calc_swerve_steering_and_speed() ;
        veh.recommend_swerve_steering_and_speed() ;
                
        veh.print_results() ;  // print results of case 1     
        
        // Example Case 2: (translation case)
        // set the current vehicle angle
        veh.ang_rad = 30.0 * veh.D2R ;
        
        // set the desired velocity (linear and angular for the vehicle)
        veh.vel_pt0_m_per_sec[0] = 1.1 * Math.cos(60.0*veh.D2R) ;
        veh.vel_pt0_m_per_sec[1] = 1.1 * Math.sin(60.0*veh.D2R) ;
        veh.ang_vel_deg_per_sec  = 0.0 ;
        
        veh.calc_swerve_steering_and_speed() ;
        veh.recommend_swerve_steering_and_speed() ;
        
        veh.print_results() ;  // print results of case 2 
        
        // Example Case 3: (recommend reverse wheel rotation based on current steering)
        // set the current vehicle angle
        veh.ang_rad = 30.0 * veh.D2R ;
        
        // set the desired velocity (linear and angular for the vehicle)
        veh.vel_pt0_m_per_sec[0] = -1.1 * Math.cos(60.0*veh.D2R) ;
        veh.vel_pt0_m_per_sec[1] = -1.1 * Math.sin(60.0*veh.D2R) ;
        veh.ang_vel_deg_per_sec  = -25.0 ;
        
        veh.calc_swerve_steering_and_speed() ;
        veh.recommend_swerve_steering_and_speed() ;
        
        veh.print_results() ;  // print results of case 3 
    }
}