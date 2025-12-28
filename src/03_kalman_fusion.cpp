/**
 * File: 03_kalman_fusion.cpp
 * Author: Luiz Rosa
 * Description: Real-time Sensor Fusion (Ultrasonic + IMU) using Kalman Filter.
 * Dependencies: Eigen 3.x
 * Compile: g++ -I /usr/include/eigen3 03_kalman_fusion.cpp -o kalman_fusion
 */ 
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace std;

class KalmanFilterRaw {
private:
    // State: [0]=position, [1]=velocity
    double x[2]; 
    
    // Covariance P (2x2)
    double P[2][2];
    
    // Physics constants
    double dt;
    double process_noise; // Q
    double meas_noise_R;  // R

public:
    KalmanFilterRaw(double _dt, double _meas_std) {
        dt = _dt;
        meas_noise_R = _meas_std * _meas_std;
        
        // Initial State
        x[0] = 0.0; 
        x[1] = 0.0;
        
        // Initial P (Identity * 100)
        P[0][0] = 100.0; P[0][1] = 0.0;
        P[1][0] = 0.0;   P[1][1] = 100.0;
        
        // Process Noise (Approximation)
        process_noise = 0.01;
    }

    void init_state(double p, double v) {
        x[0] = p;
        x[1] = v;
    }

    void predict(double accel_input) {
        // --- 1. PREDICT STATE ---
        // x_new = F * x + B * u
        // p = p + v*dt + 0.5*a*dt^2
        // v = v + a*dt
        double p_new = x[0] + x[1]*dt + 0.5*accel_input*dt*dt;
        double v_new = x[1] + accel_input*dt;
        
        x[0] = p_new;
        x[1] = v_new;

        // --- 2. PREDICT COVARIANCE ---
        // P = F P F.T + Q
        // F is [[1, dt], [0, 1]]
        // We expand the matrix multiplication manually for performance
        
        double p00 = P[0][0];
        double p01 = P[0][1];
        double p10 = P[1][0];
        double p11 = P[1][1];

        // Temp calculation for F * P
        double fp00 = p00 + dt * p10;
        double fp01 = p01 + dt * p11;
        double fp10 = p10;
        double fp11 = p11;

        // Final P calculation (multiplying by F transpose) + Q
        // Q is added to diagonals
        P[0][0] = (fp00 * 1.0) + (fp01 * dt) + process_noise;
        P[0][1] = (fp00 * 0.0) + (fp01 * 1.0);
        P[1][0] = (fp10 * 1.0) + (fp11 * dt);
        P[1][1] = (fp10 * 0.0) + (fp11 * 1.0) + process_noise;
    }

    void update(double z_meas) {
        // H = [1, 0] (We only measure position)
        
        // --- 1. RESIDUAL (y) ---
        // y = z - Hx
        double y = z_meas - x[0];

        // --- 2. RESIDUAL COVARIANCE (S) ---
        // S = H P H.T + R
        // Since H is [1, 0], HPH.T is just the top-left element of P
        double S = P[0][0] + meas_noise_R;

        // --- 3. KALMAN GAIN (K) ---
        // K = P H.T * S^-1
        double K0 = P[0][0] / S;
        double K1 = P[1][0] / S;

        // --- 4. UPDATE STATE ---
        x[0] = x[0] + K0 * y;
        x[1] = x[1] + K1 * y;

        // --- 5. UPDATE COVARIANCE ---
        // P = (I - KH) P
        // Temp storage for new P
        double new_P00 = (1.0 - K0) * P[0][0];
        double new_P01 = (1.0 - K0) * P[0][1];
        double new_P10 = -K1 * P[0][0] + P[1][0]; // (0 - K1*1)*P00 + 1*P10
        double new_P11 = -K1 * P[0][1] + P[1][1];
        
        P[0][0] = new_P00;
        P[0][1] = new_P01;
        P[1][0] = new_P10;
        P[1][1] = new_P11;
    }

    double get_position() { return x[0]; }
    double get_velocity() { return x[1]; }
};

int main() {
    // Run Configuration
    double dt = 0.1;
    KalmanFilterRaw kf(dt, 2.0); // 2.0m std deviation for sensor
    kf.init_state(40.0, -10.0);
    
    cout << "--- Starting Embedded ADAS Simulation (No External Libs) ---" << endl;
    cout << "Time,Raw_Dist,Accel,Filt_Dist,Filt_Vel,TTC" << endl;

    double current_pos = 40.0;
    double current_vel = -10.0;
    
    // Simulate 5 seconds
    for(int i=0; i<50; i++) {
        double t = i * dt;
        
        // Scenario: Brake at t > 2.0
        double true_acc = (t > 2.0) ? 3.0 : 0.0;
        
        // Physical update (Truth)
        current_pos += current_vel * dt + 0.5 * true_acc * dt * dt;
        current_vel += true_acc * dt;
        
        // Sensor Noise Generation
        double noise = ((rand() % 100) / 25.0) - 2.0; 
        double z_sensor = current_pos + noise;

        // --- ALGORITHM STEPS ---
        kf.predict(true_acc); // Fusion with IMU
        kf.update(z_sensor);  // Correction with Ultrasonic
        
        // TTC Logic
        double v_est = kf.get_velocity();
        double p_est = kf.get_position();
        double ttc = (v_est < -0.1) ? abs(p_est / v_est) : 999.0;
        
        // Output CSV format
        cout << fixed << setprecision(2) 
             << t << "," << z_sensor << "," << true_acc << "," 
             << p_est << "," << v_est << "," << ttc << endl;
    }
    return 0;
}
