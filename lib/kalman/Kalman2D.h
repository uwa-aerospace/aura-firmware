#ifndef KALMAN2D_H
#define KALMAN2D_H

// For baro only
struct Kalman2D {
  // State
  float x[2];        // [h, h_dot]
  float P[2][2];     // Covariance
  float Q[2][2];     // Process noise
  float R;           // Measurement noise (altitude)
  float dt;          // Time step

  void init(float init_h, float init_hdot,
            float var_h, float var_hdot,
            float accel_noise_sigma, float meas_noise_var,
            float dt_) 
  {
    x[0] = init_h;      // initial altitude
    x[1] = init_hdot;   // initial vertical velocity
    // initial covariance
    P[0][0] = var_h;    P[0][1] = 0;
    P[1][0] = 0;        P[1][1] = var_hdot;
    // process noise Q
    dt = dt_;
    float dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt3*dt;
    float q = accel_noise_sigma*accel_noise_sigma;
    Q[0][0] = q * dt4/4.0;
    Q[0][1] = q * dt3/2.0;
    Q[1][0] = q * dt3/2.0;
    Q[1][1] = q * dt2;
    // measurement noise
    R = meas_noise_var;
  }

  void predict() {
    // State transition F = [1 dt; 0 1]
    float x0 = x[0] + dt * x[1];
    float x1 = x[1];
    // Predict state
    x[0] = x0;
    x[1] = x1;
    // Predict covariance: P = F*P*F^T + Q
    float P00 = P[0][0] + dt*(P[1][0] + P[0][1]) + dt*dt*P[1][1] + Q[0][0];
    float P01 = P[0][1] + dt*P[1][1] + Q[0][1];
    float P10 = P[1][0] + dt*P[1][1] + Q[1][0];
    float P11 = P[1][1] + Q[1][1];
    P[0][0] = P00;  P[0][1] = P01;
    P[1][0] = P10;  P[1][1] = P11;
  }

  void update(float h_meas) {
    // H = [1 0], so innovation y = h_meas - x[0]
    float y = h_meas - x[0];
    // S = H*P*H^T + R = P[0][0] + R
    float S = P[0][0] + R;
    // K = P * H^T * S^{-1} => K = [P00/S; P10/S]
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;
    // Update state
    x[0] += K0 * y;
    x[1] += K1 * y;
    // Update covariance: P = (I - K*H) * P
    float P00 = (1 - K0)*P[0][0];
    float P01 = (1 - K0)*P[0][1];
    float P10 =   -K1 * P[0][0] + P[1][0];
    float P11 =   -K1 * P[0][1] + P[1][1];
    P[0][0] = P00;  P[0][1] = P01;
    P[1][0] = P10;  P[1][1] = P11;
  }
};

#endif