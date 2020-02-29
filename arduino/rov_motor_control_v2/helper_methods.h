// the parameter is in milliseconds
float calculate_freq(int runtime) {
  return 1.0 / (0.000001 * runtime);
}

float reverse_motor(float in) {
  return 1500.0 - (in - 1500.0);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
