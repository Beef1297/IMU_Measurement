
static class Accel {

  float ax, ay, az;
  
  static float convertToG(int _a) {
    float _af = _a * (2.0 * FULL_SCALE) / (1 << 16);
    return _af;
  }

  static float convertToMS2(float _ag) {
    return _ag * 9.8;
  }

  Accel(int _ax, int _ay, int _az) {
    ax = convertToMS2(convertToG(_ax));
    ay = convertToMS2(convertToG(_ay));
    az = convertToMS2(convertToG(_az));
  }
  
  Accel(float _ax, float _ay, float _az) {
    ax = _ax;
    ay = _ay;
    az = _az;
  }
  
}

float[] convertRawAccel(int[] rawData) {
  float[] accs = new float[3];
  // return raw_values * (2 * FULL_SCALE) / (1 << 16)
  accs[0] = rawData[0] * (2 * FULL_SCALE) / (1 << 16);
  accs[1] = rawData[1] * (2 * FULL_SCALE) / (1 << 16);
  accs[2] = rawData[2] * (2 * FULL_SCALE) / (1 << 16);
  return accs;
}

Accel calcAverageAccel(ArrayList<Accel> al) {
  float _ax = 0, _ay = 0, _az = 0;
  float N = al.size();
  for (int i = 0; i < al.size() - 1; i++) {
    _ax += al.get(i).ax / N;
    _ay += al.get(i).ay / N;
    _az += al.get(i).az / N;
  }

  Accel aa = new Accel(_ax, _ay, _az);
  return aa;
}
