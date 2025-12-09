// quadrature detection

//float SSsum;
//float CCsum;
float[] calcSSandCCsum(ArrayList<Accel> al) {
  // convolution
  float SSsum = 0;
  float CCsum = 0;
  for (int i = 0; i < DATA_LEN; i++) {
    SSsum += (SS[i] * al.get(i).az);
    CCsum += (CC[i] * al.get(i).az);
  }
  return new float[]{SSsum, CCsum};
}

float detectPhase(float SSsum, float CCsum) {
  return atan2((float)CCsum, (float)SSsum);
}
float detectAmp(float SSsum, float CCsum) {
  return sqrt((SSsum*SSsum + CCsum*CCsum)) * 2.0 / DATA_LEN;
}
