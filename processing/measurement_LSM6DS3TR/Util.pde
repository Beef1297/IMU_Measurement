
float calcArrayMean(int[] a) {
  float mean = 0;
  for (int i = 0; i < a.length; i++) {
    mean += (float)a[i] / (float)a.length;
  }
  return mean;
}
