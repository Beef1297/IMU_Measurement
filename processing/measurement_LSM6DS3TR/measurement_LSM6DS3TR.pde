// 2023/12/26 keigo ushiyama

import java.lang.*;
import java.util.*;
import processing.serial.*;
import oscP5.*;
import netP5.*;
import processing.sound.*;

// osc;
OscP5 oscP5;
NetAddress myRemoteLocation;

// constant variables --
String COMPORT = "COM7"; // change this!
int BAUDRATE = 921600;
static float FULL_SCALE = 16.0; // g // change this based on the setting for ICs
static float SAMPLING_RATE = 1660.0; // Hz

// --------------
static int SENSOR_NUM = 7; // change this!
static int FSR_NUM = 0; // change this!
// -----------------

static int ACTUATOR_NUM = 6;
static int CONDITION_NUM = 2;
static int AMP_CONDITIONS = 2;
// ---
static float VIBRATION_FREQ = 70;
static float DURATION = (1.0 / VIBRATION_FREQ) * 10.0; // (time span of vibration) * num // sec
static int DATA_LEN = (int)(SAMPLING_RATE * DURATION);
//


Serial myport;
Boolean isSerialReady = false;
int[] rawAccels = new int[3];
float[] accels = new float[3];

float[] SS = new float[DATA_LEN];
float[] CC = new float[DATA_LEN];


ArrayList<Accel> accelList1 = new ArrayList<Accel>();
ArrayList<Accel> accelList2 = new ArrayList<Accel>();
ArrayList<Accel> accelList3 = new ArrayList<Accel>();
ArrayList<Accel> accelList4 = new ArrayList<Accel>();
ArrayList<Accel> accelList5 = new ArrayList<Accel>();
ArrayList<Accel> accelList6 = new ArrayList<Accel>();
ArrayList<Accel> accelList7 = new ArrayList<Accel>();
ArrayList<Accel> accelList8 = new ArrayList<Accel>();
List<ArrayList<Accel>> accelLists = Arrays.asList(
  accelList1,
  accelList2,
  accelList3,
  accelList4,
  accelList5,
  accelList6,
  accelList7
  //accelList8
  );
float[] maxAmps = new float[SENSOR_NUM];
float[] calcAmps = new float[SENSOR_NUM];


// Accel Measurement
Boolean isAccelMeasure = false;
Boolean isRealtimeRec = false;
ArrayList<Accel> accelMeasureList = new ArrayList<Accel>();
ArrayList<Integer> timestamp = new ArrayList<Integer>();
int measurementSensor = 1; // id of sensor for measurement

SinOsc sine = new SinOsc(this);


// FSR
int ixFSR = 0;
int[] FSRs = new int[5000];
int FSR1 = 0;
int FSR2 = 0;
float loadingForce1 = 0;
float loadingForce2 = 0;

Boolean isVibrationOn = false;


// TODO: record vibration amps after calibrating each amps
String _dir = "./data/";
PrintWriter accelOutputWriter = null;
PrintWriter fsrOutputWriter = null;
String programTimestamp = new java.text.SimpleDateFormat("yyyyMMdd_HHmmss").format(new java.util.Date());

// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
String file_name = "sapmle";
// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-

// for posting message to Slack (for log or memo, etc...)
//WebClient wc = new WebClient();



long localTimer = 0;
int customFramerate = 30;
int recordFsrNum = 0;


// ---
public enum EventState {
  Waiting,
    RecordFSR,
    StartCalibration,
    Calibrating,
    CalibrationFinished,
    ReplacingSensor,
}
EventState eventState = EventState.Waiting;

// ------------- serial event ----
int counter = 0;
void serialEvent(Serial p) {
  if (!isSerialReady) return;
  //counter++;
  int header = 1;
  int footer = 1;
  if (p.available() > (header + 6*SENSOR_NUM + 2*FSR_NUM + footer)-1) {
    byte[] buffer = new byte[header + 6*SENSOR_NUM + 2*FSR_NUM + footer];
    //println(buffer.length);
    myport.readBytes(buffer);
    
    //int header = int(buffer[0]);
    //if (header != 0xFF) {
    //  return;
    //}
    for (int i   = 0; i < SENSOR_NUM; i++) {
      //println(i + " :----");
      //println("buffer[0]: " + buffer[0]);
      //println("sensor_id: " + i);
      rawAccels[0] = (buffer[header + 0 + i*6] << 8) + int(buffer[header + 1 + i*6]);
      rawAccels[1] = (buffer[header + 2 + i*6] << 8) + int(buffer[header + 3 + i*6]);
      rawAccels[2] = (buffer[header + 4 + i*6] << 8) + int(buffer[header + 5 + i*6]);

      accels = convertRawAccel(rawAccels);
      //print(counter + ": "); // for debug
      //println("serial received: " + rawAccels[0] + ", " + rawAccels[1] + ", " + rawAccels[2]);
      //println("serial received: " + accels[0] + ", " + accels[1] + ", " + accels[2]);

      Accel _a = new Accel(rawAccels[0], rawAccels[1], rawAccels[2]);
      accelLists.get(i).add(_a);
      if (accelLists.get(i).size() > DATA_LEN) {
        accelLists.get(i).remove(0);
      }

      // for measurement. While measure flag is true, adding Accel data to List
      if (i == measurementSensor && (isAccelMeasure || isRealtimeRec)) {
        accelMeasureList.add(_a);
        timestamp.add((int)(System.nanoTime() / 1000));
      }

      //
    }
  }
}

void setup() {
  size(1000, 800);
  frameRate(customFramerate);

  // serial
  myport = new Serial(this, COMPORT, BAUDRATE);

  // create SS and CC for quadrent detection
  float _amp = 1;
  float _phase = 0;
  for (int i = 0; i < DATA_LEN; i++) {
    SS[i] = _amp * sin(2 * PI * VIBRATION_FREQ * (i * 1 / SAMPLING_RATE) + _phase);
    CC[i] = _amp * cos(2 * PI * VIBRATION_FREQ * (i * 1 / SAMPLING_RATE) + _phase);
  }


  isSerialReady = true;
  myport.clear();
  myport.write('s');
  println("DATA_LEN: ", DATA_LEN);
}

void dispose() {
  println("onExit");
  myport.write('e');
  //toggle_vibration(false);
}

float convertValToNewton(int val) {
  float _vol = 3.3 * ((float)val / 4095.0); // convert to voltage
  //println(_vol);
  // y[V] = 0.158 x[N] + 1.65 -> x = (y - 1.65) / 0.158
  float a = 0.2221;
  float b = 1.5;
  //float a = 0.3461;
  //float b = 1.5;
  float _f = (_vol - b) / a;
  //println(val);
  return _f;
}

void displayFSR() {
  fill(200, 50, 50);
  noStroke();
  circle(600, 50, FSR1/50);
  fill(0);
  //float pf1 = convertValToNewton(FSR1);
  text(loadingForce1, 600, 100);
  //float meanFSR1 = calcArrayMean(FSRs);
  //text(meanFSR1, 600, 140);

  fill(200, 50, 50);
  circle(800, 50, FSR2/50);
  fill(0);
  //float pf2 = convertValToNewton(FSR2);
  text(loadingForce2, 800, 100);
}

void displayAccels() {
  for (int sid = 0; sid < SENSOR_NUM; sid++) {
    pushMatrix();
    translate(0, height/8*(sid+1));
    scale(1, -1);
    maxAmps[sid] = 0;
    for (int i = 0; i < accelLists.get(sid).size() - 5; i++) { // not to abrrupt during calibration
      float xScaler = width / (float)DATA_LEN;
      float yScaler = 0.5;
      noFill();
      strokeWeight(2);

      stroke(0);
      line(0, 0, width, 0); // origin

      // to exclude gravity
      Accel aa = calcAverageAccel(accelLists.get(sid));

      stroke(255, 0, 0);
      line(i * xScaler, accelLists.get(sid).get(i).ax - aa.ax * yScaler,
        (i + 1) * xScaler, accelLists.get(sid).get(i+1).ax - aa.ax * yScaler);
      stroke(0, 255, 0);
      line(i * xScaler, accelLists.get(sid).get(i).ay - aa.ay * yScaler,
        (i + 1) * xScaler, accelLists.get(sid).get(i+1).ay - aa.ay * yScaler);
      stroke(0, 0, 255);
      line(i * xScaler, accelLists.get(sid).get(i).az - aa.az * yScaler,
        (i + 1) * xScaler, accelLists.get(sid).get(i+1).az - aa.az * yScaler);

      if (accelLists.get(sid).get(i).az - aa.az > maxAmps[sid]) {
        maxAmps[sid] = accelLists.get(sid).get(i).az - aa.az;
        //println(aa.az);
      }
    }
    popMatrix();
  }
}



void outputCurrentAccelsToFile(int ampId) {
  String filename = createFileName("accel-ai" + ampId);
  accelOutputWriter = createWriter(filename); // since need to separate file for each calibration and ampId
  for (int i = 0; i < DATA_LEN; i++) {
    String row = "";
    for (int j = 0; j < SENSOR_NUM; j++) {
      ArrayList<Accel> a = accelLists.get(j);
      row += String.format("%.3f", a.get(i).ax) + ", "
        + String.format("%.3f", a.get(i).ay) + ", "
        + String.format("%.3f", a.get(i).az) + ", ";
    }
    accelOutputWriter.println(row);
  }
  accelOutputWriter.close();
}


void draw() {
  //println(frameRate);
  background(255);

  textSize(32);
  fill(0);

  // drawing fsr
  displayFSR();

  // drawing accels
  displayAccels();

  // recording FSR while installing Vp2s
  //if (eventState == EventState.RecordFSR) {
  //  // output fsr values to csv file
  //  localTimer += 1;
  //  fsrOutputWriter.println(FSR1 + ", " + loadingForce1 + ", " + FSR2 + ", " + loadingForce2);
  //  if (localTimer >= customFramerate * 2) {
  //    println("recording finished");
  //    fsrOutputWriter.close();
  //    eventState = EventState.Waiting;
  //    localTimer = 0;
  //  }
  //}


  for (int sid = 0; sid < SENSOR_NUM; sid++) {
    if (accelLists.get(sid).size() >= DATA_LEN) {
      float[] SS_CC_sum = calcSSandCCsum(accelLists.get(sid));
      float _phase = detectPhase(SS_CC_sum[0], SS_CC_sum[1]);
      float _amp = detectAmp(SS_CC_sum[0], SS_CC_sum[1]);
      fill(0);
      text("calc amp, measured amp", 50, 25);
      text(String.format("%.3f", _amp), 50, 60 + sid*40);
      text(String.format("%.3f", maxAmps[sid]), 250, 60 + sid*40);
    }
  }
}

String createAmpFileName() {
  String filename = _dir + "max_settings/" + file_name + "-max_amp_settings.csv";
  return filename;
}

// create File Name with time stamp
String createFileName(String name) {
  String timestamp = new java.text.SimpleDateFormat("yyyyMMdd_HHmmss").format(new java.util.Date());
  String filename = _dir + "acceleration_data/" + file_name + "/" + file_name + "-" + name + "-" + timestamp + ".csv";
  return filename;
}

// QD: quadrature detection
// using OSC and Max
float sineAmp = 0.1;
ArrayList<Float> calcAmpList = new ArrayList<Float>();
void measureAmpWithQD() {
  String filename = createFileName("maxamp-qd");
  PrintWriter maxAmpWriter = createWriter(filename);
  maxAmpWriter.println("frequency,amp_setting,qd_amp [m/s^2],max_amp [m/s^2]");
  sine.freq(VIBRATION_FREQ);
  sine.play();
  delay(1);
  for (float sineAmp = 0.0; sineAmp <= 1.0; sineAmp += 0.01) {
    sine.amp(sineAmp);
    delay(int(DURATION * 1000)); // insert delay to fill the buffer with sine data
    float[] SS_CC_sum = calcSSandCCsum(accelLists.get(measurementSensor));
    float _phase = detectPhase(SS_CC_sum[0], SS_CC_sum[1]);
    float _amp = detectAmp(SS_CC_sum[0], SS_CC_sum[1]);
    maxAmpWriter.print(VIBRATION_FREQ);
    maxAmpWriter.print(",");
    maxAmpWriter.print(sineAmp);
    maxAmpWriter.print(",");
    maxAmpWriter.print(_amp);
    maxAmpWriter.print(",");
    maxAmpWriter.print(maxAmps[measurementSensor]);
    maxAmpWriter.println();
  }
  maxAmpWriter.close();
  sine.stop();
  println("--- qd measurement ended -----");
}

//float tspAmp = 1.0;
//float TARGET_RMS = 1; // G
//void calibrateTspAmp() {
//  float rms = 0;
//  println("before while in calibrateTspAmp");
//  while (abs(TARGET_RMS - rms) > 0.1) {
//    accelMeasureList = new ArrayList<Accel>();
//    isAccelMeasure = true;
//    oneTspSF.play();
//    oneTspSF.amp(tspAmp);
//    delay(int(oneTspSF.duration() * 1000 + 10));
//    isAccelMeasure = false;
//    int N = accelMeasureList.size();
//    println("sample size: " + N);
//    for (Accel a : accelMeasureList) {
//      rms += ((a.az/9.80665) * (a.az/9.80665));
//    }
//    rms = sqrt(rms/N); // Due to RMS (root mean squared), don't forget to calc sqrt()
//    if (rms > TARGET_RMS) {
//      tspAmp -= 0.025;
//    } else {
//      tspAmp += 0.025;
//    }
//    println("current rms: " + rms + "tspAmp: " + tspAmp);
//  }
//  println("end of calibrateTspAmp");
//}

//void playWavAndMeasureAccel() {
//  println("------- measurement started -----------");
//  String filename = createFileName("measurement");
//  accelOutputWriter = createWriter(filename); // since need to separate file for each calibration and ampId
//  accelMeasureList = new ArrayList<Accel>();
//  timestamp = new ArrayList<Integer>();
//  isAccelMeasure = true;
//  sf.play();
//  sf.amp(tspAmp);
//  delay(int(sf.duration() * 1000 + 500)); // 500 ms margin
//  isAccelMeasure = false;
//  String header = "timestamp,az";
//  accelOutputWriter.println(header);
//  for (int i = 0; i < timestamp.size(); i++) {
//    accelOutputWriter.println(String.format("%d,%.3f", timestamp.get(i), accelMeasureList.get(i).az));
//  }
//  accelOutputWriter.close();
//  println("------------ measurement ended ------------");
//}

void checkSineOnce() {
  sine.freq(VIBRATION_FREQ);
  sine.amp(1.0);
  sine.play();
  delay(1000);
  sine.stop();
}

void keyPressed() {

  if (key == CODED) {
    if (keyCode == RIGHT) {
    } else if (keyCode == LEFT) {
    } else if (keyCode == UP) {
    } else if (keyCode == DOWN) {
    }
  }

  if (key == 'f') {
    //println("start recording FSR!");
    //recordFsrNum += 1;
    //String filename = createFileName("fsr_" + recordFsrNum);
    //fsrOutputWriter = createWriter(filename);
    //eventState = EventState.RecordFSR;
  } else if (key == 's') {
    // send 'start'
    println("start sending data");
    myport.write('s');
  } else if (key == 'e') {
    // send 'end'
    println("stop sending data");
    myport.write('e');
  } else if (key == 'c') {
    //eventState = EventState.StartCalibration;
    //delay(100); // to get vibration wave before measurement
  } else if (key == 't') {
    println("tsp amp calibration");
    thread("calibrateTspAmp");
  } else if (key == 'v') {
    isVibrationOn = !isVibrationOn;
    toggle_vibration(isVibrationOn);
  } else if (key == 'q') {
    // Quadrature detection
    println("start measuring max amp with quadrature detection");
    thread("measureAmpWithQD");
  } else if (key == 'a') {
    // play sine wave for one sec to check amp once
    thread("checkSineOnce");
  } else if (key == 'w') {
    println("play wav file and measure acceleration");
    thread("playWavAndMeasureAccel");
  } else if (key == 'r') {
    if (isAccelMeasure) {
      println("Now running play and recording");
      return;
    }
    // realtime rec start
    println("realtime recording start");
    String filename = createFileName("measurement");
    accelOutputWriter = createWriter(filename); // since need to separate file for each calibration and ampId
    accelMeasureList = new ArrayList<Accel>();
    timestamp = new ArrayList<Integer>();
    isRealtimeRec = true;
  } else if (key == 'o') {
    if (isAccelMeasure) {
      println("Now running play and recording");
      return;
    }
    // realtime rec end
    println("realtime recording ended");
    isRealtimeRec = false;
    String header = "timestamp,az";
    accelOutputWriter.println(header);
    for (int i = 0; i < timestamp.size(); i++) {
      accelOutputWriter.println(String.format("%d,%.3f", timestamp.get(i), accelMeasureList.get(i).az));
    }
    accelOutputWriter.close();
  }
}
