///* incoming osc message are forwarded to the oscEvent method. */
//void oscEvent(OscMessage theOscMessage) {
//  /* print the address pattern and the typetag of the received OscMessage */
//  print("### received an osc message.");
//  print(" addrpattern: "+theOscMessage.addrPattern());
//  println(" typetag: "+theOscMessage.typetag());
//}

int[] vibrationGates = new int[6];

void init_osc() {
  oscP5 = new OscP5(this, 3333); // listening port
  //oscServer = new NetAddress("127.0.0.1", 12000);
  myRemoteLocation = new NetAddress("127.0.0.1", 12000); // sending port
}

void send_cue_to_left() {
  OscMessage myMessage = new OscMessage("/cue/left");
  //myMessage.add(1.0);
  oscP5.send(myMessage, myRemoteLocation);
}

void send_cue_to_right() {
  OscMessage myMessage = new OscMessage("/cue/right");
  //myMessage.add(1.0);
  oscP5.send(myMessage, myRemoteLocation);
}

//void toggle_noise() {
//  OscMessage myMessage = new OscMessage("/noise");
//  //myMessage.add(1.0);
//  oscP5.send(myMessage, myRemoteLocation);
//}

void toggle_vibration(Boolean on) {
  OscMessage myMessage = new OscMessage("/vibration/toggle");
  if (on) {
    myMessage.add(1);
  } else {
    myMessage.add(0);
  }
  oscP5.send(myMessage, myRemoteLocation);
}

void set_anterior_vibration() {
  OscMessage myMessage = new OscMessage("/vibration/ant");
  //myMessage.add(1.0);
  oscP5.send(myMessage, myRemoteLocation);
}

void set_vibration_off() {
  OscMessage myMessage = new OscMessage("/vibration/off");
  //myMessage.add(1.0);
  oscP5.send(myMessage, myRemoteLocation);
}

void set_vibration_amp(int actuatorId, int ampId, float amp) {
  String ampAddr = "/c" + (ampId);
  OscMessage myMsg = new OscMessage("/vibration/amp/" + actuatorId + ampAddr);
  myMsg.add(amp);
  oscP5.send(myMsg, myRemoteLocation);
}

void all_gates_off() {
  for (int i = 0; i < ACTUATOR_NUM; i++) {
    vibrationGates[i] = 0;
    OscMessage myMsg = new OscMessage("/vibration/gate/" + (i+1)); 
    myMsg.add(0);
    oscP5.send(myMsg, myRemoteLocation);
  }
}

void set_vibration_gate(int id, int state) {
  vibrationGates[id - 1] = state;
  OscMessage myMsg = new OscMessage("/vibration/gate/" + id); 
  //int value = isOn ? 1 : 0;
  myMsg.add(state);
  oscP5.send(myMsg, myRemoteLocation);
}

void set_posterior_vibration() {
  OscMessage myMessage = new OscMessage("/vibration/post");
  //myMessage.add(1.0);
  oscP5.send(myMessage, myRemoteLocation);
}
