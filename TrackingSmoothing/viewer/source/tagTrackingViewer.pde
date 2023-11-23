import oscP5.*;
import netP5.*;

OscP5 oscP5;
NetAddress myRemoteLocation;

void setup() {
  size(800, 800, P3D);
  frameRate(60);
  noSmooth();
  oscP5 = new OscP5(this, 15460);
  myRemoteLocation = new NetAddress("127.0.0.1", 15460);
  trackers = new Tracker[5][36];
  for (int c = 0; c < 5; c++) {
    for (int i = 0; i < trackers[c].length; i++) {
      trackers[c][i] = new Tracker(new PVector());
    }
  }
  predicted = new Tracker[3][36];
  for (int c = 0; c < 3; c++) {
    for (int i = 0; i < predicted[c].length; i++) {
      predicted[c][i] = new Tracker(new PVector());
    }
  }
  adjust = new Tracker[7];
  for (int i = 0; i < adjust.length; i++) {
    adjust[i] = new Tracker(new PVector());
  }
  finalt = new Tracker[10];
  for (int i = 0; i < finalt.length; i++) {
    finalt[i] = new Tracker(new PVector());
  }
  calibrate = new Tracker[20];
  for (int i = 0; i < calibrate.length; i++) {
    calibrate[i] = new Tracker(new PVector());
  }
}

int rmx = 0;
int rmy = 0;
float zoom = 2f;
int view = 0;
String[] viewNames = new String[]{"Final view", "Raw trackers", "Combined and Predicted", "Adjust Exceptions range", "Tracker Calibration"};
void draw() {
  colorMode(RGB);
  background(200);
  pushMatrix();
  translate(width/2, height/2);
  scale(zoom);
  rotateY(rmx/100f);
  rotateX(rmy/100f);
  noStroke();
  float lineSize = 0.5f;
  fill(255, 0, 0);
  box(1000, lineSize, lineSize);
  fill(0, 255, 0);
  box(lineSize, 1000, lineSize);
  fill(0, 0, 255);
  box(lineSize, lineSize, 1000);
  fill(50, 50, 50, 50);
  for (int i = -10; i <= 10; i++) {
    pushMatrix();
    translate(i*20, 0, 0);
    box(lineSize, 400, lineSize);
    popMatrix();
  }
  for (int i = -10; i <= 10; i++) {
    pushMatrix();
    translate(0, i*20, 0);
    box(400, lineSize, lineSize);
    popMatrix();
  }
  int boxSize = 6;
  stroke(0);
  if (view == 1) { //Raw
    colorMode(HSB);
    for (int c = 0; c < 5; c++) {
      //if (c == 0) fill(255, 150, 150, 200);
      //else if (c == 1) fill(150, 255, 150, 200);
      //else if (c == 2) fill(150, 150, 255, 200);
      fill((c*75)%255, 120, 255, 200);
      for (int i = 0; i < trackers[c].length; i++) {
        Tracker t = trackers[c][i];
        if (t.tick > 2) continue;
        pushMatrix();
        translate(t.pos.x*100, t.pos.y*100, t.pos.z*100);
        box(boxSize);
        popMatrix();
      }
    }
    colorMode(RGB);
  } else if (view == 2) { //Predict
    colorMode(HSB);
    for (int c = 0; c < 3; c++) {
      int tr = 255;
      int size = boxSize;
      if (c == 1) {
        tr = 100;
        size = (int)(boxSize * 0.7);
      }
      if (c == 2) {
        tr = 50;
        size = (int)(boxSize * 0.5);
      }
      for (int i = 0; i < trackers[c].length; i++) {
        Tracker t = predicted[c][i];
        if (t.tick > 2) continue;
        fill((i*60)%255, 255, 255, tr);
        pushMatrix();
        translate(t.pos.x*100, t.pos.y*100, t.pos.z*100);
        box(size);
        popMatrix();
      }
    }
    colorMode(RGB);
  } else if (view == 3) { //Adjust
    for (int i = 0; i < adjust.length; i++) {
      Tracker t = adjust[i];
      if (t.tick > 2) continue;
      fill(255, 255, 255);
      if (t.type == 2)
        fill(0, 255, 0);
      else if (t.type == 1)
        fill(100, 255, 100, 100);
      else if (t.type == 3)
        fill(255, 0, 0);
      pushMatrix();
      translate(t.pos.x*100, t.pos.y*100, t.pos.z*100);
      box(boxSize);
      popMatrix();
    }
  } else if (view == 0) { //Final
    colorMode(HSB);
    for (int i = 0; i < finalt.length; i++) {
      Tracker t = finalt[i];
      if (t.tick > 2) continue;
      fill((i*80)%255, 255, 255);
      pushMatrix();
      //double[] rot = quaternionToEulerAngles(t.quat[0], t.quat[1], t.quat[2], t.quat[3]);
      double[] mat = quatToMatrix(t.quat);
      translate(t.pos.x*100, t.pos.y*100, t.pos.z*100);
      applyMatrix(
        (float)mat[0], (float)mat[1], (float)mat[2], 0f,
        (float)mat[3], (float)mat[4], (float)mat[5], 0f,
        (float)mat[6], (float)mat[7], (float)mat[8], 0f,
        0f, 0f, 0f, 1f);
      box(boxSize);
      translate(-boxSize/2, boxSize/4, 0);
      box(boxSize/5, boxSize/2, boxSize/5);
      popMatrix();
    }
    colorMode(RGB);
  } else if (view == 4) { //Calibrate
    colorMode(HSB);
    for (int i = 0; i < calibrateQuads.size(); i++) {
      Tracker t = calibrateQuads.get(i);
      fill(255, 120, 255, 200);
      pushMatrix();
      translate(t.pos.x*100, t.pos.y*100, t.pos.z*100);
      box(boxSize);
      popMatrix();
    }
    for (int i = 0; i < calibrate.length; i++) {
      Tracker t = calibrate[i];
      if (t.tick > 2) continue;
      float size = 1f;
      if (t.type == 1)
        fill(255, 200, 255, 200);
      else if (t.type == 2) {
        size = 0.5f;
        fill(100, 200, 255, 100);
      }
      pushMatrix();
      translate(t.pos.x*100, t.pos.y*100, t.pos.z*100);
      box(boxSize * size);
      popMatrix();
    }
    colorMode(RGB);
  }
  popMatrix();
  fill(10);
  text(viewNames[view], 0, 10);
}
void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  e = e * 0.1f;
  e += 1f;
  zoom /= e;
}
void mouseDragged() {
  rmx -= pmouseX - mouseX;
  rmy += pmouseY - mouseY;
}
void keyPressed() {
  view++;
  if (view >= viewNames.length)
    view = 0;
}
void oscEvent(OscMessage theOscMessage) {
  if (theOscMessage.addrPattern().equals("/debug/trackers/position")) {
    int i = theOscMessage.get(0).intValue();
    int c = theOscMessage.get(1).intValue();
    float x = theOscMessage.get(2).floatValue();
    float z = theOscMessage.get(3).floatValue();
    float y = theOscMessage.get(4).floatValue();
    trackers[c][i] = new Tracker(new PVector(-x, y, z));
  } else if (theOscMessage.addrPattern().equals("/debug/predicted/position")) {
    int i = theOscMessage.get(0).intValue();
    println(i);
    int p = theOscMessage.get(1).intValue();
    float x = theOscMessage.get(2).floatValue();
    float z = theOscMessage.get(3).floatValue();
    float y = theOscMessage.get(4).floatValue();
    predicted[p][i] = new Tracker(new PVector(-x, y, z));
  } else if (theOscMessage.addrPattern().equals("/debug/adjust/position")) {
    int i = theOscMessage.get(0).intValue();
    int t = theOscMessage.get(1).intValue();
    float x = theOscMessage.get(2).floatValue();
    float z = theOscMessage.get(3).floatValue();
    float y = theOscMessage.get(4).floatValue();
    adjust[i] = new Tracker(new PVector(-x, y, z), t);
  } else if (theOscMessage.addrPattern().equals("/debug/tick")) {
    for (int c = 0; c < 2; c++) {
      for (int i = 0; i < trackers[c].length; i++) {
        trackers[c][i].tick++;
      }
    }
    for (int c = 0; c < 2; c++) {
      for (int i = 0; i < predicted[c].length; i++) {
        predicted[c][i].tick++;
      }
    }
    for (int i = 0; i < finalt.length; i++) {
      finalt[i].tick++;
    }
    for (int i = 0; i < calibrate.length; i++) {
      calibrate[i].tick++;
    }
  } else if (theOscMessage.addrPattern().equals("/debug/final/position")) {
    int i = theOscMessage.get(0).intValue() - 1;
    float x = theOscMessage.get(1).floatValue();
    float z = theOscMessage.get(2).floatValue();
    float y = theOscMessage.get(3).floatValue();
    float q1 = theOscMessage.get(4).floatValue();
    float q2 = theOscMessage.get(5).floatValue();
    float q3 = theOscMessage.get(6).floatValue();
    float q4 = theOscMessage.get(7).floatValue();
    finalt[i] = new Tracker(new PVector(-x, y, z), new double[]{-q1, q3, q2, q4});
  } else if (theOscMessage.addrPattern().equals("/debug/calibrate/position")) {
    int i = theOscMessage.get(0).intValue() - 1;
    int t = theOscMessage.get(1).intValue();
    float x = theOscMessage.get(2).floatValue();
    float z = theOscMessage.get(3).floatValue();
    float y = theOscMessage.get(4).floatValue();
    if (t == 0)
      calibrateQuads.add(new Tracker(new PVector(-x, y, z)));
    else
      calibrate[i] = new Tracker(new PVector(-x, y, z), t);
  } else if (theOscMessage.addrPattern().equals("/debug/calibrate/clear")) {
    calibrateQuads.clear();
  }
}
public final double[] quatToMatrix(double[] q) {
  double sqw = q[3]*q[3];
  double sqx = q[0]*q[0];
  double sqy = q[1]*q[1];
  double sqz = q[2]*q[2];

  // invs (inverse square length) is only required if quaternion is not already normalised
  double invs = 1 / (sqx + sqy + sqz + sqw);
  double m00 = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
  double m11 = (-sqx + sqy - sqz + sqw)*invs ;
  double m22 = (-sqx - sqy + sqz + sqw)*invs ;

  double tmp1 = q[0]*q[1];
  double tmp2 = q[2]*q[3];
  double m10 = 2.0 * (tmp1 + tmp2)*invs ;
  double m01 = 2.0 * (tmp1 - tmp2)*invs ;

  tmp1 = q[0]*q[2];
  tmp2 = q[1]*q[3];
  double  m20 = 2.0 * (tmp1 - tmp2)*invs ;
  double  m02 = 2.0 * (tmp1 + tmp2)*invs ;
  tmp1 = q[1]*q[2];
  tmp2 = q[0]*q[3];
  double  m21 = 2.0 * (tmp1 + tmp2)*invs ;
  double  m12 = 2.0 * (tmp1 - tmp2)*invs ;
  return new double[]{m00, m01, m02, m10, m11, m12, m20, m21, m22};
}
Tracker[] finalt;
Tracker[] adjust;
Tracker[][] trackers;
Tracker[][] predicted;
Tracker[] calibrate;
ArrayList<Tracker> calibrateQuads = new ArrayList<Tracker>();
class Tracker {
  public PVector pos;
  public int tick = 0;
  public int type = 0;
  public double[] quat = new double[4];
  public Tracker(PVector p) {
    pos = p;
  }
  public Tracker(PVector p, double[] q) {
    pos = p;
    quat = q;
  }
  public Tracker(PVector p, int t) {
    pos = p;
    type = t;
  }
}
