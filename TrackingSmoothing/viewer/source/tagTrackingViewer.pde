import oscP5.*;
import netP5.*;

OscP5 oscP5;
NetAddress myRemoteLocation;

void setup() {
  size(400, 400, P3D);
  frameRate(25);
  noSmooth();
  oscP5 = new OscP5(this, 39570);
  myRemoteLocation = new NetAddress("127.0.0.1", 39570);
  trackers = new Tracker[2][16];
  for (int c = 0; c < 2; c++) {
    for (int i = 0; i < trackers[c].length; i++) {
      trackers[c][i] = new Tracker(new PVector());
    }
  }
  predicted = new Tracker[2][16];
  for (int c = 0; c < 2; c++) {
    for (int i = 0; i < predicted[c].length; i++) {
      predicted[c][i] = new Tracker(new PVector());
    }
  }
  adjust = new Tracker[7];
  for (int i = 0; i < adjust.length; i++) {
    adjust[i] = new Tracker(new PVector());
  }
  finalt = new Tracker[7];
  for (int i = 0; i < finalt.length; i++) {
    finalt[i] = new Tracker(new PVector());
  }
}

int rmx = 0;
int rmy = 0;
float zoom = 1f;
int view = 3;
String[] viewNames = new String[]{"Raw trackers", "Combined and Predicted", "Adjust Exceptions range", "Final view"};
void draw() {
  colorMode(RGB);
  background(200);
  pushMatrix();
  translate(width/2, height/2);
  scale(zoom);
  rotateY(rmx/100f);
  rotateX(rmy/100f);
  noStroke();
  fill(255, 0, 0);
  box(1000, 1, 1);
  fill(0, 255, 0);
  box(1, 1000, 1);
  fill(0, 0, 255);
  box(1, 1, 1000);
  fill(50, 50, 50, 50);
  for (int i = -10; i <= 10; i++) {
    pushMatrix();
    translate(i*20, 0, 0);
    box(1, 400, 1);
    popMatrix();
  }
  for (int i = -10; i <= 10; i++) {
    pushMatrix();
    translate(0, i*20, 0);
    box(400, 1, 1);
    popMatrix();
  }
  stroke(0);
  if (view == 0) {
    for (int c = 0; c < 2; c++) {
      if (c == 0) fill(255, 150, 150, 200);
      else if (c == 1) fill(150, 255, 150, 200);
      for (int i = 0; i < trackers[c].length; i++) {
        Tracker t = trackers[c][i];
        if (t.tick > 2) continue;
        pushMatrix();
        translate(t.pos.x*100, t.pos.y*100, t.pos.z*100);
        box(10);
        popMatrix();
      }
    }
  } else if (view == 1) {
    colorMode(HSB);
    for (int c = 0; c < 2; c++) {
      int tr = 255;
      int size = 10;
      if (c == 1) {
        tr = 100;
        size = 7;
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
  } else if (view == 2) {
    for (int i = 0; i < adjust.length; i++) {
      Tracker t = adjust[i];
      if (t.tick > 2) continue;
      fill(255, 255, 255);
      if (t.type == 2)
        fill(255, 0, 0);
      else if (t.type == 1)
        fill(100, 255, 100, 100);
      pushMatrix();
      translate(t.pos.x*100, t.pos.y*100, t.pos.z*100);
      box(10);
      popMatrix();
    }
  } else if (view == 3) {
    colorMode(HSB);
    for (int i = 0; i < finalt.length; i++) {
      Tracker t = finalt[i];
      if (t.tick > 2) continue;
      fill((i*80)%255, 255, 255);
      pushMatrix();
      translate(t.pos.x*100, t.pos.y*100, t.pos.z*100);
      box(10);
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
    int t = theOscMessage.get(0).intValue();
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
  } else if (theOscMessage.addrPattern().equals("/VMT/Room/Unity")) {
    int i = theOscMessage.get(0).intValue() - 1;
    float x = theOscMessage.get(3).floatValue();
    float y = theOscMessage.get(4).floatValue();
    float z = theOscMessage.get(5).floatValue();
    finalt[i] = new Tracker(new PVector(-x, y, z));
  }
}
Tracker[] finalt;
Tracker[] adjust;
Tracker[][] trackers;
Tracker[][] predicted;
class Tracker {
  public PVector pos;
  public int tick = 0;
  public int type = 0;
  public Tracker(PVector p) {
    pos = p;
  }
  public Tracker(PVector p, int t) {
    pos = p;
    type = t;
  }
}
