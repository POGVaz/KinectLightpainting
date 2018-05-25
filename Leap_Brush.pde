// Daniel Shiffman
// Kinect Point Cloud example

// https://github.com/shiffman/OpenKinect-for-Processing
// http://shiffman.net/p5/kinect/

import org.openkinect.freenect.*;
import org.openkinect.processing.*;

import peasy.PeasyCam;

// Kinect Library object
KinectTracker tracker;
Kinect kinect;

PeasyCam cam;

boolean mostra = true;

// Angle for rotation
float a = 0;
int birl = 0;

// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];

float[] desenho_X = new float[512];
float[] desenho_Y = new float[512];
float[] desenho_Z = new float[512];

void setup() {
  // Rendering in P3D
  size(800, 600, P3D);
  kinect = new Kinect(this);
  kinect.initDepth();
  
  tracker = new KinectTracker();

  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);
  }
  
  cam = new PeasyCam(this, 400);
}

void draw() {

  background(0);
  
  // Run the tracking analysis
  tracker.track();
  // Show the image
  tracker.display(mostra);

  // Get the raw depth as array of integers
  int[] depth = kinect.getRawDepth();

  // We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
  int skip = 2;

  // Translate and rotate
  translate(width/2, height/2, -50);
  rotateY(a);

  for (int x = 0; x < kinect.width; x += skip) {
    for (int y = 0; y < kinect.height; y += skip) {
      int offset = x + y*kinect.width;

      // Convert kinect data to world xyz coordinate
      int rawDepth = depth[offset];
      PVector v = depthToWorld(x, y, rawDepth);

      stroke(255);
      pushMatrix();
      // Scale up by 200
      float factor = 200;
      translate(v.x*factor, v.y*factor, factor-v.z*factor);
      // Draw a point
      if (rawDepth < tracker.threshold) {
        point(0, 0);
      }
      popMatrix();
    }
  }

  // Let's draw the "lerped" location
  PVector v2 = tracker.getLerpedPos();
  fill(100, 250, 50, 200);
  noStroke();
  ellipse(v2.x, v2.y, 20, 20);
  
  desenho_X[birl] = v2.x;
  desenho_Y[birl] = v2.y;
  desenho_Z[birl] = v2.z;
  if (birl == 511) {
      birl = 0;
  }
  else{
       birl++;
     }
  for(int i =0;i<birl;i++){
    fill(random(0, 255), random(0, 255), random(0, 255), 100);
    ellipse(desenho_X[i]*3, desenho_Y[i]*3, 50, 50);
    pushMatrix();
    translate(desenho_X[i]*3,desenho_Y[i]*3, desenho_Z[i]);
    sphere(28);
    popMatrix();
  }
  

}

// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}

// Adjust the threshold with key presses
void keyPressed() {
  int t = tracker.getThreshold();
  if (key == CODED) {
    if (keyCode == UP) {
      t+=5;
      tracker.setThreshold(t);
    } else if (keyCode == DOWN) {
      t-=5;
      tracker.setThreshold(t);
    } else if (keyCode == ALT) {
      mostra = !mostra;
    } else if (keyCode == LEFT) {
      birl = 0;
    }
  }
}
