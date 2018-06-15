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
  kinect.setTilt(15);
  
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
  //Inicia o video de kinect
  kinect.initVideo();

  // Get the raw depth as array of integers
  int[] depth = kinect.getRawDepth();

  // We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
  int skip = 2;
  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;
  float count = 0;
  
  float sumX2 = 0;
  float sumY2 = 0;

  // Translate and rotate
  translate(width/2, height/2, -50);
  rotateY(a);

  for (int x = 0; x < kinect.width; x += skip) {
    for (int y = 0; y < kinect.height; y += skip) {
      int offset = x + y*kinect.width;

      // Convert kinect data to world xyz coordinate
      int rawDepth = depth[offset];
      PVector v = depthToWorld(x, y, rawDepth);
      strokeWeight(1);
      stroke(255);
      pushMatrix();
      // Scale up by 200
      float factor = 2000;
      translate(v.x*factor, v.y*factor, 5*factor-v.z*5*factor);
      // Draw a point
      //point(0, 0);
      if (rawDepth < tracker.threshold) {
        sumX = sumX+v.x*factor;
        sumY = sumY+v.y*factor;
        sumZ = sumZ+factor-v.z*factor;
        
        sumX2 = sumX2 + x;
        sumY2 = sumY2 + y;
        
        count = count + 1;
      }
      popMatrix();
    }
  }
  
  PImage imagem = kinect.getVideoImage();
  PImage recorte = imagem.get(int(sumX2/count)-25, int((sumY2/count) + 15)-25, 50, 50);
  recorte.loadPixels();
  int dimension = recorte.width * recorte.height;
  
  int countR = 0;
  
  //boolean flagR = false;
  
  for (int i = 0; i < dimension; i += 2) {
    float r = red(recorte.pixels[i]);
    float g = green(recorte.pixels[i]);
    float b = blue(recorte.pixels[i]);
    
    if(r > b && r > g){
      countR = countR + 1;
    }
    //else print("O");
    //else if (g > 200){
      //print("Verde!");
    //}
    //else if (b > 200) {
      //print("Azul!");
    //}
    //else {
      //print("nada!");
    //}
  }
  if (countR/dimension > 0.3){
    print("Vermelho!");
  }
  image(imagem, 0, 0);
  
  //if (flagR) print("Vermelho!");
  //else print("nada!");
  
  //PVector v2 = tracker.getLerpedPos();
  //float factor = 2000;
  //int offset = int(v2.x) + int(v2.y*kinect.width);
  //int rawDepth = depth[offset];
  //PVector v = depthToWorld(int(v2.x), int(v2.y), rawDepth);
  //pushMatrix();
  //translate(v.x*factor,v.y*factor,factor - v.z*factor);
  //sphere(28);
  //popMatrix();
  
  
  
  translate(0,0);
  desenho_X[birl] = sumX/count;
  desenho_Y[birl] = sumY/count;
  desenho_Z[birl] = sumZ/count;
  
  
  if (birl == 511) {
      birl = 0;
  }
  else{
       birl++;
     }
  
  stroke(random(0, 255), random(0, 255), random(0, 255));
  strokeWeight(10);
  beginShape();
  for(int i =0;i<birl;i++){
    curveVertex(desenho_X[i],desenho_Y[i], desenho_Z[i]);
    //fill(random(0, 255), random(0, 255), random(0, 255), 100);
    //ellipse(desenho_X[i], desenho_Y[i], 50, 50);
    //pushMatrix();
    //translate(desenho_X[i],desenho_Y[i], desenho_Z[i]);
    //box(28);
    //popMatrix();
  }
  endShape();
  
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
