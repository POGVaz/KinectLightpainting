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

boolean mostra = false;

boolean desenha = true;

final int MAX_POINTS = 4095;  

// Angle for rotation
float a = 0;
int birl = 0;

// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];

float[] desenho_X = new float[MAX_POINTS];
float[] desenho_Y = new float[MAX_POINTS];
float[] desenho_Z = new float[MAX_POINTS];
int[] cor       = new int[MAX_POINTS];
int[] traco       = new int[MAX_POINTS];
int traco1 = 0;

int[] stops = new int[MAX_POINTS];

void setup() {
  // Rendering in P3D
  size(1024, 768, P3D);
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
  translate(0, 0, -50);
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
      float factor = 500;
      translate(v.x*factor, v.y*factor, 5*factor-v.z*5*factor);
      // Draw a point
      if (rawDepth < tracker.threshold) {
        point(0, 0);
        sumX = sumX+v.x*factor;
        sumY = sumY+v.y*factor;
        sumZ = sumZ+5*factor-v.z*5*factor;
        
        sumX2 = sumX2 + x;
        sumY2 = sumY2 + y;
        
        count = count + 1;
      }
      popMatrix();
    }
  }
  
   //Variáveis de suporte, para debug e calibar a posição do cursor.
  PImage imagem = kinect.getVideoImage();
  PImage recorte = imagem.get(int(sumX2/count)-25, int((sumY2/count) + 15)-25, 50, 50);
  recorte.loadPixels();
  int dimension = recorte.width * recorte.height;
  
  image(imagem, 0, 0);
  
  float r = 0;
  float g = 0;
  float b = 0;
  
  for (int i = 0; i < dimension; i += 2) {
    r += red(recorte.pixels[i]);
    g += green(recorte.pixels[i]);
    b += blue(recorte.pixels[i]);
  }

  if (desenha) {
    desenho_X[birl] = sumX/count;
    desenho_Y[birl] = sumY/count;
    desenho_Z[birl] = sumZ/count;
    
    if (birl == MAX_POINTS-1) {
      birl = 0;
    }
    else{
      birl++;
    }
    
    if (r > g && r > b){
      cor[birl] = 1;
    }
    else if (g > b){
      cor[birl] = 2;
    }
    else {
      cor[birl] = 3;
    }
  }
traco[birl] = traco1;  
  
  
  translate(0,0);
  noFill();
  
  beginShape();
    for(int i = 0; i<birl; i++){
      strokeWeight(traco[i]);
      if (cor[i] == 1) {
        stroke(255, 0, 0);
      }
      else if (cor[i] == 2) {
        stroke(0, 255, 0);
      }
      else if (cor[i] == 3) {
        stroke(0, 0, 255);
      }
      curveVertex(desenho_X[i],desenho_Y[i], desenho_Z[i]);
      for (int j=0; j<stops.length; j++){
        if (stops[j] == i) {
          endShape();
          beginShape();
        }
      }
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

// Ajuste de parâmetros com o pressinar de teclas.
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
    else if (keyCode == RIGHT){
       //desenha = !desenha;
       //append(stops, birl);
        if (traco1 == 0) {
          traco1 = 5;
        }
        else{
          traco1 = 0;
        }
    }
  }
}
