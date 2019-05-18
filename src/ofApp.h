#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include <vector>
#include "problem/problem.h"


class ofApp : public ofBaseApp {
private:
  Problem* P;
  int WIDTH;  // window width
  int HEIGHT;  // window height

  int SCALE;
  float AGENTRAD;
  float GOALRAD;
  int FONTSIZE;
  int TRIANGLESIZE;

  ofxFloatSlider timestepSlider;
  ofxFloatSlider speedSlider;
  ofxPanel gui;
  ofImage img;
  std::vector<ofImage*> sushineta;
  ofImage shokunin;
  ofImage imgHashi;

  bool showicon;
  bool showgoal;
  bool isDirect;
  bool autoplay;
  bool loop;
  bool snapshot;

  Nodes nodes;
  Nodes specials;
  Agents A;
  ofTrueTypeFont font;
  bool fontOn;
  bool lineOn;
  bool edgeOn;
  bool sushiOn;

public:
  ofApp(Problem* _P);

  void setup();
  void update();
  void draw();
  void printKeys();

  void keyPressed(int key);
  void keyReleased(int key);
  void mouseMoved(int x, int y );
  void mouseDragged(int x, int y, int button);
  void mousePressed(int x, int y, int button);
  void mouseReleased(int x, int y, int button);
  void mouseEntered(int x, int y);
  void mouseExited(int x, int y);
  void windowResized(int w, int h);
  void dragEvent(ofDragInfo dragInfo);
  void gotMessage(ofMessage msg);
};
