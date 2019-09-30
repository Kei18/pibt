#include <iostream>
#include "ofApp.h"
#include "util/util.h"

// use real photos
// #define REALSUSHI

// visualization params
static const int OSX_SCREEN_WIDTH  = 1280;
static const int OSX_SCREEN_HEIGHT = 800;
static const int SCREEN_Y_BUFFER = 75;
static const int SCREEN_X_BUFFER = 50;
static const int WINDOW_X_BUFFER = 25;
static const int WINDOW_Y_TOP_BUFFER = 75;
static const int WINDOW_Y_BOTTOM_BUFFER = 25;
static const std::string FONT = "/Library/Fonts/PTMono.ttc";

// for sushi mode
static const std::vector<std::string> SUSHINETA =
  {
   "aburi_engawa.png", "aburi_hotate.png", "aburi_salmon.png", "aburi_toro.png",
   "akagai.png", "akami.png", "amaebi.png", "anago.png", "battera.png", "buri.png",
   "ebi.png", "engawa.png", "hamachi.png", "harasu.png", "hotate.png", "ika.png",
   "ikura.png", "inarizushi.png", "iwashi.png", "kanpyo.png", "kappa.png",
   "katsuo.png", "kazunoko.png", "kohada.png", "menegi.png", "nattou.png",
   "negitoro.png", "ootoro.png", "salmon.png", "shirako.png", "shirasu.png",
   "syako.png", "syari.png", "tai.png", "tako.png", "takuwan.png", "tekkamaki.png",
   "tsuna.png", "uni2.png", "yunomi.png"
  };
static const int REALSUSHI_NUM = 34;
static const std::string SHOKUNIN = "../../material/sushi/syokunin.png";
static const std::string HASHIIMG = "../../material/sushi/chopstics.png";

// coloring
static const ofColor BGCOLOR = ofColor(0, 0, 0);
static const ofColor NODECOLOR = ofColor(255, 255, 255);
static const ofColor ENDPOINTCOLOR = ofColor(180, 180, 180);
static const ofColor FONTCOLOR = ofColor(100, 100, 100);
static const ofColor EDGECOLOR = ofColor(200, 200, 200);
static const std::vector<ofColor> AGENTCOLORS =
  {
   ofColor(233, 30, 99), ofColor(33, 150, 243),
   ofColor(76, 175, 80), ofColor(255, 152, 0),
   ofColor(0, 188, 212),  ofColor(156, 39, 176),
   ofColor(121, 85, 72), ofColor(255, 235, 59),
   ofColor(244, 67, 54), ofColor(96, 125, 139),
   ofColor(0, 150, 136), ofColor(63, 81, 181)
  };

ofApp::ofApp(Problem* _P) : P(_P) {
  int map_w = P->getG()->getW();
  int map_h = P->getG()->getH();

  int window_max_w = OSX_SCREEN_WIDTH  - SCREEN_X_BUFFER * 2 - WINDOW_X_BUFFER * 2;
  int window_max_h = OSX_SCREEN_HEIGHT - SCREEN_Y_BUFFER * 2
    - WINDOW_Y_TOP_BUFFER - WINDOW_Y_BOTTOM_BUFFER;

  SCALE = std::min(window_max_w / map_w, window_max_h / map_h) + 1;

  WIDTH  = map_w * SCALE + 2 * WINDOW_X_BUFFER;
  HEIGHT = map_h * SCALE + WINDOW_Y_TOP_BUFFER + WINDOW_Y_BOTTOM_BUFFER;

  AGENTRAD = std::max((float)SCALE/std::sqrt(2)/2, 3.0);
  GOALRAD  = std::max((float)SCALE/4.0, 2.0);
  FONTSIZE = std::max(SCALE/8, 6);
  TRIANGLESIZE = std::max(SCALE/15, 2);
  isDirect = P->getG()->isDirected();

  // set images
  showicon = P->visual_showicon;
  img.load("../../" + P->visual_icon);
  img.resize(AGENTRAD*2, AGENTRAD*2);
  shokunin.load(SHOKUNIN);
  shokunin.resize(AGENTRAD*2, AGENTRAD*2);
  imgHashi.load(HASHIIMG);
  imgHashi.resize(AGENTRAD*2, AGENTRAD*2);
#ifdef REALSUSHI
  for (int i = 1; i <= REALSUSHI_NUM; ++i) {
    ofImage *neta = new ofImage;
    neta->load("../../material/real-sushi/" + std::to_string(i) + ".png");
    neta->resize(AGENTRAD*2, AGENTRAD*2);
    sushineta.push_back(neta);
  }
#else
  for (auto netaname : SUSHINETA) {
    ofImage *neta = new ofImage;
    neta->load("../../material/sushi/" + netaname);
    neta->resize(AGENTRAD*2, AGENTRAD*2);
    sushineta.push_back(neta);
  }
#endif

  showgoal = true;
  fontOn = false;
  lineOn = false;
  edgeOn = isDirect;
  sushiOn = false;

  autoplay = false;
  loop = false;
  snapshot = false;

  nodes = P->getG()->getNodes();
  specials = P->getG()->getPickup();
  Nodes delivery = P->getG()->getDelivery();
  for (auto v : delivery) {
    if (!inArray(v, specials)) specials.push_back(v);
  }
  A = P->getA();
  printKeys();
}

//--------------------------------------------------------------
void ofApp::setup() {
  ofSetWindowShape(WIDTH, HEIGHT);
  ofBackground(BGCOLOR);
  ofSetCircleResolution(32);
  ofSetFrameRate(30);
  font.load(FONT, FONTSIZE);

  gui.setup();
  gui.add(timestepSlider.setup("time step", 0, 0, P->getTerminationTime()));
  gui.add(speedSlider.setup("speed", 0.1, 0, 1));
}

//--------------------------------------------------------------
void ofApp::update() {
  if (!autoplay) return;

  float t;
  t = timestepSlider + speedSlider;
  if (t <= P->getTerminationTime()) {
    timestepSlider = t;
  } else {
    if (loop) {
      timestepSlider = 0;
    } else {
      timestepSlider = P->getTerminationTime();
    }
  }
}

//--------------------------------------------------------------
void ofApp::draw() {
  if (snapshot) {
    ofBeginSaveScreenAsPDF("screenshot-"+ofGetTimestampString()+".pdf", false);
  }

  Vec2f pos1, pos2;
  int t1, t2;
  float x, y, p, q;
  int colors = AGENTCOLORS.size();
  Nodes neighbor;

  // draw nodes
  ofSetLineWidth(1);
  ofFill();
  for (auto v : nodes) {
    ofSetColor(NODECOLOR);
    pos1 = v->getPos() * SCALE;
    if (inArray(v, specials)) {
      ofSetColor(ENDPOINTCOLOR);
    } else {
      ofSetColor(NODECOLOR);
    }
    x = pos1.x-SCALE/2+0.5 + WINDOW_X_BUFFER + SCALE/2;
    y = pos1.y-SCALE/2+0.5 + WINDOW_Y_TOP_BUFFER + SCALE/2;
    ofDrawRectangle(x, y, SCALE-0.5, SCALE-0.5);
    if (fontOn) {
      ofSetColor(FONTCOLOR);
      font.drawString(std::to_string(v->getId()), x + 1, y + FONTSIZE + 1);
    }
  }

  // draw edge
  if (edgeOn) {
    ofSetColor(EDGECOLOR);
    for (auto v : nodes) {
      pos1 = v->getPos() * SCALE;
      neighbor = v->getNeighbor();
      x = pos1.x + WINDOW_X_BUFFER + SCALE/2;
      y = pos1.y + WINDOW_Y_TOP_BUFFER + SCALE/2;
      for (auto u : neighbor) {
        pos2 = u->getPos() * SCALE;
        p = pos2.x + WINDOW_X_BUFFER + SCALE/2;
        q = pos2.y + WINDOW_Y_TOP_BUFFER + SCALE/2;
        ofDrawLine(x, y, p, q);
        if (!isDirect) continue;
        p = (p + x) / 2;
        q = (q + y) / 2;
        if (x != p) {
          if (x < p) {
            ofDrawTriangle(p + TRIANGLESIZE, q,
                           p - TRIANGLESIZE, q - TRIANGLESIZE,
                           p - TRIANGLESIZE, q + TRIANGLESIZE);
          } else {
            ofDrawTriangle(p - TRIANGLESIZE, q,
                           p + TRIANGLESIZE, q - TRIANGLESIZE,
                           p + TRIANGLESIZE, q + TRIANGLESIZE);
          }
        } else {
          if (y < q) {
            ofDrawTriangle(p, q + TRIANGLESIZE,
                           p - TRIANGLESIZE, q - TRIANGLESIZE,
                           p + TRIANGLESIZE, q - TRIANGLESIZE);
          } else {
            ofDrawTriangle(p, q - TRIANGLESIZE,
                           p - TRIANGLESIZE, q + TRIANGLESIZE,
                           p + TRIANGLESIZE, q + TRIANGLESIZE);
          }
        }
      }
    }
  }

  // draw goals of agents
  if (showgoal) {
    for (auto a : A) {
      ofSetColor(AGENTCOLORS[a->getId() % colors]);
      t1 = (int)timestepSlider;

      // goal
      auto g = a->getHist()[t1]->g;
      if (g) {
        ofSetColor(AGENTCOLORS[a->getId() % colors]);
        pos1 = g->getPos() * SCALE;
        x = pos1.x + WINDOW_X_BUFFER + SCALE/2;
        y = pos1.y + WINDOW_Y_TOP_BUFFER + SCALE/2;
        if (sushiOn) {
          if (a->getHist()[t1]->tau) {
            imgHashi.draw(x - AGENTRAD, y - AGENTRAD);
          } else {
            t2 = t1 + 1;
            if (P->getTerminationTime() != t2 - 1) {
              g = a->getHist()[t2]->g;
              if (g) {
                pos1 = g->getPos() * SCALE;
                x = pos1.x + WINDOW_X_BUFFER + SCALE/2;
                y = pos1.y + WINDOW_Y_TOP_BUFFER + SCALE/2;
                ofSetColor(255, 255, 255);
                shokunin.draw(x - AGENTRAD, y - AGENTRAD);
              }
            }
          }
        } else {
          ofDrawRectangle(x - GOALRAD/2, y - GOALRAD/2, GOALRAD, GOALRAD);
        }
      }
    }
  }

  for (auto a : A) {
    ofSetColor(AGENTCOLORS[a->getId() % colors]);
    t1 = (int)timestepSlider;
    t2 = t1 + 1;

    // agent position
    pos1 = a->getHist()[t1]->v->getPos();
    if (t2 <= P->getTerminationTime()) {
      pos2 = a->getHist()[t2]->v->getPos();
      pos1 += (pos2 - pos1) * (timestepSlider - t1);
    }
    pos1 *= SCALE;
    x = pos1.x + WINDOW_X_BUFFER + SCALE/2;
    y = pos1.y + WINDOW_Y_TOP_BUFFER + SCALE/2;

    if (!sushiOn) {
      if (showicon) {
        ofSetColor(255,255,255);
        img.draw(x - AGENTRAD, y - AGENTRAD);
        ofSetColor(AGENTCOLORS[a->getId() % colors]);
      } else {
        ofDrawCircle(x, y, AGENTRAD);
      }
    } else {
      ofDrawEllipse(x, y+AGENTRAD/2, SCALE, AGENTRAD);
      ofSetColor(255,255,255);
      ofDrawEllipse(x, y+AGENTRAD/2, SCALE-5, AGENTRAD-5);
      auto tau = a->getHist()[t1]->tau;
      if (tau != nullptr) {
        // imgSushi.draw(x - AGENTRAD, y - AGENTRAD);
        sushineta[tau->getId() % sushineta.size()]->draw(x - AGENTRAD, y - AGENTRAD);
      }
      ofSetColor(AGENTCOLORS[a->getId() % colors]);
    }

    // goal
    if (lineOn) {
      auto g = a->getHist()[t1]->g;
      if (g != nullptr) {
        pos1 = g->getPos() * SCALE;
        ofDrawLine(pos1.x + WINDOW_X_BUFFER + SCALE/2,
                   pos1.y + WINDOW_Y_TOP_BUFFER + SCALE/2, x, y);
      }
    }

    // task
    if (!sushiOn && !showicon) {
      auto tau = a->getHist()[t1]->tau;
      if (tau == nullptr) {
        ofSetColor(255,255,255);
        ofDrawCircle(x, y, AGENTRAD-2);
      }
    }

    // id
    if (fontOn) {
      ofSetColor(FONTCOLOR);
      font.drawString(std::to_string(a->getId()), x-FONTSIZE/2, y+FONTSIZE/2);
    }
  }

  gui.draw();

  if (snapshot) {
    ofEndSaveScreenAsPDF();
    snapshot = false;
  }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
  float t;
  if (key == 'p') autoplay = !autoplay;
  if (key == 'l') loop = !loop;
  if (key == 'r') timestepSlider = 0;
  if (key == 'v') lineOn = !lineOn;
  if (key == 'e') edgeOn = !edgeOn;
  if (key == 'i') showicon = !showicon;
  if (key == 'g') showgoal = !showgoal;
  if (key == 's') sushiOn = !sushiOn;
  if (key == 'f') {
    fontOn = !fontOn;
    fontOn &= (SCALE - FONTSIZE > 6);
  }

  if (key == OF_KEY_RIGHT) {
    t = timestepSlider + speedSlider;
    timestepSlider = std::min((float)P->getTerminationTime(), t);
  }
  if (key == OF_KEY_LEFT) {
    t = timestepSlider - speedSlider;
    timestepSlider = std::max((float)0, t);
  }
  if (key == OF_KEY_UP) {
    t = speedSlider + 0.001;
    speedSlider = std::min(t, (float)speedSlider.getMax());
  }
  if (key == OF_KEY_DOWN) {
    t = speedSlider - 0.001;
    speedSlider = std::max(t, (float)speedSlider.getMin());
  }
  if (key == 32) { // space
    snapshot = true;
  }
}

void ofApp::printKeys() {
  std::cout << "You can control visualization." << "\n";
  std::cout << "- p : play or pause" << "\n";
  std::cout << "- l : loop or not" << "\n";
  std::cout << "- r : reset" << "\n";
  std::cout << "- v : show virtual line" << "\n";
  std::cout << "- f : show node id" << "\n";
  std::cout << "- e : show edge" << "\n";
  std::cout << "- i : show icon" << "\n";
  std::cout << "- g : show goal" << "\n";
  std::cout << "- s : sushi mode" << "\n";
  std::cout << "- right : progress" << "\n";
  std::cout << "- left  : back" << "\n";
  std::cout << "- up    : speed up" << "\n";
  std::cout << "- down  : speed down" << "\n";
  std::cout << "- space : screen shot" << "\n";
}

void ofApp::keyReleased(int key) {}
void ofApp::mouseMoved(int x, int y ) {}
void ofApp::mouseDragged(int x, int y, int button) {}
void ofApp::mousePressed(int x, int y, int button) {}
void ofApp::mouseReleased(int x, int y, int button) {}
void ofApp::mouseEntered(int x, int y) {}
void ofApp::mouseExited(int x, int y) {}
void ofApp::windowResized(int w, int h) {}
void ofApp::gotMessage(ofMessage msg) {}
void ofApp::dragEvent(ofDragInfo dragInfo) {}
