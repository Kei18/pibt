#include "ofMain.h"
#include "ofApp.h"
#include "app.h"


int main(int argc, char *argv[]) {
  auto P = run(argc, argv);

  // define params of visualization
  ofSetupOpenGL(10, 10, OF_WINDOW);
  ofRunApp(new ofApp(P));
}
