#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include <FL/gl.h>
#include <string>
#include <iostream>

#include <cstdlib>  // for generating RNG
#include <stack>   // for generating L-System

#include "modelerglobals.h"
#include "boids.h"  // for the boids

#include "mat.h"   // for values
#include "vec.h"  // for vectors for Boids

using namespace std;


// To make a PlantModel, we inherit off of ModelerView
class PlantModel : public ModelerView 
{
public:
  PlantModel(int x, int y, int w, int h, char *label)
    : ModelerView(x,y,w,h,label), basis("1"), out(basis),
	  prevA(0), prevB(0), flock()
  { };

  virtual void generate();
  virtual string getOutput();
  virtual void setString(string t);
  virtual void draw();

private:
  string basis;
  string out;
  // for checking if values changed
  double prevA; 
  double prevB;
  Boids flock;
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* RenderModel(int x, int y, int w, int h, char *label)
{ 
  return new PlantModel(x,y,w,h,label); 
}

void PlantModel::setString(string t)
{
  out = t;
}

// Generate the L-System
void PlantModel::generate()
{
  string temp = basis;  // eventually move to out

  string dev = "0[+-1]";  // string produced by number of branches
  stack<int> pos;      // stack for holding positions of '1' in temp

  // clock_t runtime = clock();    // start runtime

  int b = 0;  // deviation generator
  // generate the deviation by BRANch per branch by append "1/" b times
  while (b < VAL(BRAN)) {
    dev.insert(3,"1/");
	  ++b;
  }


  int n = 0;  // recursion generation
  // run generation n times (use while for if RECU == 0, ignore)
  while (n < VAL(RECU))
  {
    ++n;  // increment

    // get positions of all the '1' into stack
	  int len = temp.length();
    for (int i = 0; i < len; ++i)
      if (temp[i] == '1')
        pos.push(i);

    // replace all '1' with "0[(1/)^b]"
    while (!pos.empty())
	  {
      temp.replace(pos.top(),1,dev);
	    pos.pop();
	  }
  }

  // runtime = clock() - runtime;

  // for db of output
  // cout << dev << " " << endl << temp << endl << endl;

  // for db for runtime
  // cout << "Duration: " << runtime << "ms" << endl;

  out = temp;
}

// output the string for render
string PlantModel::getOutput()
{
  return out;
}

// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out SampleModel
void PlantModel::draw()
{
  // This call takes care of a lot of the nasty projection 
  // matrix stuff.  Unless you want to fudge directly with the 
  // projection matrix, don't bother with this ...
  ModelerView::draw();

  // change generated values if recursion depth or branch count changes
  if (prevA != VAL(RECU) || prevB != VAL(BRAN))
  {
    PlantModel::generate();
    prevA = VAL(RECU);
    prevB = VAL(BRAN);
  }

  // initialize string tape from L-System
  string tape = getOutput();
  // initialize value for rotation (if branches > 1)
  double turnAngle = (VAL(BRAN) > 1) ? 360/VAL(BRAN) : 360;
  
  // draw stuff
  glPushMatrix();
    
    /* DRAW FLOOR */
    setAmbientColor(.1f,.1f,.1f);
    
    // floor
    glPushMatrix();
      glTranslated(-30, -0.5, -30);
      setDiffuseColor(204.0f/255, 204.0f/255, 255.0f/255);
      drawBox(60, 0.5, 60);
    glPopMatrix();

    /* DRAW BOIDS */
    flock.draw();

    /* DRAW PLANT */
    setAmbientColor(.1f,.1f,.1f);

    // setup the root
    glTranslated(VAL(XPOS),VAL(YPOS),VAL(ZPOS));
    glRotated(-90, 1, 0, 0);

    // DB view base
    /*
    setDiffuseColor(1,0,0);
    drawCylinder(0.5f, 10, 10);
    */

    // read through the tape
    for (unsigned int i = 0; i < tape.length(); ++i)
    {
      // interpret the char in tape for render rule
      switch(tape[i])
      {
        // render branch
        case '0':  setDiffuseColor(VAL(TR),VAL(TG),VAL(TB));
                   drawCylinder(VAL(BLEN)*VAL(TALL),VAL(BWID),VAL(CWID)*VAL(BWID));
                   break;
        // render leaf
        case '1':  setDiffuseColor(VAL(LR),VAL(LG),VAL(LB));
                   drawCylinder(VAL(BLEN),VAL(BWID),0);
                   break;
        // push matrix
        case '[':  glPushMatrix();
                   glTranslated(0,0,VAL(BLEN)*VAL(TALL));
                   glScaled(VAL(CWID),VAL(CWID),1);
                   break;
        // push angle
        case '+':  glRotated(VAL(BAGL),0,1,0);
                   break;
        // turn around branch
        case '/':  glRotated(-VAL(BAGL),0,1,0);
                   glRotated(turnAngle,0,0,1);
                   glRotated( VAL(BAGL),0,1,0);
                   break;
        // pop angle
        case '-':  glRotated(-VAL(BAGL),0,1,0);
                   break;
        // pop matrix
        case ']':  glPopMatrix();
                   break;
      }
    }

  glPopMatrix();
}


// changed main() to nomain() for replacement.
int main()
{
  // Initialize the controls
  // Constructor is ModelerControl(name, minimumvalue, maximumvalue, 
  // stepsize, defaultvalue)
  ModelerControl controls[NUMCONTROLS];

  /* PARAMETERS */
  // TREE PARAMETERS
    // Tree Position
      controls[XPOS] = ModelerControl("X Position", -5.0f, 5.0f, 0.1f, 0);
      controls[YPOS] = ModelerControl("Y Position", -5.0f, 5.0f, 0.1f, 0);
      controls[ZPOS] = ModelerControl("Z Position", -5.0f, 5.0f, 0.1f, 0);
    // Critical Values for L-System
      controls[BRAN] = ModelerControl("Branches", 0, 20, 1, 4);
      controls[RECU] = ModelerControl("Recursion Depth", 0, 20, 1, 4);
    // Required
      controls[TALL] = ModelerControl("Tallness", 0.0f, 5.0f, 0.1f, 1);
      controls[BAGL] = ModelerControl("Branch Angle", 0, 180, 5, 40);
      // Trunk Color
        controls[TR] = ModelerControl("Trunk Color R", 0.0f, 1.0f, 0.01f,  83.0f/255);
        controls[TG] = ModelerControl("Trunk Color G", 0.0f, 1.0f, 0.01f,  53.0f/255);
        controls[TB] = ModelerControl("Trunk Color B", 0.0f, 1.0f, 0.01f,  10.0f/255);
      // Trunk Color
        controls[LR] = ModelerControl("Leaf Color R", 0.0f, 1.0f, 0.01f, 186.0f/255);
        controls[LG] = ModelerControl("Leaf Color G", 0.0f, 1.0f, 0.01f, 218.0f/255);
        controls[LB] = ModelerControl("Leaf Color B", 0.0f, 1.0f, 0.01f,  95.0f/255);
    // Branch Properties
      controls[BLEN] = ModelerControl("Base Branch Length", 0.0f, 5.0f, 0.10f, 2.0f);
      controls[BWID] = ModelerControl("Base Branch Width",  0.0f, 1.5f, 0.01f, 0.3f);
      controls[CWID] = ModelerControl("Child Branch Width", 0.0f, 1.0f, 0.01f, 1.0f);
  // BOIDS PARAMETER
    controls[BOIDOR] = ModelerControl("Boids Toggle Orientation",0,1,1,0);
    controls[BOIDPD] = ModelerControl("Boids Perception Distance",2,20,1,10);
    controls[BOIDMD] = ModelerControl("Boids Minimum Distance",0.5f,4.0f,0.5f,1.0f);
    controls[BOIDFR] = ModelerControl("Boids Frame Time", 1, 40, 1, 1);

  ModelerApplication::Instance()->Init(&RenderModel, controls, NUMCONTROLS);
  return ModelerApplication::Instance()->Run();
}
