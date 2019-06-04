#include "boids.h"

#include "modelerglobals.h"  // calling VAL(PARAMETER)
#include "modelerdraw.h"    // For draw() function
#include "modelerapp.h"    // piggyback modelerglobals

#include <cstdlib>   // For srand() and rand()
#include <random>   // For RNG
#include <ctime>   // For time()
#include <cmath>  // For trig

#include "vec.h"

#define PI 3.1415926535897;

using namespace std;

//==[ HELPER FUNCTION ]========================================================
/* RNG */
double rng (double min, double max)
{
  random_device rd;
  mt19937 gen(rd());
  uniform_real_distribution<double> random_double_in_range
    = uniform_real_distribution<double>(min,max);
  return random_double_in_range(gen);
}
/* Cross Product */
Vec3<double> crossProduct (Vec3<double> a, Vec3<double> b)
{
  return Vec3<double>(a[1]*b[2]-a[2]*b[1],
                      a[2]*b[0]-a[0]*b[2],
                      a[0]*b[1]-a[1]*b[0]);
}
/* Dot Product */
double dotProduct (Vec3<double> a, Vec3<double> b)
{ return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
/* Radian To Degree */
double degree (double r)
{
  return r * 180 / PI;
}

//==[ BOID ]====================================================================
/* CONSTRUCTORS */
// default
Boid::Boid ()
{
  double x = rng(-BOID_RANGE,BOID_RANGE);
  double y = rng(-BOID_RANGE,BOID_RANGE);
  double z = rng(-BOID_RANGE,BOID_RANGE);

  // setting inverse point to center
  Vec3<double> centerPointer = Vec3<double>(-x,-y,-z);
  centerPointer.normalize();
  
  maxVel = MAX_VELOCITY;

  setPosition(Vec3<double>(x,y,z));
  setVelocity(centerPointer);
}
// basis
Boid::Boid (Vec3<double> lower, Vec3<double> upper)
{
  double x = rng(lower[0],upper[0]);
  double y = rng(lower[1],upper[1]);
  double z = rng(lower[2],upper[2]);

  // setting inverse point to center
  Vec3<double> centerPointer = Vec3<double>(-x,-y,-z);
  centerPointer.normalize();
  
  maxVel = MAX_VELOCITY;

  setPosition(Vec3<double>(x,y,z));
  setVelocity(centerPointer);
}
// specific
Boid::Boid (double x, double y, double z)
{
  Vec3<double> centerPointer = Vec3<double>(-x,-y,-z);
  centerPointer.normalize();
  
  maxVel = MAX_VELOCITY;

  setPosition(Vec3<double>(x,y,z));
  setVelocity(centerPointer);
}
Boid::Boid (Vec3<double> p)
{
  // setting inverse point to center
  Vec3<double> centerPointer = Vec3<double>(-p[0],-p[1],-p[2]);
  centerPointer.normalize();
  
  maxVel = MAX_VELOCITY;

  setPosition(p);
  setVelocity(centerPointer);
}

/* FUNCTIONS */
Vec3<double> Boid::getPosition () { return position; }
Vec3<double> Boid::getVelocity () { return velocity; }
Vec3<double> Boid::getOrientation () { return orientation; }
void Boid::setPosition (Vec3<double> v){ position = v; }
void Boid::setVelocity (Vec3<double> v)
{
  // limit_velocity rule
  if (v.length() > MAX_VELOCITY)
  {
    v.normalize();
    orientation = v;
    velocity = v * MAX_VELOCITY;
  }
  else
  {
    velocity = v;
    v.normalize();
    orientation = v;
  }
}

bool Boid::sameAs (Boid& b) { return (&b == this); }


//==[ BOIDS ]===================================================================
/* CONSTRUCTORS */
// default
Boids::Boids ()
{
  // set frame counter
  frames = 0;
  goalPoint = 0;

  // set destination positions
  double angle;
  Vec3<double> temp[8];

  for (int i = 0; i < 8; ++i)
  {
    angle = PI;
    angle *= (2.0*i/8);
    temp[i] = Vec3<double> (cos(angle)*BOID_POINT, BOID_HEIGHT, sin(angle)*BOID_POINT);
  }

  setDestinations(temp,8);

  Vec3<double> lo = Vec3<double> (-BOID_RANGE, 1, -BOID_RANGE);
  Vec3<double> hi = Vec3<double> ( BOID_RANGE, BOID_RANGE, BOID_RANGE);
  setBoundaries(lo,hi);

  // initalize all boids in the flock
  for (int i = 0; i < BOID_COUNT; ++i)
    flock[i] = Boid(lo,hi);

  // DB output
  // output();
}

/* FUNCTIONS */
// PUBLIC
void Boids::update ()
{
  // Get Boid Frame Time for Runtime
  int getFrameTime = VAL(BOIDFR);

  // init vectors for use (under frame rate)
  if ((++frames % getFrameTime) == 0)
  {
    Vec3<double> r1,r2,r3,r4,r5,r6;
    
    for (int i = 0; i < BOID_COUNT; ++i)
    {
      // get boid
      Boid *b = &flock[i];
  
      // get vectors for all rules
      r1 =  separation(b);
      r2 =   alignment(b);
      r3 =    cohesion(b);
      r4 = destination(b);
      r5 =    bounding(b);
      r6 =   offcenter(b);
      
      // DB Value Output
      /*
      cout << "BOID " << i << endl
           << "R1: " << r1 << endl
           << "R2: " << r2 << endl
           << "R3: " << r3 << endl
           << "R4: " << r4 << endl
           << "R5: " << r5 << endl << endl;
      */

      // set values
      b->setVelocity(b->getVelocity()+r1+r2+r3+r4+r5+r6);
      b->setPosition(b->getPosition()+b->getVelocity());
    }
  }
}

void Boids::draw()
{
  // update values
  update();
  //output();

  // init values for position, orientation, and axis of rotation
  Vec3<double> base = Vec3<double>(0,0,1);
  Vec3<double> p, o, a;
  double r;

  // actual render
  for (int i = 0; i < BOID_COUNT; ++i)
  {
    glPushMatrix();

      p = flock[i].getPosition();
      // draw boid
      glTranslated(p[0],p[1],p[2]);
      setAmbientColor(.1f,.1f,.1f);
      setDiffuseColor(COLOR_BLUE);
      drawSphere(0.25f);

      if (VAL(BOIDOR) == 1)
      {
        o = flock[i].getOrientation();
        // get axis of rotation (cross product)
        a = crossProduct(base, o);
        // get angle of rotation (from Projection)
        r = acos(dotProduct(base, o) / base.length() / base.length() / o.length());
        // draw stuff
        glRotated(degree(r), a[0],a[1],a[2]);
        setAmbientColor(1,1,1);
        setDiffuseColor(COLOR_RED);
        drawCylinder(1,0.05f,0.05f);
      }

    glPopMatrix();
  }

  // DB visualize the destinations
  /*
  for (unsigned int i = 0; i < destList.size(); ++i)
  {
    glPushMatrix();
      p = destList[i];
      glTranslated(p[0],p[1],p[2]);
      setAmbientColor(.1f,.1f,.1f);
      setDiffuseColor(COLOR_GREEN);
      drawSphere(0.1f);
    glPopMatrix();
  }
  */
}

void Boids::setDestinations (Vec3<double>* points, int size)
{
  for (int i = 0; i < size; ++i)
    destList.push_back(*(points + i));
}

void Boids::setBoundaries (Vec3<double> lo, Vec3<double> up)
{
  loBound = lo;
  upBound = up;
}

// PRIVATE
// Rules of the Boids
// 1. Separation
Vec3<double> Boids::separation (Boid *b)
{
  Vec3<double> out = Vec3<double>();

  Vec3<double> range;
  for (int i = 0; i < BOID_COUNT; ++i)
  {
    if (!flock[i].sameAs(*b))
    {
      range = flock[i].getPosition() - b->getPosition();
      out -= (range.length() < VAL(BOIDMD)) ? range : Vec3<double>();
    }
  }

  return out * SEP_SCALE;
}
// 2. Alignment
Vec3<double> Boids::alignment (Boid *b)
{
  Vec3<double> out = Vec3<double>();
  Vec3<double> range;

  for (int i = 0; i < BOID_COUNT; ++i)
    if (!flock[i].sameAs(*b))
    {
      // check if within perception distance
      range = flock[i].getPosition() - b->getPosition();
      if (range.length() <= VAL(BOIDPD))
        out += flock[i].getVelocity();
    }

  out /= (BOID_COUNT - 1);

  return (out - b->getVelocity()) * ALI_SCALE;
}
// 3. Choesion
Vec3<double> Boids::cohesion (Boid *b)
{
  Vec3<double> out = Vec3<double>();
  Vec3<double> range;

  for (int i = 0; i < BOID_COUNT; ++i)
    if (!flock[i].sameAs(*b))
    {
      // check if within perception distance
      range = flock[i].getPosition() - b->getPosition();
      if (range.length() <= VAL(BOIDPD))
        out += flock[i].getPosition();
    }

  out /= (BOID_COUNT - 1);

  return (out - b->getPosition()) * COH_SCALE;
}
// 4. Destination
Vec3<double> Boids::destination (Boid *b)
{
  Vec3<double> pos, des, dist;

  pos = b->getPosition();
  des = destList[goalPoint];
  dist = des - pos;

  if (dist.length() < DEST_BIAS)
  {
    goalPoint = (goalPoint + 1) % destList.size();
    des = destList[goalPoint];
    dist = des - pos;
  }

  dist.normalize();

  return dist * DES_SCALE;
}
// 5. Bounding
Vec3<double> Boids::bounding (Boid *b)
{
  Vec3<double> out = Vec3<double>();
  Vec3<double> pos = b->getPosition();

  // for all axis
  for (int i = 0; i < 3; ++i)
  {
    if (pos[i] < loBound[i])
      out[i] = BOUND_PUSH;
    else if (pos[i] > upBound[i])
      out[i] = -BOUND_PUSH;
  }

  return out * BOU_SCALE;
}
// 6. Off-Center
Vec3<double> Boids::offcenter (Boid *b)
{
  Vec3<double> out = Vec3<double>();

  Vec3<double> pos = b->getPosition();
  // distance from center of tree
  double range = sqrt(pos[0]*pos[0]+pos[2]*pos[2]);

  // if in there, push boid away;
  if (range < OFC_RANGE){
    out = Vec3<double>(pos[0],0,pos[2]);
    out.normalize();
  }

  return out * BOUND_PUSH * OFC_SCALE;
}


/* HELPER FUNCTION */
void Boids::output ()
{
  char* iden = "    ";
  // get frame
  cout << "Frame: " << frames << "  Goal: " << goalPoint << endl;
  // get each boid's parameter
  for (int i = 0; i < BOID_COUNT; ++i)
  {
    double dist = (destList[goalPoint] - flock[i].getPosition()).length();

    cout << "[" << i << "] ";
    cout << "Pos: " << flock[i].getPosition() << endl;
    cout << iden << "Vel: " << flock[i].getVelocity() << endl;
    cout << iden << "Ori: " << flock[i].getOrientation() << endl;
    cout << " Dist: " << dist << endl;
  }
}