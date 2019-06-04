#ifndef __BOIDS_HEADER__
#define __BOIDS_HEADER__

#include "vec.h"

#include <vector>

/* BOID CONSTANTS */
#define BOID_RANGE 15

/* BOIDS CONSTANTS */
#define BOID_COUNT 15     // amount of boids in scene
#define MAX_VELOCITY 0.5  // max velocity of boids

#define BOID_POINT 15     // distance for the goals
#define BOID_HEIGHT 5     // height of the goals

#define BOUND_PUSH 1      // push force when out of range or in tree
#define DEST_BIAS 2       // radius of the destination
#define OFC_RANGE 10      // radius of the tree to keep boid away

// scalar value of forces
#define SEP_SCALE 1       // separation
#define ALI_SCALE 0.125   // alignment
#define COH_SCALE 0.01    // cohesion
#define DES_SCALE 0.05    // destination
#define BOU_SCALE 0.3     // bounding
#define OFC_SCALE 1       // off-center

/* INDIVIDUAL BOID */
class Boid
{
public:
  /* CONSTRUCTORS */
  // default
  Boid ();
  // for specific range
  Boid (Vec3<double>,Vec3<double>);
  // for specific location
  Boid (double, double, double);
  Boid (Vec3<double>);

  /* FUNCTIONS */
  // get functions
  Vec3<double> getPosition ();
  Vec3<double> getVelocity ();
  Vec3<double> getOrientation ();
  // set functions
  void setPosition (Vec3<double> v);
  void setVelocity (Vec3<double> v); // and also orientation
  // compare function
  bool sameAs(Boid&);

private:
  // attributes holding position, velocity, and orientation
  Vec3<double> position, velocity, orientation;
  double maxVel;
};


/* BOID FLOCKS */
class Boids
{
public:
  /* CONSTRUCTORS */
  // default
  Boids ();
  // for specific count of boids [make it later]
  // Boids (int);

  /* FUNCTIONS */
  // move all boids to new position
  void update();
  // draw boids
  void draw();
  // set positions for destination rule
  void setDestinations(Vec3<double>*, int);
  // set boundaries for boundaries
  void setBoundaries(Vec3<double>, Vec3<double>);

private:
  /* ATTRIBUTES */
  // holding an array of boids
  Boid flock[BOID_COUNT];
  // hold list of destinations for destination rules
  vector<Vec3<double>> destList;
  // hold the objective location pointer for flock to a ceratin destination
  int goalPoint;
  // hold range for bounding
  Vec3<double> loBound, upBound;
  // contain frame
  unsigned int frames;

  /* FUNCTIONS */
  /* RULES of the BOID */
  // 1. Separation: have the boid maintain distance between each other
  Vec3<double> separation (Boid *b);
  // 2. Alignment: have the boid match velocity & alignment of nearby velocity
  Vec3<double> alignment (Boid *b);
  // 3. Choesion: have the boid cohere to the center mass
  Vec3<double> cohesion (Boid *b);

  /* ADDITIONAL RULES of the BOID */
  // 4. Destination: Tendency towards a particular place
  Vec3<double> destination (Boid *b);
  // 5. Bounding: Keep the flock in a certain area
  Vec3<double> bounding (Boid *b);
  // 6. Off-Center: stay away from the tree
  Vec3<double> offcenter (Boid *b);

  /* HELPER FUNCTION */
  // DB - Return attributes of boids
  void output();
};


#endif