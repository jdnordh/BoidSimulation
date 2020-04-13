//------------------------------------------------------------------------------
// A simple example showing how to use the triangle geometry
//------------------------------------------------------------------------------
#include "givr.h"
#include <glm/gtc/matrix_transform.hpp>

#include "io.h"
#include "turntable_controls.h"
#include <cmath>
#include <ctime>
#include <iostream>
#include <fstream>
#include <string>

#define null nullptr

using namespace glm;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;
using std::vector;
using givr::vec3f;
using givr::mat4f;
using std::string;

struct Controler;
struct BoidCollection;
struct Boid;
struct Obstacle;
struct Config
{
	/*
	 * Config file structure:
	 * Avoid Scale
	 * Align Scale
	 * Cohesion Scale
	 * Obstacle Avoid Scale
	 * Follow the Leader Scale
	 */
   vector<float> m_vals;
   float AvoidScale() { return m_vals[0]; }
   float AlignScale() { return m_vals[1]; }
   float CohesionScale() { return m_vals[2]; }
   float ObstacleAvoidScale() { return m_vals[3]; }
   float LeaderFollowScale() { return m_vals[4]; }
};

Config GetConfigFromFile()
{
   string filename = "../options.ini";
   std::ifstream in;
   in.open(filename);
   string line;
   Config c;
   while (!in.eof())
   {
      getline(in, line);
      float read = atof(line.c_str());
      c.m_vals.push_back(read);
   }
   return c;
}

Config g_config = GetConfigFromFile();

float RandFloat();
float RandFloatInRange(float lo, float hi);
bool FloatEquals(float lhs, float rhs);
bool VecIsZero(vec3f vec);
vec3f GetRandomNormalVector();
Boid* GenerateRandomBoid(const vec3f& min, const vec3f& max);
Obstacle* GenerateRandomObstacle(const vec3f& min, const vec3f& max);

int g_seed = 893546;

float sqr(float x)
{
   return pow(x, 2);
}

struct Obstacle
{
   bool IsVerticle;
   vec3f Position;
   float Radius;
	Obstacle(bool isVerticle, vec3f position, float radius)
	{
      IsVerticle = isVerticle;
      Position = position;
      Radius = radius;
	}

   mat4f GetTranformation() const
   {
      vec3f scaleVec = { Radius,1.f,Radius };
      vec3f xAxis = { 1.f,0,0 };
      float rotation = IsVerticle ? 0.f : pi<float>() / 2;
      auto transformed = scale(rotate(mat4f{ 1.f }, rotation, xAxis), scaleVec);
      mat4f translated = mat4f{ 1.f, 0, 0, Position.x,
			0, 1.f, 0, Position.y,
         0, 0, 1.f, Position.z,
         0, 0, 0, 1.f
   };
      return transpose(translated) * transformed;
	}
};

struct Boid
{
   const float BOID_SCALE = 0.5f;
   vec3f ForceAccumulation;
   vec3f Acceleration;
   vec3f Position;
   vec3f Velocity;
   const vec3f DefaultForwardVector = {1,0,0};
	
   Boid(vec3f position = {0,0,0}, vec3f velocity = {0,0,0})
	{
      ForceAccumulation = { 0,0,0 };
      Position = position;
      Velocity = velocity;
      Acceleration = { 0,0,0 };
	}

   vec3f Forward() const
   {
   	if (VecIsZero(Velocity))
   	{
         return { 1,0,0 };
   	}
      return normalize(Velocity);
   }

   void AvoidObstacle(Obstacle* obstacle)
   {
      float forceScale = g_config.ObstacleAvoidScale();
      vec3f avoid = obstacle->Position;
   	// Avoid a cylinder
   	if (obstacle->IsVerticle)
   	{
         avoid.y = Position.y;
   	}
      else
      {
         avoid.z = Position.z;
      }
      vec3f dir = Position - avoid;
      float distance = length(dir);
   	if (FloatEquals(distance, 0))
   	{
         distance = 0.0001f;
   	}
      const float safeDist = 4.f;
   	if (distance < obstacle->Radius + safeDist)
   	{
         ForceAccumulation += normalize(dir) * forceScale / distance;
   	}
   }
	
	void Avoid(Boid* other, float distance)
   {
      const float scale = g_config.AvoidScale();
      vec3f dirToGo = Position - other->Position;
      dirToGo = normalize(dirToGo);
      ForceAccumulation += dirToGo * scale / distance;
   }

	void Align(Boid* other, float distance)
   {
      const float scale = g_config.AlignScale();
      vec3f dirToGo = normalize(other->Velocity);
      ForceAccumulation += dirToGo * scale / distance;
   }

	void GoTogether(Boid* other, float distance)
   {
      const float scale = g_config.CohesionScale();
      vec3f dirToGo = other->Position - Position;
      dirToGo = normalize(dirToGo);
      ForceAccumulation += dirToGo * scale / distance;
   }

   mat4f GetTransformation() const 
   {
      vec3f forward = Forward();
      const vec3f gravity = { 0, -50, 0 };
      vec3f netAccel = normalize(gravity + Acceleration);
   	if (VecIsZero(netAccel))
   	{
         netAccel = { 1, 0 ,0 };
   	}
      vec3f left = cross(forward, netAccel);
      vec3f up = normalize(cross(forward, left));
      left = normalize(cross(up, forward));
   	
      return transpose(mat4f{ forward.x, up.x, left.x, Position.x,
      forward.y, up.y, left.y, Position.y,
      forward.z, up.z, left.z, Position.z,
      0,0,0,1 });
   }

	void DoTimeStep(const vec3f& minBounds, const vec3f& maxBounds, vec3f* leader = null)
   {
      float deltaTime = 0.01f;
      float maxSpeed = 25.f;

      float leaderScale = g_config.LeaderFollowScale();
      if (leader != null)
      {
         vec3f toLeader = normalize(*leader - Position);
         ForceAccumulation += leaderScale * toLeader;
      }
   	

      Velocity += ForceAccumulation * deltaTime;
      float speed = length(Velocity);
   	if (speed > maxSpeed)
   	{
         Velocity *= (maxSpeed / speed);
   	}
      Position += Velocity * deltaTime;
      if (Position.x < minBounds.x)
      {
      	// Loop in the x direction
         Position.x = maxBounds.x;
      }
      else if (Position.x > maxBounds.x)
      {
         Position.x = minBounds.x;
      }
      if (Position.y < minBounds.y)
      {
         Velocity.y = abs(Velocity.y);
      }
      else if (Position.y > maxBounds.y)
      {
         Velocity.y = -abs(Velocity.y);
      }
      if (Position.z < minBounds.z)
      {
         Velocity.z = abs(Velocity.z);
      }
      else if (Position.z > maxBounds.z)
      {
         Velocity.z = -abs(Velocity.z);
      }
      Acceleration = ForceAccumulation;
      ForceAccumulation = { 0,0,0 };
   }
};

struct BoidCollection
{
   const float AVOID_DISTANCE = 3.f;
   const float ALIGN_DISTANCE = 6.f;
   const float COHESION_DISTANCE = 9.f;
	
   vector<Boid*> Boids;
   vector<Obstacle*> Obstacles;
	
   BoidCollection()
   {
   }

	void AddBoid(Boid* boid)
   {
      Boids.push_back(boid);
   }

	void AddObstacle(Obstacle* obstacle)
   {
      Obstacles.push_back(obstacle);
   }
	
	void DoTimeStep(const vec3f& minBounds, const vec3f& maxBounds, vec3f* leader = null)
   {
      for (int i = 0; i < Boids.size(); ++i)
      {
         auto thisBoid = Boids[i];
	      for(int j = i + 1; j < Boids.size(); ++j)
	      {
            auto otherBoid = Boids[j];
            float distance = length(thisBoid->Position - otherBoid->Position);
            if (FloatEquals(distance, 0))
            {
               distance = 0.0001;
            }
            if (distance < AVOID_DISTANCE)
            {
               thisBoid->Avoid(otherBoid, distance);
               otherBoid->Avoid(thisBoid, distance);
            }
            //if (distance < ALIGN_DISTANCE)
            {
               thisBoid->Align(otherBoid, distance);
               otherBoid->Align(thisBoid, distance);
            }
            //else if (distance < COHESION_DISTANCE)
            {
               thisBoid->GoTogether(otherBoid, distance);
               otherBoid->GoTogether(thisBoid, distance);
            }
	      }
      	for (auto obstacle : Obstacles)
      	{
            thisBoid->AvoidObstacle(obstacle);
      	}
      }
   	for(auto& boid : Boids)
   	{
         boid->DoTimeStep(minBounds, maxBounds, leader);
   	}
   }
};

struct Controler
{
   BoidCollection BoidCollection;
   io::CursorPosition CursorPosition;
   const vec3f MaxBounds = { 50,20,20 };
   const vec3f MinBounds = { -50,-20,-20 };

   bool FollowTheLeader = false;
   vec3f LeaderPosition = {0,0,0};
   Controler()
   {
   }
};

Controler g_controler;

int main(void)
{
   io::GLFWContext windows;
   auto window = windows.create(io::Window::dimensions{ 1920, 1000 }, "Boids Simulation");
   window.enableVsync(true);

   auto view = View(TurnTable(), Perspective());
   TurnTableControls controls(window, view.camera);
	
   window.keyboardCommands()
      | io::Key(GLFW_KEY_ESCAPE,
         [&](auto const&/*event*/) { window.shouldClose(); })
      | io::Key(GLFW_KEY_UP,
         [&](auto const&/*event*/)
		   {
		      if (g_controler.BoidCollection.Boids.size() < 250)
		      {
		         g_controler.BoidCollection.AddBoid(
		            GenerateRandomBoid(g_controler.MinBounds, g_controler.MaxBounds));
		      }
		   })
      | io::Key(GLFW_KEY_DOWN,
         [&](auto const&/*event*/)
         {
            if (g_controler.BoidCollection.Boids.size() > 1)
            {
               g_controler.BoidCollection.Boids.pop_back();
            }
         })
      | io::Key(GLFW_KEY_LEFT,
         [&](auto const&/*event*/)
         {
            if (g_controler.LeaderPosition.x > g_controler.MinBounds.x)
            {
               g_controler.LeaderPosition.x -= 5.f;
            }
         })
      | io::Key(GLFW_KEY_RIGHT,
         [&](auto const&/*event*/)
         {
            if (g_controler.LeaderPosition.x < g_controler.MaxBounds.x)
            {
               g_controler.LeaderPosition.x += 5.f;
            }
         })
      | io::Key(GLFW_KEY_SPACE,
         [&](auto const& event)
         {
            if (event.action == GLFW_PRESS)
            {
               g_controler.FollowTheLeader = !g_controler.FollowTheLeader;
            }
         });
   
   glClearColor(1.f, 1.f, 1.f, 1.f);

   // Make a boid
   const float BOID_SCALE = 0.5f;
   auto ts = TriangleSoup();
	   vec3f points[] = {
	   vec3f(0, 1, 0) * BOID_SCALE,
	   vec3f(-0.5, 0, 2) * BOID_SCALE,
	   vec3f(0, -1, 0) * BOID_SCALE,
	   vec3f(-0.5, 0, -2) * BOID_SCALE,
	   vec3f(3, 0, 0) * BOID_SCALE,
   };
   ts.push_back(points[0], points[1], points[2]);
   ts.push_back(points[0], points[2], points[3]);
   ts.push_back(points[0], points[4], points[1]);
   ts.push_back(points[0], points[4], points[3]);
   ts.push_back(points[2], points[4], points[1]);
   ts.push_back(points[2], points[4], points[3]);
   auto boidStyle = Phong(Colour(0.99f, 0.5f, 0.45f), LightPosition(100.f, 100.f, 100.f), AmbientFactor(0.3f));
   auto boidModel = givr::createInstancedRenderable(ts, boidStyle);

   // Make a cylinder obstacle
   auto obstacleStyle = Phong(Colour(0.1f, 0.1f, 0.1f), LightPosition(100.f, 100.f, 100.f), AmbientFactor(0.3f));
   auto cyl = Cylinder(Radius(1.f), Point1( 0, g_controler.MinBounds.y, 0), Point2(0,g_controler.MaxBounds.y, 0));
   auto cylinder = givr::createInstancedRenderable(cyl, obstacleStyle);

   // Make a leader quad
   vec3f min = g_controler.MinBounds;
   vec3f max = g_controler.MaxBounds;
   auto q = Quad(Point1(0, min.y, min.z), Point2(0, max.y, min.z), 
      Point3(0, max.y, max.z), Point4(0, min.y, max.z));
   auto leaderStyle = Phong(Colour(0.1f, 0.8f, 0.8f), LightPosition(100.f, 100.f, 100.f), AmbientFactor(0.3f));
   auto quad = givr::createInstancedRenderable(q, leaderStyle);
	
	for(int i = 0; i < 50; ++i)
	{
      g_controler.BoidCollection.AddBoid(GenerateRandomBoid(g_controler.MinBounds, g_controler.MaxBounds));
	}
	for(int i = 0; i < 10; ++i)
	{
     g_controler.BoidCollection.AddObstacle(GenerateRandomObstacle(g_controler.MinBounds, g_controler.MaxBounds));
	}

   window.run([&](float frameTime)
      {
         view.projection.updateAspectRatio(window.width(), window.height());

         // Render boids
         for (const auto& boid : g_controler.BoidCollection.Boids)
         {
            auto matrix = boid->GetTransformation();
            addInstance(boidModel, matrix);
         }
         draw(boidModel, view);

         for(const auto& obstacle : g_controler.BoidCollection.Obstacles)
         {
            auto matrix = obstacle->GetTranformation();
            addInstance(cylinder, matrix);
         }
         draw(cylinder, view);

			if (g_controler.FollowTheLeader)
			{
            addInstance(quad, translate(mat4f{ 1.f }, g_controler.LeaderPosition));
            draw(quad, view);
			}
         vec3f* leader = null;
			if (g_controler.FollowTheLeader)
			{
            leader = &g_controler.LeaderPosition;
			}
		
         g_controler.BoidCollection.DoTimeStep(g_controler.MinBounds, g_controler.MaxBounds, leader);
      });

   exit(EXIT_SUCCESS);
}

bool FloatEquals(float lhs, float rhs)
{
   const float epsilon = 0.000001f;
   return abs(lhs - rhs) < epsilon;
}

bool VecIsZero(vec3f vec)
{
   return FloatEquals(vec.x, 0.f) && FloatEquals(vec.y, 0.f) && FloatEquals(vec.z, 0.f);
}

vec3f GetRandomNormalVector()
{
   return normalize(vec3f(RandFloat(), RandFloat(), RandFloat()));
}

// Get a random float from 0 - 1
float RandFloat()
{
	return (float)rand() / (float)RAND_MAX;
}

float RandFloatInRange(float lo, float hi)
{
   auto r = rand();
   float value = lo + ((float)r / (float)RAND_MAX) * (hi - lo);
   return value;
}

Boid* GenerateRandomBoid(const vec3f& min, const vec3f& max)
{
   srand(g_seed);
   g_seed += 13;
   vec3f velocity = 
   {
      RandFloatInRange(-5, 5),
      RandFloatInRange(-5, 5),
      RandFloatInRange(-5, 5)
   };
   vec3f position = 
   {
   	RandFloatInRange(min.x, max.x),
      RandFloatInRange(min.y, max.y),
      RandFloatInRange(min.z, max.z)
   };
   return new Boid(position, velocity);
}

Obstacle* GenerateRandomObstacle(const vec3f& min, const vec3f& max)
{
   srand(g_seed);
   g_seed += 13;
   bool isVerticle = RandFloatInRange(0.f, 2.f) > 1.f ? true : false;
   vec3f pos;
	if (isVerticle)
	{
      pos.y = 0.f;
      pos.x = RandFloatInRange(min.x, max.x);
      pos.z = RandFloatInRange(min.z, max.z);
	}
   else
   {
      pos.z = 0.f;
      pos.x = RandFloatInRange(min.x, max.x);
      pos.y = RandFloatInRange(min.y, max.y);
   }
   float radius = RandFloatInRange(0.2f, 2.f);
   return new Obstacle(isVerticle, pos, radius);
}