// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

//#include "tutorial_device.h"
#include <embree3/rtcore.h>
#include <iostream>
#include <iomanip>
#include <cmath>


/* scene data */
RTCScene g_scene = nullptr;
RTCDevice g_device = nullptr;
int g_x;

struct Vertex   { float x,y,z,r;
Vertex(float _x,float _y, float  _z):x(_x),y(_y),z(_z){}
float Mag(){
	return sqrt(x*x+y*y+z*z);
}
void Print(){
	std::cout << x <<", " << y << ", " << z << std::endl;
}

};
struct Triangle { int v0, v1, v2; };

unsigned int addPrism (RTCScene scene_i);
double CalculateDistance(RTCScene scene, double px, double py, double pz, double dx, double dy, double dz, int &geomID,
                    bool &inside);

int main(int argc, char* argv[])
{

	 g_device = rtcNewDevice("");
	 g_scene = rtcNewScene(g_device);
	 rtcSetSceneFlags(g_scene, RTC_SCENE_FLAG_CONTEXT_FILTER_FUNCTION);
     rtcSetSceneBuildQuality(g_scene, RTC_BUILD_QUALITY_HIGH);

	 addPrism(g_scene);
	 rtcCommitScene (g_scene);

	 int geomID  = -1;
	 bool inside = false;
	 double Dist = CalculateDistance(g_scene,0.,0.5,10.,0.,0.,-1,geomID,inside);

	 std::cout << "Dist : " << Dist << " : Inside : " << inside << std::endl;

	 Dist = CalculateDistance(g_scene,0.,0.5,0.5,0.,0.,-1,geomID,inside);
	 std::cout << "Dist : " << Dist << " : Inside : " << inside << std::endl;
	 //Deleting the scene
	 rtcReleaseScene (g_scene);
	 return 0;
}

double CalculateDistance(RTCScene scene, double px, double py, double pz, double dx, double dy, double dz, int &geomID,
                    bool &inside)
{
  Vertex initial(px,py,pz);
  RTCRayHit ray; 
  ray.ray.flags = 0;
  ray.ray.org_x = px;
  ray.ray.org_y = py;
  ray.ray.org_z = pz;
  ray.ray.dir_x = dx;
  ray.ray.dir_y = dy;
  ray.ray.dir_z = dz;
  ray.ray.tnear = 0.;
  ray.ray.tfar  = 1E20f;
  {
    RTCIntersectContext context;

    g_x = px;

   rtcInitIntersectContext(&context);

    rtcIntersect1(scene, &context, &ray);
 }
  geomID = ray.hit.geomID;

  std::cout << "========= Trying to Print Ray ==========" << std::endl;
  std::cout << ray.hit.Ng_x << "," << ray.hit.Ng_y << "," << ray.hit.Ng_z << std::endl;

  Vertex final(initial.x+ray.ray.tfar*dx,initial.y+ray.ray.tfar*dy,initial.z+ray.ray.tfar*dz);
  std::cout << "Final : ";final.Print();
  Vertex diff(final.x-initial.x,final.y-initial.y,final.z-initial.z);
  std::cout << "Diff Mag : " << diff.Mag() << std::endl;
  return ray.ray.tfar;
}

/* adds a cube to the scene */
unsigned int addPrism (RTCScene scene_i)
{
  /* create a triangulated Prism with 8 triangles and 6 vertices */
  RTCGeometry mesh = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);


  /* set vertices and vertex colors */
  Vertex* vertices = (Vertex*) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(Vertex),8);
  //vertex_colors[0] = Vec3fa(0,1,1);
  vertices[0].x = 0; vertices[0].y = 1; vertices[0].z = 3;
  //vertex_colors[1] = Vec3fa(-1,0,1);
  vertices[1].x = -1; vertices[1].y = 0; vertices[1].z = 3;
  //vertex_colors[2] = Vec3fa(1,0,1);
  vertices[2].x = 1; vertices[2].y = 0; vertices[2].z = 3;
  //vertex_colors[3] = Vec3fa(0,1,-1);
  vertices[3].x = 0; vertices[3].y = +1; vertices[3].z = -1;
  //vertex_colors[4] = Vec3fa(-1,0,-1);
  vertices[4].x = -1; vertices[4].y = 0; vertices[4].z = -1;
  //vertex_colors[5] = Vec3fa(1,0,-1);
  vertices[5].x = +1; vertices[5].y = 0; vertices[5].z = -1;

  /* set triangles and face colors */
  int tri = 0;
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(Triangle),8);

  // Top Face (only one triangle)
  //face_colors[tri] = Vec3fa(1,0,0);
  triangles[tri].v0 = 0; triangles[tri].v1 = 1; triangles[tri].v2 = 2; tri++;
// Bottom Face (only one triangle)
  //face_colors[tri] = Vec3fa(1,0,0);
  triangles[tri].v0 = 3; triangles[tri].v1 = 4; triangles[tri].v2 = 5; tri++;

  // first rectangular side (two triangles)
  //face_colors[tri] = Vec3fa(0,1,0);
  triangles[tri].v0 = 0; triangles[tri].v1 = 1; triangles[tri].v2 = 3; tri++;
  //face_colors[tri] = Vec3fa(0,1,0);
  triangles[tri].v0 = 1; triangles[tri].v1 = 4; triangles[tri].v2 = 3; tri++;


  // second rectangular side (two triangles)
  //face_colors[tri] = Vec3fa(0,0,1);/*0.5f);*/
  triangles[tri].v0 = 0; triangles[tri].v1 = 3; triangles[tri].v2 = 2; tri++;
  //face_colors[tri] = Vec3fa(0,0,1);/*0.5f);*/
  triangles[tri].v0 = 2; triangles[tri].v1 = 3; triangles[tri].v2 = 5; tri++;

  // third rectangular side (two triangles)
  //face_colors[tri] = Vec3fa(1.0f);
  triangles[tri].v0 = 1; triangles[tri].v1 = 4; triangles[tri].v2 = 2; tri++;
  //face_colors[tri] = Vec3fa(1.0f);
  triangles[tri].v0 = 2; triangles[tri].v1 = 4; triangles[tri].v2 = 5; tri++;

//  rtcSetGeometryVertexAttributeCount(mesh,1);
//  rtcSetSharedGeometryBuffer(mesh,RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE,0,RTC_FORMAT_FLOAT3,vertex_colors,0,sizeof(Vec3fa),6);
  
  rtcCommitGeometry(mesh);
  unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
  rtcReleaseGeometry(mesh);

  return geomID;
}
