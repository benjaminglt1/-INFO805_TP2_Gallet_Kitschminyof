/**
@file Renderer.h
@author JOL
*/
#pragma once
#ifndef _RENDERER_H_
#define _RENDERER_H_

#include "Color.h"
#include "Image2D.h"
#include "Ray.h"

/// Namespace RayTracer
namespace rt {

  inline void progressBar( std::ostream& output,
                           const double currentValue, const double maximumValue)
  {
    static const int PROGRESSBARWIDTH = 60;
    static int myProgressBarRotation = 0;
    static int myProgressBarCurrent = 0;
    // how wide you want the progress meter to be
    double fraction = currentValue /maximumValue;
    
    // part of the progressmeter that's already "full"
    int dotz = static_cast<int>(floor(fraction * PROGRESSBARWIDTH));
    if (dotz > PROGRESSBARWIDTH) dotz = PROGRESSBARWIDTH;
    
    // if the fullness hasn't changed skip display
    if (dotz == myProgressBarCurrent) return;
    myProgressBarCurrent = dotz;
    myProgressBarRotation++;
    
    // create the "meter"
    int ii=0;
    output << "[";
    // part  that's full already
    for ( ; ii < dotz;ii++) output<< "#";
    // remaining part (spaces)
    for ( ; ii < PROGRESSBARWIDTH;ii++) output<< " ";
    static const char* rotation_string = "|\\-/";
    myProgressBarRotation %= 4;
    output << "] " << rotation_string[myProgressBarRotation]
           << " " << (int)(fraction*100)<<"/100\r";
    output.flush();
  }
  
  /// This structure takes care of rendering a scene.
  struct Renderer {

    /// The scene to render
    Scene* ptrScene;
    /// The origin of the camera in space.
    Point3 myOrigin;
    /// (myOrigin, myOrigin+myDirUL) forms a ray going through the upper-left
    /// corner pixel of the viewport, i.e. pixel (0,0)
    Vector3 myDirUL;
    /// (myOrigin, myOrigin+myDirUR) forms a ray going through the upper-right
    /// corner pixel of the viewport, i.e. pixel (width,0)
    Vector3 myDirUR;
    /// (myOrigin, myOrigin+myDirLL) forms a ray going through the lower-left
    /// corner pixel of the viewport, i.e. pixel (0,height)
    Vector3 myDirLL;
    /// (myOrigin, myOrigin+myDirLR) forms a ray going through the lower-right
    /// corner pixel of the viewport, i.e. pixel (width,height)
    Vector3 myDirLR;

    int myWidth;
    int myHeight;

    Renderer() : ptrScene( 0 ) {}
    Renderer( Scene& scene ) : ptrScene( &scene ) {}
    void setScene( rt::Scene& aScene ) { ptrScene = &aScene; }
    
    void setViewBox( Point3 origin, 
                     Vector3 dirUL, Vector3 dirUR, Vector3 dirLL, Vector3 dirLR )
    {
      myOrigin = origin;
      myDirUL = dirUL;
      myDirUR = dirUR;
      myDirLL = dirLL;
      myDirLR = dirLR;
    }

    void setResolution( int width, int height )
    {
      myWidth  = width;
      myHeight = height;
    }


    /// The main rendering routine
    void render( Image2D<Color>& image, int max_depth )
    {
      std::cout << "Rendering into image ... might take a while." << std::endl;
      image = Image2D<Color>( myWidth, myHeight );
      for ( int y = 0; y < myHeight; ++y ) 
        {
          Real    ty   = (Real) y / (Real)(myHeight-1);
          progressBar( std::cout, ty, 1.0 );
          Vector3 dirL = (1.0f - ty) * myDirUL + ty * myDirLL;
          Vector3 dirR = (1.0f - ty) * myDirUR + ty * myDirLR;
          dirL        /= dirL.norm();
          dirR        /= dirR.norm();
          for ( int x = 0; x < myWidth; ++x ) 
            {
              Real    tx   = (Real) x / (Real)(myWidth-1);
              Vector3 dir  = (1.0f - tx) * dirL + tx * dirR;
              Ray eye_ray  = Ray( myOrigin, dir, max_depth );
              Color result = trace( eye_ray );
              image.at( x, y ) = result.clamp();
            }
        }
      std::cout << "Done." << std::endl;
    }


    /// The rendering routine for one ray.
    /// @return the color for the given ray.
    Color trace( const Ray& ray )
    {
      assert( ptrScene != 0 );
      Color result = Color( 0.0, 0.0, 0.0 );
      GraphicalObject* obj_i = 0; // pointer to intersected object
      Point3           p_i;       // point of intersection

      // Look for intersection in this direction.
      Real ri = ptrScene->rayIntersection( ray, obj_i, p_i );
      // Nothing was intersected
      if ( ri >= 0.0f ) return Color( 0.0, 0.0, 0.0 ); // some background color

      //3.3
      //recuperation du materiau
      /*
      Material m = obj_i->getMaterial(p_i);
      
      return Color(m.ambient[0]+m.diffuse[0],m.ambient[1]+m.diffuse[1],m.ambient[2]+m.diffuse[2]);
      */
     //3.4
     return illumination(ray,obj_i,p_i);
     
    }

    //3.4
    Color illumination(const Ray& ray,GraphicalObject* obj,Point3 p ){
      Material m = obj->getMaterial(p);
      Color C = Color(0.0,0.0,0.0);

      std::vector< Light* > lights = ptrScene->myLights;
      for ( Light* l : lights ){
          Vector3 L = l->direction(p);
          Vector3 N = obj->getNormal(p);
          Real kd = L.dot(N);
          if(kd<0) kd=0.0;

          Color D = m.diffuse;
          Color B = l->color(p);

          C = C + kd*D * B;
          //C += kd*D;
      }
      return C+m.ambient;
    }

  };

} // namespace rt

#endif // #define _RENDERER_H_
