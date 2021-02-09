/**
@file Sphere.cpp
*/
#include <cmath>
#include "Sphere.h"

void
rt::Sphere::draw( Viewer& /* viewer */ )
{
  Material m = material;
  // Taking care of south pole
  glBegin( GL_TRIANGLE_FAN );
  glColor4fv( m.ambient );
  glMaterialfv(GL_FRONT, GL_DIFFUSE, m.diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR, m.specular);
  glMaterialf(GL_FRONT, GL_SHININESS, m.shinyness );
  Point3 south_pole = localize( -90, 0 );
  glNormal3fv( getNormal( south_pole ) );
  glVertex3fv( south_pole );
  for ( int x = 0; x <= NLON; ++x )
    {
      Point3 p = localize( -90 + 180/NLAT, x * 360 / NLON );
      glNormal3fv( getNormal( p ) );
      glVertex3fv( p );
    }
  glEnd();
  // Taking care of in-between poles
  for ( int y = 1; y < NLAT - 1; ++y )
    {
      glBegin( GL_QUAD_STRIP);
      glColor4fv( m.ambient );
      glMaterialfv(GL_FRONT, GL_DIFFUSE, m.diffuse);
      glMaterialfv(GL_FRONT, GL_SPECULAR, m.specular);
      glMaterialf(GL_FRONT, GL_SHININESS, m.shinyness );
      for ( int x = 0; x <= NLON; ++x )
        {
          Point3 p = localize( -90 + y*180/NLAT,     x * 360 / NLON );
          Point3 q = localize( -90 + (y+1)*180/NLAT, x * 360 / NLON );
          glNormal3fv( getNormal( p ) );
          glVertex3fv( p );
          glNormal3fv( getNormal( q ) );
          glVertex3fv( q );
        }
      glEnd();
    }
  // Taking care of north pole
  glBegin( GL_TRIANGLE_FAN );
  glColor4fv( m.ambient );
  glMaterialfv(GL_FRONT, GL_DIFFUSE, m.diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR, m.specular);
  glMaterialf(GL_FRONT, GL_SHININESS, m.shinyness );
  Point3 north_pole = localize( 90, 0 );
  glNormal3fv( getNormal( north_pole ) );
  glVertex3fv( north_pole );
  for ( int x = NLON; x >= 0; --x )
    {
      Point3 p = localize( -90 + (NLAT-1)*180/NLAT, x * 360 / NLON );
      glNormal3fv( getNormal( p ) );
      glVertex3fv( p );
    }
  glEnd();
}

rt::Point3
rt::Sphere::localize( Real latitude, Real longitude ) const
{
  static const Real conv_deg_rad = 2.0 * M_PI / 360.0;
  latitude  *= conv_deg_rad;
  longitude *= conv_deg_rad;
  return center 
    + radius * Point3( cos( longitude ) * cos( latitude ),
                       sin( longitude ) * cos( latitude ),
                       sin( latitude ) );
}

rt::Vector3
rt::Sphere::getNormal( Point3 p )
{
  Vector3 u = p - center;
  Real   l2 = u.dot( u );
  if ( l2 != 0.0 ) u /= sqrt( l2 );
  return u;
}

rt::Material
rt::Sphere::getMaterial( Point3 /* p */ )
{
  return material; // the material is constant along the sphere.
}

rt::Real
rt::Sphere::rayIntersection( const Ray& ray, Point3& p )
{
  //3.1
  //centre de la sphère
  Point3 c = center;
  //rayon de la sphère
  rt::Real r = radius;

  //origine du rayon
  Point3 o = ray.origin;
  //direction du rayon
  Vector3 u = ray.direction;

  //calcul distance entre c et [o,u)
  
    //calcul distance entre c et o
  Vector3 co = o-c;
  Vector3 v1 = co.dot(u)*u;
  v1 = co - v1;
  rt::Real d = v1.norm();

    //calcul dist²
  rt::Real d2 = d*d; 
  //calcul r² 
  rt::Real r2 = r*r;
  


  if(d2<r2){
    //calculer les deux intesections t_1 et t_2
    Point3 t_1,t_2;
    rt::Real b = 2*(u.dot(co));
    rt::Real c = (co.dot(co)-r2);
    
    rt::Real delta = b*b-4*c;
    
    
    if(delta>0){
      //t_1 = (-b-sqrt(delta))/2a;
      rt::Real t1 = ((-b)-sqrt(delta))/2;
      //t_2 = (-b+sqrt(delta))/2a;
      rt::Real t2 = ((-b)+sqrt(delta))/2;
      t_1 = o + u*t1;
      t_2 = o + u*t2;


    //recherche de la bonne intersection
    if(t2<0.0f){
      return 1.0f;
    }
    else if(t1<0.0f){
      p = t_2;
      return -1.0f;
    }else{
      p = t_1;
      return -1.0f;
    }
    
    

    
    }
    
    
    
  }

  return 1.0f;
}
