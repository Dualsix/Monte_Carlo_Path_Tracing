#include "Triangle.h"

//=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~//
// Triangle object.                                                       //
//                                                                        //
// The triangle object is defined by three vertices in R3.  This is a     //
// simple flat triangle with no normal vector interpolation.  The         //
// triangle structure is defined to accommodate the barycentric coord     //
// method of intersecting a ray with a triangle.                          //
//=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~//

Triangle::Triangle( const Vec3 &A_, const Vec3 &B_, const Vec3 &C_ )
    {
	float base;		// Length of the base of the triangle
	float height;	// Height of the triangle
	Vec3 N_pos;		// Normal with all values positive

	// Assign values to the corner points
	A = A_;
	B = B_;
	C = C_;

	// Compute the normal to plane of the triangle
	// Cross product between two vectors created by points fo the triangle

    N = Unit( ( C - B ) ^ ( A - B ) );

	// Compute the distance from origin to plane of triangle.
	// This equals to D on the plane coordinates

    d = -( A.x * N.x + A.y * N.y + A.z * N.z );

	// Computes the bounding box;

	// Initiallizes the box coordinates
    box.X.min = box.X.min  = A.x;
    box.Y.min = box.Y.min  = A.y;
    box.Z.min = box.Z.min  = A.z;
	// Check B coordinates
	if( B.x < box.X.min ) box.X.min = B.x;
	else if( B.x > box.X.max ) box.X.max = B.x;
	if( B.y < box.Y.min ) box.Y.min = B.y;
	else if( B.y > box.Y.max ) box.Y.max = B.y;
	if( B.z < box.Z.min ) box.Z.min = B.z;
	else if( B.z > box.Z.max ) box.Z.max = B.z;
	// Check C coordinates
	if( C.x < box.X.min ) box.X.min = C.x;
	else if( C.x > box.X.max ) box.X.max = C.x;
	if( C.y < box.Y.min ) box.Y.min = C.y;
	else if( C.y > box.Y.max ) box.Y.max = C.y;
	if( C.z < box.Z.min ) box.Z.min = C.z;
	else if( C.z > box.Z.max ) box.Z.max = C.z;

	// Find the dominant axis
	// It is the axis of the longer coordinate of N

	// Compute absolute value for all components of the normal
	N_pos = N;
	N_pos.x = fabs( N_pos.x );
	N_pos.y = fabs( N_pos.y );
	N_pos.z = fabs( N_pos.z );
	
	if( N_pos.x >= N_pos.y && N_pos.x >= N_pos.z )
		axis = 0;
	else if( N_pos.y >= N_pos.x && N_pos.y >= N_pos.z )
		axis = 1;
	else if( N_pos.z >= N_pos.x && N_pos.z >= N_pos.y )
		axis = 2;

	// Compute Inverse of barycentric coord transform.

	// Compute the non inverse matrix 
    M.m [0][0] = A.x;
    M.m [0][1] = B.x;
    M.m [0][2] = C.x;
    M.m [1][0] = A.y;
    M.m [1][1] = B.y;
    M.m [1][2] = C.y;
    M.m [2][0] = A.z;
    M.m [2][1] = B.z;
    M.m [2][2] = C.z;

	// Project values to the dominant axis
	M.m [axis][0] = 1.0f;
	M.m [axis][1] = 1.0f;
	M.m [axis][2] = 1.0f;

	// Invert the matrix
	M = ( 1 / det( M ) ) * Transpose( Adjoint( M ) );

	// Compute the area

	// First compute the base of the triangle
	base = (float) dist( B, A );

	// Then compute the height of the triangle
	// It is computed as an point to line distance
	height = (float) Length( ( B - A ) ^ ( A - C ) ) / (float) Length( B - A );

	// Therefore, the area is:
	area = base * height * 0.5f;

	center = (A + B + C) / 3.0;

    next = NULL;
    }

Object *Triangle::ReadString( const char *params ) // Read params from string.
    {
    float ax, ay, az, bx, by, bz, cx, cy, cz;
    if( sscanf( params, "triangle (%f,%f,%f) (%f,%f,%f) (%f,%f,%f)", 
        &ax, &ay, &az, &bx, &by, &bz, &cx, &cy, &cz ) == 9 )
        return new Triangle( Vec3( ax, ay, az ), Vec3( bx, by, bz ), Vec3( cx, cy, cz ) );
    return NULL;
    }

Box3 Triangle::GetBounds() const // Return pre-computed box.
    {
    return box;
    }

bool Triangle::Intersect( const Ray &ray, HitGeom &hitgeom ) const
    {
	Plane Pl;	// Plane supporting the triangle
	float dist;	// Distance from the origin of the ray to the plane Pl
	Vec3 P;		// Point of intersection within the triangle
	Vec3 Bar;	// Barycentric coordinates

	// Define the plane of the triangle
	Pl = Plane( N.x, N.y, N.z, d );

	// Find the distance between the origin of the ray and the intersection
	//  with the supporting plane of the triangle
	dist = (float) Pl.Intersect( ray );

	if( ( dist > 0.0f ) && ( dist < hitgeom.distance ) )
		{
		// The object is behind the ray

		// Find the point P on the plane containing the triangle
		P = ray.origin + ( dist * ( ray.direction ) );

		// Project the coordinates of the point to a 2D plane
		// Drop the coordinate of the dominant axis of the triangle
		if( axis == 0 )		 P.x = 1.0f;
		else if( axis == 1 ) P.y = 1.0f;
		else if( axis == 2 ) P.z = 1.0f;

		// Find the barycentric coordinates
		Bar = M * ( Vec3( P.x, P.y, P.z ) );

		// In order to have an intersection all barycentric coordinates have to
		// be between 0 and 1.
		if( Bar.x >= 0 && Bar.x <= 1 && Bar.y >= 0 && Bar.y <= 1 && Bar.z >= 0 && Bar.z <= 1 )
			{
			// There is an intersection, fill all hitgeom fields
			hitgeom.distance = dist;
			hitgeom.normal	 = N;
			hitgeom.origin	 = ray.origin;
			hitgeom.point	 = P;
			return true;
			}
		}
    
	// There is not an intersection
	return false;
}


Sample Triangle::GetSample( const Vec3 &P, const Vec3 &N_point ) const
{
	float x, y;			// Origin of the little squares used for the stratisfied sampling
	float s, t;			// Coordinates of the sample on the unit square
	float s_sqrt;		// Square root of s
	float cos_theta;	// Cosine of the angle formed by the normal of the triangle and the
						//  inverse of the current sample vector.
	float rsqr;			// Distance between the sample and the point P

	Sample sample;

	// Generates two random values for s and t coordinates
	s = (float) rand( 0.0, 1.0 );
	t = (float) rand( 0.0, 1.0 );

	// Compute the square root of s
	s_sqrt = (float) sqrt( s );

    // Find the sample on the triangle itself using an area
	// preserving map from the square to a triangle
	sample.P = ( 1 - s_sqrt )		  * A +
				   ( s_sqrt * ( 1 - t ) ) * B +
				   ( s_sqrt * t )		  * C;

	// Computes the absolute value of cosine of the angle between the
	//  normal of the triangle and the inverse sample vector
	cos_theta = (float) fabs( N * Unit( P - sample.P ) );
	// Calcule the distance between the center of the triangle and the point P
	rsqr = (float) LengthSquared( center - P );
	// Assigns the weight
	sample.w = area * cos_theta / rsqr;
	// Limit weight to twopi
	if (sample.w > TwoPi)
		sample.w = TwoPi;

	return sample;
}