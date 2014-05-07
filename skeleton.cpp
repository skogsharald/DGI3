#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModel.h"
#include "math.h"

using namespace std;
using glm::vec3;
using glm::vec2;
using glm::ivec2;
using glm::mat3;



struct Pixel 
{ 
	int x; 
	int y; 
	float zinv;
	vec3 pos3d;   
}; 

struct Vertex 
{ 
	vec3 position;
};



// ----------------------------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
SDL_Surface* screen;
int t;
vector<Triangle> triangles;
vec3 cameraPos( 0, 0, -3.001 );
mat3 R; 
float yaw = 0; // Yaw angle controlling camera rotation around y-axis  
float f = SCREEN_HEIGHT; // Focal length should be the same as width and height
vec3 currentColor;
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];
// Light position variables
 
vec3 lightPower = 14.f*vec3( 1, 1, 1 );
vec3 lightPos(0,-0.5,-0.7);
vec3 indirectLightPowerPerArea = 0.5f*vec3( 1, 1, 1 );
vec3 currentNormal; 
vec3 currentReflectance;

int currentVertex;




// ----------------------------------------------------------------------------
// FUNCTIONS

void Update();
void Draw();
void VertexShader( const Vertex& v, Pixel& p );
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result );
void DrawLineSDL( SDL_Surface* surface, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( const vector<vec3>& vertices );
void UpdateR();
void ComputePolygonRows( 
const vector<Pixel>& vertexPixels, 
vector<Pixel>& leftPixels, 
vector<Pixel>& rightPixels 
); 
void DrawPolygonRows( 
const vector<Pixel>& leftPixels, 
const vector<Pixel>& rightPixels 
); 
void DrawPolygon( const vector<Vertex>& vertices );
void PixelShader( const Pixel& p );




int main( int argc, char* argv[] )
{
	LoadTestModel( triangles );
	screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT );
	t = SDL_GetTicks();	// Set start value for timer.
	UpdateR(); 
	 

	while( NoQuitMessageSDL() )
	{	
		Update();
		Draw();

	}

	SDL_SaveBMP( screen, "screenshot.bmp" );
	return 0;
}

void Update()
{
	// Compute frame time:
	int t2 = SDL_GetTicks();
	float dt = float(t2-t);
	t = t2;
	cout << "Render time: " << dt << " ms." << endl;

	Uint8* keystate = SDL_GetKeyState(0);
	vec3 forward( R[2][0], R[2][1], R[2][2] );

	if( keystate[SDLK_UP] )
		cameraPos += forward*0.1f;

	if( keystate[SDLK_DOWN] )
		cameraPos -= forward*0.1f;

	if( keystate[SDLK_RIGHT] )
		yaw += 0.03f;

	if( keystate[SDLK_LEFT] )
		yaw -= 0.03f;

	if( keystate[SDLK_RSHIFT] )
		;

	if( keystate[SDLK_RCTRL] )
		;

	if( keystate[SDLK_w] )
		;

	if( keystate[SDLK_s] )
		;

	if( keystate[SDLK_d] )
		;

	if( keystate[SDLK_a] )
		;

	if( keystate[SDLK_e] )
		;

	if( keystate[SDLK_q] )
		;

	UpdateR();

}

void UpdateR()
{
	vec3 a(cos(yaw), 0, -sin(yaw));
	vec3 b(0, 1, 0);
	vec3 c(sin(yaw), 0, cos(yaw));
	R = mat3(a, b, c);
}

void Draw() 
{ 	
	// Reset buffer
	for( int y=0; y<SCREEN_HEIGHT; ++y ){ 
		for( int x=0; x<SCREEN_WIDTH; ++x ){ 
			depthBuffer[y][x] = 0;
		}
	}

	SDL_FillRect( screen, 0, 0 ); 
	if( SDL_MUSTLOCK(screen) ) 
		SDL_LockSurface(screen); 
	for( int i=0; i<triangles.size(); ++i ) 
	{ 
		currentColor = triangles[i].color;
		currentVertex = i; 
		vector<Vertex> vertices(3); 

		vertices[0].position = triangles[i].v0; 
		vertices[1].position = triangles[i].v1;
		vertices[2].position = triangles[i].v2; 

		currentNormal = triangles[i].normal;
		currentReflectance = triangles[i].color; 
		DrawPolygon( vertices );
		

		//DrawPolygonEdges( vertices ); 

	}
	// vec3 color(1,1,1); 
	// vector<vec3> test1(3);
	// vector<vec3> test2(3);
	// test1[0] = triangles[0].v0;
	// //test1[1] = triangles[0].v1;
	// //test1[2] = triangles[0].v2;
	// test2[0] = triangles[7].v1;
	// //test2[1] = triangles[1].v1;
	// //test2[2] = triangles[1].v2;
	// ivec2 testPos1;
	// ivec2 testPos2;
	// VertexShader(test1[0], testPos1);
	// VertexShader(test2[0], testPos2);
	// DrawLineSDL(screen, testPos1, testPos2, color);

	if ( SDL_MUSTLOCK(screen) ) 
		SDL_UnlockSurface(screen); 
	SDL_UpdateRect( screen, 0, 0, 0, 0 ); 
}

void VertexShader( const Vertex& v, Pixel& p )
{



	// Translate the points according to equations 3, 4, and 5
	vec3 p_prim = (v.position - cameraPos) * R;
	p.zinv = 1/p_prim.z;
	p.x = f*(p_prim.x/p_prim.z) + SCREEN_WIDTH/2;
	p.y = f*(p_prim.y/p_prim.z) + SCREEN_HEIGHT/2;
	p.pos3d = v.position;  
	
}
 
void PixelShader( const Pixel& p ){
	int x = p.x; 
	int y = p.y; 
	if( p.zinv > depthBuffer[y][x] ) 
	{ 
		// Calculate the direct light
		vec3 n_hatt = currentNormal;
		vec3 r_hatt = glm::normalize(lightPos - p.pos3d);

		float r = glm::length(lightPos - p.pos3d);

		float dot_product = glm::dot(r_hatt, n_hatt);

		float divisor = 4 * M_PI * r * r;

		vec3 directLight = lightPower*(std::max(dot_product, 0.0f))/divisor;
		vec3 illumination = currentReflectance * (directLight + indirectLightPowerPerArea);

		depthBuffer[y][x] = p.zinv; 
		PutPixelSDL( screen, x, y, illumination ); 
	} 
 
}

void Interpolate( Pixel a, Pixel b, vector<Pixel>& result ) 
{ 
	int N = result.size();
	float xStep = (b.x - a.x)/ (float) max(N-1,1);
	float yStep = (b.y - a.y)/ (float) max(N-1,1);
	float zStep = (b.zinv - a.zinv)/ float(max(N-1,1));

	// Convert in order to interpolate correctly
	b.pos3d *= b.zinv;
	a.pos3d *= a.zinv;
	vec3 pos3dStep = (b.pos3d - a.pos3d)/ float(max(N-1, 1));

	for( int i=0; i<N; ++i ) 
	{ 
		result[i].x = a.x + xStep*i;
		result[i].y = a.y + yStep*i;
		result[i].zinv = a.zinv + zStep*i;
		result[i].pos3d = a.pos3d + pos3dStep*(float)i;
		// Convert back
		result[i].pos3d /= result[i].zinv;


	} 
}

void DrawLineSDL( SDL_Surface* surface, Pixel a, Pixel b, vec3 color )
{

	Pixel delta;
	delta.x = glm::abs( a.x - b.x );
	delta.y = glm::abs( a.y - b.y ); 
	int pixels = glm::max( delta.x, delta.y ) + 1;

	vector<Pixel> line(pixels);
	Interpolate(a, b, line);
	
	for(int i = 0; i < line.size(); ++i)
	{	
		if(line[i].x >= 0 && line[i].x <= SCREEN_WIDTH && line[i].y >= 0 && line[i].y <= SCREEN_HEIGHT)
		{
			PixelShader(line[i]);
		}
	}
}

void DrawPolygon( const vector<Vertex>& vertices ) 
{ 
	int V = vertices.size(); 
	vector<Pixel> vertexPixels( V );
	for( int i=0; i<V; ++i ) 
		VertexShader( vertices[i], vertexPixels[i] ); 
	 
	vector<Pixel> leftPixels; 
	vector<Pixel> rightPixels;
	ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
	DrawPolygonRows( leftPixels, rightPixels );
} 


void ComputePolygonRows( const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, 
	vector<Pixel>& rightPixels )
{

	int minY = numeric_limits<int>::max();
	int maxY = -numeric_limits<int>::max();
	// 1. Find max and min y-value of the polygon 
	// and compute the number of rows it occupies. 
	for(int i = 0; i< vertexPixels.size(); ++i)
	{
		minY = min(vertexPixels[i].y, minY);
		maxY = max(vertexPixels[i].y, maxY);
	}
	int rows = maxY - minY + 1;
	// 2. Resize leftPixels and rightPixels 
	// so that they have an element for each row. 
	leftPixels.resize(rows);
	rightPixels.resize(rows);


	// 3. Initialize the x-coordinates in leftPixels 
	// to some really large value and the x-coordinates 
	// in rightPixels to some really small value.
	for( int i=0; i<rows; ++i ) 
	{ 	
		leftPixels[i].x = +numeric_limits<int>::max(); 
		rightPixels[i].x = -numeric_limits<int>::max(); 
	} 

	// 4. Loop through all edges of the polygon and use 
	// linear interpolation to find the x-coordinate for 
	// each row it occupies. Update the corresponding 
	// values in rightPixels and leftPixels.
	for( int i = 0; i < vertexPixels.size(); ++i){

		int j = (i+1)%vertexPixels.size(); // The next vertex
		int pixels = abs(vertexPixels[i].y - vertexPixels[j].y) + 1;
		vector<Pixel> line(pixels);

		Interpolate(vertexPixels[i], vertexPixels[j], line);
		for(int k = 0; k < line.size(); ++k){
			int rowNum = line[k].y - minY;
			if(leftPixels[rowNum].x > line[k].x){
				leftPixels[rowNum] = line[k];
			}
			if(rightPixels[rowNum].x < line[k].x){
				rightPixels[rowNum] = line[k];

			}

		}
	} 

}

void DrawPolygonRows( const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels )
{
	for(int i = 0; i < leftPixels.size(); ++i)
	{	
		DrawLineSDL(screen, leftPixels[i], rightPixels[i], currentColor);
	}
}
 
// void DrawPolygonEdges( const vector<vec3>& vertices ) 
// { 
// 	int V = vertices.size(); 
// 	// Transform each vertex from 3D world position to 2D image position: 
// 	vector<ivec2> projectedVertices( V ); 
// 	for( int i=0; i<V; ++i ) 
// 	{ 
// 		VertexShader( vertices[i], projectedVertices[i] ); 
// 	} 
// 	// Loop over all vertices and draw the edge from it to the next vertex: 
// 	for( int i=0; i<V; ++i ) 
// 	{ 
// 		int j = (i+1)%V; // The next vertex 
// 		vec3 color( 1, 1, 1 ); 
// 		DrawLineSDL( screen, projectedVertices[i], projectedVertices[j], color ); 
// 	} 
// }
