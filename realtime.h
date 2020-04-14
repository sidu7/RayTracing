////////////////////////////////////////////////////////////////////////////////
// Temporary code.  Remove this from your raytracer.  This displays
// the contents of a scene file in realtime in a GLUT window.
////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <fstream>
#include <vector>
#include <stack>

#include <glbinding/gl/gl.h>
#include <glbinding/Binding.h>
using namespace gl;
#include <freeglut.h>

const float Radians = PI/180.0f;    // Convert degrees to radians

////////////////////////////////////////////////////////////////////////
// Shader programming class;  Encapsulates a OpenGL Shader.
////////////////////////////////////////////////////////////////////////
class ShaderProgram
{
public:
    int program;
    
    void CreateProgram() { program = glCreateProgram(); }
    void Use() { glUseProgram(program); }
    void Unuse() { glUseProgram(0); }
    void CreateShader(const std::string fname, const GLenum type);
    void LinkProgram();

};

////////////////////////////////////////////////////////////////////////
// Obj: encapsulates objects to be drawn; uses OpenGL's VAOs
////////////////////////////////////////////////////////////////////////
class Shape;
class Obj
{
public:
    MeshData* meshdata;
    Matrix4f modelTR;
    Material* material;
    Vector3f center;
    unsigned int vao;
    Obj(MeshData* m, const Matrix4f& tr, Material* b);
	Shape* shape;
    void draw();
    Vector3f Center() { return center; }
};

////////////////////////////////////////////////////////////////////////
// Realtime handles all realtime drawing/interaction
////////////////////////////////////////////////////////////////////////
class Ray;
class Intersection;
struct SkyDome
{
	std::vector<float> image;
	int width;
	int height;
	float radius;
	float *pBuffer;
	float* pUDist;
	float angle;

	void PreProcess()
	{
		angle = 0.0f;
		printf("w:%d h:%d\n", width, height);
		pBuffer = new float[width * (height + 1)];
		pUDist = &pBuffer[width * height];
		float* pSinTheta = new float[height];
		float angleFrac = PI / float(height);
		float theta = angleFrac * 0.5f;
		for (unsigned int i = 0; i < height; i++, theta += angleFrac)
			pSinTheta[i] = sin(theta);
		for (unsigned int i = 0, m = 0; i < width; i++, m += height)
		{
			float* pVDist = &pBuffer[m];
			unsigned int k = i * 3;
			pVDist[0] = 0.2126f * image[k + 0] + 0.7152f * image[k + 1] + 0.0722f * image[k + 2];
			pVDist[0] *= pSinTheta[0];
			for (unsigned int j = 1, k = (width + i) * 3; j < height; j++, k += width * 3)
			{
				float lum = 0.2126 * image[k + 0] + 0.7152 * image[k + 1] + 0.0722 * image[k + 2];
				pVDist[j] = pVDist[j - 1] + lum * pSinTheta[j];
			}
			if (i == 0)
				pUDist[i] = pVDist[height - 1];
			else
				pUDist[i] = pUDist[i - 1] + pVDist[height - 1];
		}
	}
};

class Realtime
{
public:
    bool nav;
    char motionkey;
    float speed;
	Color* imagePointer;
	unsigned int imageTexture;

    // Camera/viewing parameters
    Vector3f ambient;
    Vector3f eye;      // Position of eye for viewing scene
    Quaternionf orient;   // Represents rotation of -Z to view direction
    float ry;
    float front, back;
    float spin, tilt;
    float cDist;              // Distance from eye to center of scene
    //float lightSpin, lightTilt, lightDist;
	int max_passes;

    int mouseX, mouseY;
    bool shifted;
    bool leftDown;
    bool middleDown;
    bool rightDown;

    MeshData* sphMesh;
    MeshData* boxMesh;
    MeshData* cylMesh;

    ShaderProgram lighting;
	ShaderProgram raytraceout;

    int width, height;
    void setScreen(const int _width, const int _height) { width=_width;  height=_height; }
    void setCamera(const Vector3f& _eye, const Quaternionf& _o, const float _ry, const float _fd = 0.0f,const float _cr = 0.0f)
	{
		eye = _eye; orient = _o; ry = _ry; focal_distance = _fd; confusion_radius = _cr;
	}
    void setAmbient(const Vector3f& _a) { ambient = _a; }
    int setTexture(const int width, const int height, unsigned char* image);
    
    std::vector<Obj*> objs;
	std::vector<Shape*> shapes;
	std::vector<Shape*> bboxes;
    std::vector<Obj*> lights;

	KdBVH<float, 3, Shape*> Tree;

	// Depth of field parameters
	float focal_distance;
	float confusion_radius;

	// Image Based Lighting
	SkyDome skyDome;

	// CSG 
	bool readingCSG;
	std::stack<Shape*> shapeStack;

    Quaternionf ViewQuaternion() {
        Quaternionf q = angleAxis((tilt-90.0f)*Radians, Vector3f(1,0,0))
                       *orient.conjugate()
                       *angleAxis(spin*Radians, Vector3f(0,0,1));
        return q.conjugate();
    }

    Vector3f ViewDirection() {
        return ViewQuaternion().toRotationMatrix() * Vector3f(0.0f, 0.0f, -1.0f);
    }

    void DrawScene();
	void DrawOutput();
	void DrawFSQ();
    void ReshapeWindow(int w, int h);
    void KeyboardUp(unsigned char key, int x, int y);
    void KeyboardDown(unsigned char key, int x, int y);
    void MouseButton(int button, int state, int x, int y);
    void MouseMotion(int x, int y);
    
    void sphere(const Vector3f center, const float r, Material* mat);
    void box(const Vector3f base, const Vector3f diag, Material* mat);
    void cylinder(const Vector3f base, const Vector3f axis, const float radius, Material* mat);

	void triangleMesh(MeshData* meshdata);

    Realtime();
	void run(Color* image, int pass);

	Vector3f TraceRay(Ray& ray);
	Vector3f SampleBrdf(Vector3f wo, Vector3f N, Material* mat, float pd, float pr);
	Vector3f SampleLobe(Vector3f N, float c, float fi);
	Vector3f EvalScattering(Vector3f wo, Vector3f N, Vector3f wi, Material* mat, float t);
	float PdfBrdf(Vector3f wo, Vector3f N, Vector3f wi, float pd, float pr, float pt, Material* mat);
	float DTerm(Vector3f m, Vector3f N, float alpha);
	float GTerm(Vector3f v, Vector3f m, Vector3f N,float alpha);
	Vector3f FTerm(float LdotH, Material* mat);
	Vector3f EvalRadiance(Intersection& Q);
	Intersection SampleLight();
	Intersection SampleSphere(Vector3f center, float radius, Obj* light);
	float PdfLight(Intersection& L);
	float GeometryFactor(Intersection& P,Intersection& L);

	template<typename Iter>
	Iter select_randomly(Iter start, Iter end) {
		static std::random_device rd;
		static std::mt19937 gen(rd());
		std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
		std::advance(start, dis(gen));
		return start;
	}
};

