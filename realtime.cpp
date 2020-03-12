////////////////////////////////////////////////////////////////////////////////
// Temporary code.  Remove this from your raytracer.  This displays
// the contents of a scene file in realtime in a GLUT window.
////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <fstream>
#include <vector>

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"
#include "shapes.h"


// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::mt19937_64 RNGen;
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].


// Write the image as a HDR(RGBE) image.  
#include "rgbe.h"

// Stupid C++ needs callbacks to be static functions.
static Realtime* globalRealtime = nullptr;
void CBDrawScene()  { globalRealtime->DrawScene(); }
void CBDrawOutput() { globalRealtime->DrawOutput(); }
void CBReshapeWindow(int w, int h)  { globalRealtime->ReshapeWindow(w,h); }
void CBKeyboardDown(unsigned char key, int x, int y)  { globalRealtime->KeyboardDown(key, x, y); }
void CBKeyboardUp(unsigned char key, int x, int y)  { globalRealtime->KeyboardUp(key, x, y); } 
void CBMouseButton(int button, int state, int x, int y)  { globalRealtime->MouseButton(button, state, x, y); } 
void CBMouseMotion(int x, int y)  { globalRealtime->MouseMotion(x,y); }
void CBAnimate(int value)
{
    glutTimerFunc(30, CBAnimate, 1);
    // atime = 360.0*glutGet(GLUT_ELAPSED_TIME)/12000;
    glutPostRedisplay();
}

unsigned int MakeVAO(MeshData* meshdata)
{
    unsigned int vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    
    if (meshdata && meshdata->vertices.size() == 0) {
        std::cerr << "Missing meshdata->vertices in MakeVAO\n";
        exit(-1); }
    if (meshdata && meshdata->triangles.size() == 0) {
        std::cerr << "Missing meshdata->triangles in MakeVAO\n";
        exit(-1); }
    
    GLuint Pbuff;
    glGenBuffers(1, &Pbuff);
    glBindBuffer(GL_ARRAY_BUFFER, Pbuff);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*11*meshdata->vertices.size(), 
                 &(meshdata->vertices[0].pnt.x()), GL_STATIC_DRAW);
    
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 11*sizeof(float), (void*)0);
    
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 11*sizeof(float), (void*)(3*sizeof(float)));
    
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 11*sizeof(float), (void*)(6*sizeof(float)));
    
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 11*sizeof(float), (void*)(8*sizeof(float)));
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    
    GLuint Ibuff;
    glGenBuffers(1, &Ibuff);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, Ibuff);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*3*meshdata->triangles.size(),
                 &(meshdata->triangles[0].x()), GL_STATIC_DRAW);
    
    glBindVertexArray(0);

    return vao;
}

MeshData* SphMesh()
{
    MeshData* meshdata = new MeshData();
    unsigned int n = 20;
    float d = 2.0f*PI/float(n*2);
    for (unsigned int i=0;  i<=n*2;  i++) {
        float s = i*2.0f*PI/float(n*2);
        for (unsigned int j=0;  j<=n;  j++) {
            float t = j*PI/float(n);
            float x = cos(s)*sin(t);
            float y = sin(s)*sin(t);
            float z = cos(t);
            meshdata->vertices.push_back(VertexData(Vector3f(x,y,z),
                                                    Vector3f(x,y,z),
                                                    Vector2f(s/(2*PI), t/PI),
                                                    Vector3f(sin(s), cos(s), 0.0)));
            if (i>0 && j>0) {
                meshdata->triangles.push_back(TriData((i-1)*(n+1) + (j-1), 
                                                      (i-1)*(n+1) + (j  ), 
                                                      (i  )*(n+1) + (j  )));
                meshdata->triangles.push_back(TriData((i-1)*(n+1) + (j-1),
                                                      (i  )*(n+1) + (j  ),
                                                      (i  )*(n+1) + (j-1))); } } }
    return meshdata;
}

MeshData* BoxMesh()
{
    Matrix4f face[6] = {
        Matrix4f::Identity(),
        rotate(180.0f*Radians, Vector3f(1.0f, 0.0f, 0.0f)),
        rotate( 90.0f*Radians, Vector3f(1.0f, 0.0f, 0.0f)),
        rotate(-90.0f*Radians, Vector3f(1.0f, 0.0f, 0.0f)),
        rotate( 90.0f*Radians, Vector3f(0.0f, 1.0f, 0.0f)),
        rotate(-90.0f*Radians, Vector3f(0.0f, 1.0f, 0.0f))};
       
    Matrix4f half = translate(Vector3f(0.5f, 0.5f, 0.5f))*scale(Vector3f(0.5f, 0.5f, 0.5f));
    MeshData* meshdata = new MeshData();
    for (unsigned int f=0;  f<6;  f++) {
        Matrix4f m4 = half*face[f];
        Matrix3f m3 = m4.block<3,3>(0,0);
        for (unsigned int i=0;  i<2;  i++) {
            for (unsigned int j=0;  j<2;  j++) {
              Vector4f p = m4*Vector4f(float(2*i)-1.0f, float(2*j)-1.0f, 1.0f, 1.0f);
              Vector3f tnrm = m3*Vector3f(0.0f, 0.0f, 1.0f);
              Vector3f ttan = m3*Vector3f(1.0, 0.0, 0.0);
              meshdata->vertices.push_back(VertexData(Vector3f(p[0], p[1], p[2]),
                                                      Vector3f(tnrm[0], tnrm[1], tnrm[2]),
                                                      Vector2f(float(i), float(j)),
                                                      Vector3f(ttan[0], ttan[1], ttan[2])));
              meshdata->triangles.push_back(TriData(4*f+0, 4*f+1, 4*f+3));
              meshdata->triangles.push_back(TriData(4*f+0, 4*f+3, 4*f+2)); } } }
    return meshdata;
}

MeshData* CylMesh()
{
    MeshData* meshdata = new MeshData();
    unsigned int n = 20;
    float d = 2.0f*PI/float(n*2);
    for (unsigned int i=0;  i<=n;  i++) {
        float s = i*2.0f*PI/float(n);
        float x = cos(s);
        float y = sin(s);
        
        meshdata->vertices.push_back(VertexData(Vector3f(x, y, 0.0f),
                                                Vector3f(x, y, 0.0f),
                                                Vector2f(s/(2*PI), 0.0f),
                                                Vector3f(-sin(s), cos(s), 0.0f)));

        meshdata->vertices.push_back(VertexData(Vector3f(x, y, 1.0f),
                                                Vector3f(x, y, 0.0f),
                                                Vector2f(s/(2*PI), 0.0f),
                                                Vector3f(-sin(s), cos(s), 0.0f)));

        if (i>0) {
            meshdata->triangles.push_back(TriData((i-1)*2+1, (i-1)*2, (i  )*2));
            meshdata->triangles.push_back(TriData((i-1)*2+1, (i  )*2, (i  )*2+1)); } }
    return meshdata;
}

////////////////////////////////////////////////////////////////////////
// Shader programming class;  Encapsulates a OpenGL Shader.
////////////////////////////////////////////////////////////////////////
void ShaderProgram::CreateShader(const std::string fname, const GLenum type)
{
    // Read a file into a string
    std::ifstream f;
    f.open(fname, std::ios_base::binary); // Open
    f.seekg(0, std::ios_base::end);       // Position at end
    int length = f.tellg();               // to get the length
    
    char* src = new char [length+1];  // Create buffer of needed length
    f.seekg (0, std::ios_base::beg);      // Position at beginning
    f.read (src, length);             //   to read complete file
    f.close();                            // Close
    
    src[length] = char(0);            // Finish with a NULL
    
    // Create a shader, attach, send it the source, and compile it.
    int shader = glCreateShader(type);
    glAttachShader(program, shader);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);
    
    // Get the compilation status
    int status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    
    // If compilation status is not OK, get and print the log message.
    if (status != 1) {
        int length;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
        char* buffer = new char[length];
        glGetShaderInfoLog(shader, length, nullptr, buffer);
        printf("Compile log(%s):\n%s\n", type==GL_VERTEX_SHADER?"Vertex":"Fragment", buffer);
        delete buffer;
    }
}

void ShaderProgram::LinkProgram()
{
    // Link program and check the status
    glLinkProgram(program);
    int status;
    glGetProgramiv(program, GL_LINK_STATUS, &status);
    
    // If link failed, get and print log
    if (status != 1) {
        int length;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
        char* buffer = new char[length];
        glGetProgramInfoLog(program, length, nullptr, buffer);
        printf("Link log:\n%s\n", buffer);
        delete buffer;
    }
}


void applyMaterial(Material* mat, const unsigned int program)
{
    int loc = glGetUniformLocation(program, "Kd");
    glUniform3fv(loc, 1, &mat->Kd[0]);
    
    loc = glGetUniformLocation(program, "Ks");
    glUniform3fv(loc, 1, &mat->Ks[0]);
    
    loc = glGetUniformLocation(program, "alpha");
    glUniform1f(loc, mat->alpha);
    
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, mat->texid);
    loc = glGetUniformLocation(program, "tex");
    glUniform1i(loc, 0);
    
    loc = glGetUniformLocation(program, "emitter");
    glUniform1i(loc, 0);
}

////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
void applyLight(Material* mat, const unsigned int program)
{
    Vector3f Z;
    
    int loc = glGetUniformLocation(program, "Kd");
    glUniform3fv(loc, 1, &mat->Kd[0]);
    
    loc = glGetUniformLocation(program, "emitter");
    glUniform1i(loc, 1);
}

////////////////////////////////////////////////////////////////////////
// Obj: encapsulates objects to be drawn; uses OpenGL's VAOs
////////////////////////////////////////////////////////////////////////
Obj::Obj(MeshData* m, const Matrix4f& tr, Material* b)
    : meshdata(m), modelTR(tr), material(b)
{
    Vector4f sum(0,0,0,0);
    //for (int i=0;  i<meshdata->vertices.size();  i++)
    //    sum += modelTR*Vector4f(v.pnt[0], v.pnt[1], v.pnt[2], 1.0);
    for (auto v : meshdata->vertices) // C++11 for loop
        sum += modelTR*Vector4f(v.pnt[0], v.pnt[1], v.pnt[2], 1.0);

    center = (sum/meshdata->vertices.size()).block<3,1>(0,0); 
    
    std::cout << "center: " << center.transpose() << std::endl;
    
    vao = MakeVAO(meshdata);
}

void Obj::draw()
{
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, 3*meshdata->triangles.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

////////////////////////////////////////////////////////////////////////
// Realtime handles all realtime drawing/interaction
////////////////////////////////////////////////////////////////////////

// Constructor for Realtime.  Initializes OpenGL, GLUT,as well as the
// data elements of the class.
Realtime::Realtime()
{   
    // Initialize the OpenGL bindings
    glbinding::Binding::initialize(false);

    globalRealtime = this;
    // Initialize GLUT
    int argc = 0;
    char* argv;
    
    glutInit(&argc, &argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitContextVersion (3, 3);
    glutInitContextProfile(GLUT_COMPATIBILITY_PROFILE);

    glutInitWindowSize(200, 200);
    glutCreateWindow("Class Framework");
    glutSetOption((GLenum)GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);

    glutIgnoreKeyRepeat(true);
    glutDisplayFunc(&CBDrawScene);
    glutReshapeFunc(&CBReshapeWindow);
    glutKeyboardFunc(&CBKeyboardDown);
    glutKeyboardUpFunc(&CBKeyboardUp);
    glutMouseFunc(&CBMouseButton);
    glutMotionFunc(&CBMouseMotion); 
    glutTimerFunc(30, CBAnimate, 1);

    printf("OpenGL Version: %s\n", glGetString(GL_VERSION));
    printf("GLSL Version: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));
    printf("Rendered by: %s\n", glGetString(GL_RENDERER));
    fflush(stdout);
    
    // Create the shader program
    lighting.CreateProgram();
    lighting.CreateShader("realtime.vert", GL_VERTEX_SHADER);
    lighting.CreateShader("realtime.frag", GL_FRAGMENT_SHADER);

    glBindAttribLocation(lighting.program, 0, "vertex");
    glBindAttribLocation(lighting.program, 1, "vertexNormal");
    glBindAttribLocation(lighting.program, 2, "vertexTexture");
    glBindAttribLocation(lighting.program, 3, "vertexTangent");
    lighting.LinkProgram();

    // Several generic meshes which can be transfofrmed to *any* sphere, box, or cylinder.
    sphMesh = SphMesh();
    boxMesh = BoxMesh();
    cylMesh = CylMesh();

    // Initialize various member attributes
    //materials.push_back(new Material());

    nav = false;
    spin = 0.0f;
    tilt = 90.0f;
    speed = 0.05f;
    front = 0.1f;
    back = 1000.0f;

    shifted = false;
    leftDown = false;
    middleDown = false;
    rightDown = false;
    motionkey = 0;
	
	// Create the output shader program
	raytraceout.CreateProgram();
	raytraceout.CreateShader("raytraceout.vert", GL_VERTEX_SHADER);
	raytraceout.CreateShader("raytraceout.frag", GL_FRAGMENT_SHADER);

	glBindAttribLocation(raytraceout.program, 0, "vertex");
	glBindAttribLocation(raytraceout.program, 1, "vertexNormal");
	glBindAttribLocation(raytraceout.program, 2, "vertexTexture");
	glBindAttribLocation(raytraceout.program, 3, "vertexTangent");
	raytraceout.LinkProgram();
}

// This function enters the event loop.
void Realtime::run(Color* image, int pass)
{
	// Bounding boxes
	for(Shape* shape : shapes)
	{
		Box* b = new Box(shape->Bounding_Box().min(),
			shape->Bounding_Box().diagonal());
		b->object = shape->object;
		bboxes.push_back(b);
	}
	
	Tree.init(shapes.begin(), shapes.end());
	
	max_passes = 4096;

    cDist = eye.norm();
	imagePointer = image;
    glutReshapeWindow(width, height);
    glutMainLoop();
}
Vector3f Realtime::TraceRay(Ray& ray)
{
	Vector3f C = Vector3f(0.0f,0.0f,0.0f);
	Vector3f W = Vector3f(1.0f,1.0f,1.0f);
	Intersection P;
	const float RussianRoulette = 0.8f;
	const float Epsilon = pow(10,-6);
	Vector3f wo = -ray.D;
	Minimizer minimizer(ray, &P);
	BVMinimize(Tree, minimizer);

	if (P.object == nullptr)
	{
		return C;
	}
	if (P.object->material->isLight())
	{
		return EvalRadiance(P);
	}

	while (myrandom(RNGen) <= RussianRoulette)
	{
		Vector3f N = P.N.normalized();
		float kd = P.object->material->Kd.norm();
		float ks = P.object->material->Ks.norm();
		float s = kd + ks;
		float pd = kd / s;
		float pr = ks / s;
		
		// Explicit
		Intersection L = SampleLight();
		float p_some = PdfLight(L) / GeometryFactor(P, L);
		Vector3f wiL = (L.P - P.P).normalized();
		Ray new_ray3(P.P, wiL);
		float q_some = PdfBrdf(wo, N, wiL, pd, pr, P.object->material->alpha) * RussianRoulette;
		float w_mis = p_some * p_some / (p_some * p_some + q_some * q_some);
		
		Intersection I;
		Minimizer mini3(new_ray3, &I);
		BVMinimize(Tree, mini3);
		
		if(p_some > 0.0f && I.object != nullptr && I.object == L.object)
		{
			Vector3f ff = EvalScattering(wo, N, wiL, P.object->material);
			C += W.cwiseProduct((ff * w_mis / p_some).cwiseProduct(EvalRadiance(L)));
		}

		// Implicit		
		Vector3f wi = SampleBrdf(wo,N,P.object->material, pd);
		wi.normalize();
		Ray new_ray(P.P, wi);
		Intersection Q;
		Minimizer mini2(new_ray, &Q);
		BVMinimize(Tree, mini2);

		if (Q.object == nullptr)
		{
			break;
		}

		Vector3f f = EvalScattering(wo,N, wi,P.object->material);
		float p = PdfBrdf(wo, N, wi,pd,pr, P.object->material->alpha) * RussianRoulette;

		if (p < Epsilon)
		{
			break;
		}

		W = W.cwiseProduct(f) / p;

		if (Q.object->material->isLight())
		{
			float q = PdfLight(Q) / GeometryFactor(P, Q);
			w_mis = p * p / (p * p + q * q);
			C += W.cwiseProduct(w_mis * EvalRadiance(Q));
			break;
		}

		P = Q;
		wo = -wi;
	}

	return C;
}

Intersection Realtime::SampleLight()
{
	Obj* light = *select_randomly(lights.begin(),lights.end());
	Sphere* sphere = static_cast<Sphere*>(light->shape);
	return SampleSphere(sphere->center, sphere->radius, light);
}

Intersection Realtime::SampleSphere(Vector3f center, float radius, Obj* light)
{
	float psi1 = myrandom(RNGen);
	float psi2 = myrandom(RNGen);

	float z = 2 * psi1 - 1;
	float r = sqrt(1 - z * z);
	float a = 2 * PI * psi2;

	Vector3f normal = Vector3f(r * cos(a), r * sin(a), z);
	Vector3f point = center + radius * normal;
	return Intersection(light, point, normal);
}

float Realtime::PdfLight(Intersection& L)
{
	Sphere* s = static_cast<Sphere*>(L.object->shape);
	return 1.0f / (4 * PI * s->radius * s->radius * lights.size());
}

float Realtime::GeometryFactor(Intersection& P, Intersection& L)
{
	Vector3f D = P.P - L.P;
	float D_dot_D = D.dot(D);
	return fabs(P.N.dot(D) * L.N.dot(D) / (D_dot_D * D_dot_D));
}

Vector3f Realtime::SampleBrdf(Vector3f wo, Vector3f N, Material* mat, float pd)
{
	float psi1 = myrandom(RNGen);
	float psi2 = myrandom(RNGen);
	
	float cpsi = myrandom(RNGen);

	// Choose Diffuse
	if (cpsi < pd)
	{
		return SampleLobe(N, sqrt(psi1), 2 * PI * psi2);
	}
	// Choose Reflection

	// GGX
	//float alpha = sqrt(2.0f / (mat->alpha + 2.0f));
	//float cosTm = cos(atan(alpha * sqrt(psi1) / sqrt(1 - psi1)));

	// Phong
	float cosTm = pow(psi1, 1 / (mat->alpha + 1));

	Vector3f m = SampleLobe(N, cosTm, 2 * PI * psi2);

	return 2.0f * wo.dot(m) * m - wo;
}

Vector3f Realtime::SampleLobe(Vector3f N, float c, float fi)
{
	float s = sqrt(1 - c * c);
	Vector3f K = Vector3f(s * cos(fi), s * sin(fi), c);
	Quaternionf q = Quaternionf::FromTwoVectors(Vector3f::UnitZ(), N);
	return q._transformVector(K);
}

Vector3f Realtime::EvalScattering(Vector3f wo, Vector3f N, Vector3f wi, Material* mat)
{
	Vector3f Ed = mat->Kd / PI;

	Vector3f m = (wo + wi).normalized();
	float G = GTerm(wi, m, N, mat->alpha) * GTerm(wo, m, N, mat->alpha);

	Vector3f Er = DTerm(m, N, mat->alpha) * G * FTerm(wi.dot(m), mat) / (4 * fabs(N.dot(wi)) * fabs(N.dot(wo)));
	
	return fabs(N.dot(wi)) * (Ed + Er);
}

float Realtime::PdfBrdf(Vector3f wo, Vector3f N, Vector3f wi, float pd, float pr, float alpha)
{
	float Pdiff = fabs(N.dot(wi)) / PI;

	Vector3f m = (wo + wi).normalized();
	float Pref = DTerm(m, N, alpha) * fabs(N.dot(m)) / (4 * fabs(m.dot(wi)));

	return pd * Pdiff + pr*Pref;
}

float Realtime::DTerm(Vector3f m, Vector3f N, float alpha)
{
	float mDotN = m.dot(N);
	int X = mDotN > 0 ? 1 : 0;
	
	float tanTm = sqrt(fabs(1.0f - mDotN * mDotN)) / mDotN;

	//GGX
	//alpha = sqrt(2.0f / (alpha + 2.0f));
	//return X * ((alpha * alpha) / (PI * pow(mDotN, 4) * pow(alpha * alpha + tanTm * tanTm, 2)));

	// Phong
	return ((alpha + 2) / (2 * PI)) * pow(mDotN, alpha);
}

float Realtime::GTerm(Vector3f v, Vector3f m, Vector3f N, float alpha)
{
	float d = v.dot(m) / v.dot(N);
	int X = d > 0 ? 1 : 0;

	float vDotN = v.dot(N);
	
	if(vDotN > 1.0f)
	{
		return 1.0f;
	}
	
	float tanTv = sqrt(1.0f - vDotN * vDotN) / vDotN;

	if (tanTv == 0.0f)
	{
		return 1.0f;
	}

	// GGX
	//alpha = sqrt(2.0f / (alpha + 2.0f));
	//return X * 2.0f / (1.0f + sqrt(1.0f + alpha * alpha * tanTv * tanTv));

	// Phong
	float a = sqrt(alpha / 2 + 1) / tanTv;
	
	if (a < 1.6f)
	{
		return X * (3.535f * a + 2.181f * a * a) / (1.0f + 2.276f * a + 2.577f * a * a);
	}
	else
	{
		return X * 1.0f;
	}

}

Vector3f Realtime::FTerm(float LdotH, Material* mat)
{
	return mat->Ks + (Vector3f(1.0f, 1.0f, 1.0f) - mat->Ks) * pow(1.0f - fabs(LdotH), 5);
}

Vector3f Realtime::EvalRadiance(Intersection& Q)
{
	return Q.object->material->Kd;
}

// Called when the scene needs to be redrawn.
void Realtime::DrawScene()
{
    Vector3f viewDir = ViewDirection();
    Vector2f dir2 = Vector2f(viewDir.x(), viewDir.y()).normalized();
    if (motionkey == 'w')
        eye += speed*Vector3f(dir2.x(), dir2.y(), 0.0);
    if (motionkey == 's')
        eye -= speed*Vector3f(dir2.x(), dir2.y(), 0.0);
    if (motionkey == 'd')
        eye += speed*Vector3f(dir2.y(), -dir2.x(), 0.0);
    if (motionkey == 'a')
        eye -= speed*Vector3f(dir2.y(), -dir2.x(), 0.0);
    if (motionkey == 'e')
        eye -= speed*Vector3f(0.0f, 0.0f, -1.0f);
    if (motionkey == 'c')
        eye -= speed*Vector3f(0.0f, 0.0f, 1.0f);
    
    int loc;

    glClearColor(0.3,0.3, 0.3, 1.0);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glClear(GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT);

    Matrix4f WorldView;
    Matrix4f R = toMat4(ViewQuaternion().conjugate());
    WorldView = R*translate(-eye);

    float rx = (ry*width)/height;
    Matrix4f WorldProj = frustum(-front*rx, front*rx, -front*ry, front*ry, front, back);

    lighting.Use();

    loc = glGetUniformLocation(lighting.program, "WorldProj");
    glUniformMatrix4fv(loc, 1, GL_FALSE, WorldProj.data());

    loc = glGetUniformLocation(lighting.program, "WorldView");
    glUniformMatrix4fv(loc, 1, GL_FALSE, WorldView.data());

    loc = glGetUniformLocation(lighting.program, "ambient");
    glUniform3fv(loc, 1, &ambient[0]);

    loc = glGetUniformLocation(lighting.program, "eyePos");
    glUniform3fv(loc, 1, &eye[0]);

    Vector3f lightEmit[8];
    Vector3f lightPosn[8];
    int  lightNum = lights.size();
    for (int i=0;  i<lightNum;  i++) {
        lightPosn[i] = lights[i]->Center();
        lightEmit[i] = lights[i]->material->Kd; }

    loc = glGetUniformLocation(lighting.program, "lightNum");
    glUniform1i(loc, lightNum);

    loc = glGetUniformLocation(lighting.program, "lightPosn");
    glUniform3fv(loc, 8, &lightPosn[0][0]);

    loc = glGetUniformLocation(lighting.program, "lightEmit");
    glUniform3fv(loc, 8, &lightEmit[0][0]);
        
    // for (unsigned int i=0;  i<objs.size();  i++) {
    //     Material* material = objs[i]->material;
    //     Matrix4f& modelTR = objs[i]->modelTR;
    //     ...
    //     objs[i]->draw();
    for (auto obj : objs) {     // C++11 loop
        Material* material = obj->material;
        Matrix4f& modelTR = obj->modelTR;
        Matrix3f normalTR = modelTR.inverse().block<3,3>(0,0);

        loc = glGetUniformLocation(lighting.program, "ModelTr");
        glUniformMatrix4fv(loc, 1, GL_FALSE, modelTR.data());

        loc = glGetUniformLocation(lighting.program, "normalTR");
        glUniformMatrix3fv(loc, 1, GL_FALSE, normalTR.data());

        if (!material) {
            std::cerr << "No material associated with object\n";
            exit(-1); }
        if (material->isLight())
            applyLight(material, lighting.program);
        else
            applyMaterial(material, lighting.program);
        obj->draw(); }

    lighting.Unuse();
    glutSwapBuffers();
}

void WriteHdrImage(const std::string outName, const int width, const int height, Color* image)
{
	// Turn image from a 2D-bottom-up array of Vector3D to an top-down-array of floats
	float* data = new float[width * height * 3];
	float* dp = data;
	for (int y = height - 1; y >= 0; --y) {
		for (int x = 0; x < width; ++x) {
			Color pixel = image[y * width + x];
			*dp++ = pixel[0];
			*dp++ = pixel[1];
			*dp++ = pixel[2];
		}
	}

	// Write image to file in HDR (a.k.a RADIANCE) format
	rgbe_header_info info;
	char errbuf[100] = { 0 };

	FILE* fp = fopen(outName.c_str(), "wb");
	info.valid = false;
	int r = RGBE_WriteHeader(fp, width, height, &info, errbuf);
	if (r != RGBE_RETURN_SUCCESS)
		printf("error: %s\n", errbuf);

	r = RGBE_WritePixels_RLE(fp, data, width, height, errbuf);
	if (r != RGBE_RETURN_SUCCESS)
		printf("error: %s\n", errbuf);
	fclose(fp);

	delete data;
}

void Realtime::DrawOutput()
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	Vector3f E = eye;
	float rx = ry * width / height;
	Quaternionf o = ViewQuaternion();
	Vector3f X = rx * o._transformVector(Vector3f::UnitX());
	Vector3f Y = ry * o._transformVector(Vector3f::UnitY());
	Vector3f Z = -1 * o._transformVector(Vector3f::UnitZ());

	//std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::high_resolution_clock::now();
	
	for (int pass = 1; pass <= max_passes; ++pass)
	{
#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
		for (int y = 0; y < height; y++) {

			//fprintf(stderr, "Rendering %4d\r", y);
			for (int x = 0; x < width; x++) {

				float dx = 2 * (x + myrandom(RNGen)) / width - 1;
				float dy = 2 * (y + myrandom(RNGen)) / height - 1;

				/*Color color;
				if ((x - width / 2) * (x - width / 2) + (y - height / 2) * (y - height / 2) < 100 * 100)
					color = Color(myrandom(RNGen), myrandom(RNGen), myrandom(RNGen));
				else if (abs(x - width / 2) < 4 || abs(y - height / 2) < 4)
					color = Color(0.0, 1.0, 0.0);
				else
					color = Color(1.0, 1.0, 1.0);*/
				Vector3f dir = dx * X + dy * Y + Z;
				Ray ray(E, dir.normalized());
				
				Vector3f test = imagePointer[y * width + x];
				test += TraceRay(ray);
				if (!(isnan(test.x()) || isinf(test.x()) ||
					isnan(test.y()) || isinf(test.y()) || 
					isnan(test.z()) || isinf(test.z())))
				{
					imagePointer[y * width + x] = test;
				}
			}

		}

		
		//std::chrono::time_point<std::chrono::steady_clock> end = std::chrono::high_resolution_clock::now();
		//std::chrono::duration<float> duration = end - start;
		//
		//float ms = duration.count() * 1000.0f;
		//printf("Execution Time: %f ms", ms);
		if (pass == 1 || pass == 8 || pass == 64 || pass == 512 || pass == 2048 || pass == 4096)
		{
			glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			raytraceout.Use();

			glGenTextures(1, &imageTexture);
			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, imageTexture);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, (GLint)GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, (GLint)GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, (GLint)GL_LINEAR_MIPMAP_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, (GLint)GL_LINEAR);

			glTexImage2D(GL_TEXTURE_2D, 0, (GLint)GL_RGB32F, width, height, 0, GL_RGB, GL_FLOAT, &imagePointer[0]);
			glGenerateMipmap(GL_TEXTURE_2D);

			int loc = glGetUniformLocation(raytraceout.program, "imageTex");
			glUniform1i(loc, 1);
			loc = glGetUniformLocation(raytraceout.program, "pass");
			glUniform1i(loc, pass);

			DrawFSQ();
			raytraceout.Unuse();
			glutSwapBuffers();


			WriteHdrImage("Raycast_" + std::to_string(pass) + "_reflection_phong.hdr", width, height, imagePointer);
			printf("Written to HDR file\n");
		}
		
		//printf("Pass %d\n",pass);
		//fprintf(stderr, "\n");
	}
	
	glutLeaveMainLoop();
	fprintf(stderr, "\n");
}

void Realtime::DrawFSQ()
{
	static unsigned int quadVAO = 0;
	static unsigned int quadVBO;
	if (quadVAO == 0)
	{
		float quadVert[] = {
			-1.0f, 1.0f, 0.0f, 0.0f, 1.0f,
			-1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
			1.0f, 1.0f, 0.0f, 1.0f, 1.0f,
			1.0f, -1.0f, 0.0f, 1.0f, 0.0f
		};

		// Setup plane vao
		glGenVertexArrays(1, &quadVAO);
		glGenBuffers(1, &quadVBO);
		glBindVertexArray(quadVAO);
		glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(quadVert), &quadVert, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
	}
	glBindVertexArray(quadVAO);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBindVertexArray(0);
}

// Called by GLUT when the window size is changed.
void Realtime::ReshapeWindow(int w, int h)
{
    if (w && h)
        glViewport(0, 0, w, h);
    width = w;
    height = h;

    // Force a redraw
    glutPostRedisplay();
}

// Called by GLUT for keyboard actions.
void Realtime::KeyboardDown(unsigned char key, int x, int y)
{
    printf("key: %c\n", key);
    switch(key) {
    case 9:
        nav = !nav;
        break;

    case 'v': {
        Quaternionf q = ViewQuaternion();
        printf("camera  %g %g %g   %g   q %g %g %g %g\n",
               eye[0], eye[1], eye[2], ry,  q.w(), q.x(), q.y(), q.z());
        printf("screen %d %d\n", width, height);
        fflush(stdout); }
        break;

    case 'w': case 's': case 'a': case 'd': case 'e': case 'c':
        motionkey = key;
        break;
	case 'r':
		glutDisplayFunc(&CBDrawOutput);
		break;
    case 27:                    // Escape key
    case 'q':
        glutLeaveMainLoop();
        break;
    }
}

void Realtime::KeyboardUp(unsigned char key, int x, int y)
{
    motionkey = 0;
    fflush(stdout);
}

// Called by GLut when a mouse button changes state.
void Realtime::MouseButton(int button, int state, int x, int y)
{        
    // Record the position of the mouse click.
    mouseX = x;
    mouseY = y;

    // Test if the SHIFT keey was down for this mouse click
    shifted = glutGetModifiers() && GLUT_ACTIVE_SHIFT;

    // Ignore high order bits, set by some (stupid) GLUT implementation.
    button = button%8;

    if (button == GLUT_LEFT_BUTTON) {
        leftDown = (state == GLUT_DOWN); }

    else if (button == GLUT_MIDDLE_BUTTON) {
        middleDown = (state == GLUT_DOWN); }

    else if (button == GLUT_RIGHT_BUTTON) {
        rightDown = (state == GLUT_DOWN); }

    else if (button == 3) {
        Vector3f C = eye + cDist*ViewDirection();
        cDist = pow(cDist, 1.0f/1.02f); 
        eye = C - cDist*ViewDirection(); }

    else if (button == 4) {
        Vector3f C = eye + cDist*ViewDirection();
        cDist = pow(cDist, 1.02f);
        eye = C - cDist*ViewDirection(); }
    
    // Force a redraw
    glutPostRedisplay();
    fflush(stdout);
}

void Realtime::MouseMotion(int x, int y)
{
    // Calculate the change in the mouse position
    int dx = x-mouseX;
    int dy = y-mouseY;

    if (leftDown) {        // Rotate light position
        if (nav) {
            spin += dx/2.0f;
            tilt += dy/2.0f; }
        else {
            Vector3f C = eye + cDist*ViewDirection();
            spin += dx/2.0f;
            tilt += dy/2.0f;
            eye = C - cDist*ViewDirection(); } }

    else if (middleDown) { }

    // Record this position
    mouseX = x;
    mouseY = y;

    // Force a redraw
    glutPostRedisplay();
}


void Realtime::sphere(const Vector3f center, const float r, Material* mat)
{
    Matrix4f m = translate(center) * scale(Vector3f(r,r,r));
    Vector3f rrr(r,r,r);
    Obj* obj = new Obj(sphMesh, m, mat);
    objs.push_back(obj);
	Sphere* sphere = new Sphere(center, r);
	sphere->object = obj;
	obj->shape = sphere;
	shapes.push_back(sphere);
    if (mat->isLight())
        lights.push_back(obj);
}

void Realtime::box(const Vector3f base, const Vector3f diag, Material* mat)
{
    Matrix4f m = translate(base) * scale(Vector3f(diag[0],diag[1],diag[2]));
    Obj* obj = new Obj(boxMesh, m, mat);
    objs.push_back(obj);
	Box* box = new Box(base, diag);
	box->object = obj;
	obj->shape = box;
	shapes.push_back(box);
    if (mat->isLight())
        lights.push_back(obj);
}


void Realtime::cylinder(const Vector3f base, const Vector3f axis, const float radius, Material* mat)
{
    Vector3f Z(0.0f, 0.0f, 1.0f);
    Vector3f C = axis.normalized();
    Vector3f B = C.cross(Z);
    if (B.norm() <1e-8)
        B = Vector3f(0,1,0);
    else
        B = B.normalized();
    Vector3f A = B.cross(C).normalized();
    Matrix4f R;
    R<< A(0),B(0),C(0),0.0f,    // Row-wise text (although init is col-wise)
        A(1),B(1),C(1),0.0f,
        A(2),B(2),C(2),0.0f,
        0.0f, 0.0f, 0.0f, 1.0;

    Matrix4f m = translate(base)*R*scale(Vector3f(radius,radius,axis.norm()));
    Vector3f rrr(radius,radius,radius);
    Obj* obj = new Obj(cylMesh, m, mat);
    objs.push_back(obj);
	Cylinder* cylinder = new Cylinder(base,axis,radius);
	cylinder->object = obj;
	obj->shape = cylinder;
	shapes.push_back(cylinder);
    if (mat->isLight())
        lights.push_back(obj);
}

void Realtime::triangleMesh(MeshData* meshdata)
{
	Obj* obj = new Obj(meshdata, Matrix4f::Identity(), meshdata->mat);
	objs.push_back(obj);
	for(TriData tridata : meshdata->triangles)
	{
		Triangle* triangle = new Triangle(meshdata, tridata);
		triangle->object = obj;
		shapes.push_back(triangle);
	}
	if (meshdata->mat->isLight())
		lights.push_back(obj);
}

namespace Eigen
{
	AlignedBox<float, 3> bounding_box(const Shape* obj)
	{
		return obj->Bounding_Box();
	}
}