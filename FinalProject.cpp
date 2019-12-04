/*
Author: <Yaru Niu> 
Class: ECE6122 
Last Date Modified: <2019-12-04>
Description: Final Project: the main program and functions to run the simulation of GaTech Buzzy Bowl UAV show.
The program is including one forcess to render the 3D scene and 15 processes to control the UAVs.
*/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "mpi.h"
#include "iomanip"
#include <cmath>
#include <math.h>
#include <cstdlib>
#include <GL/glut.h>
#include <chrono>
#include <thread>
#include "bitmap.h"

using namespace std;

// Send location and velocity vector in each direction
const int numElements = 6; 
// (Main task + 15 UAVs) * numElements
const int rcvSize = 16 * 6; 
// Receive buffer and send buffer
double* rcvbuffer = new double[rcvSize];
double sendBuffer[numElements] = {0, 0, 0, 0, 0, 0};
// Define the football field size
const double playFiledLength = 91.44;
const double fullFieldLength = 109.7;
const double fullFieldWidth = 48.8;
// Define the UAV mass
const double mass = 1;
// Other parameters or global variables
int flag1 = 1;
int flag2 = 1;
long int colorFlag = 0;
double colorValue = 1.0;
const double alpha1 = 1.5;
const double alpha2 = 0.7;
const double beta = 0.2;
bool collision = false;

// Struct BMP
BMP inBitmap;
GLuint texture[1];
#define checkImageWidth 64
#define checkImageHeight 64
GLubyte checkImage[checkImageWidth][checkImageHeight][3];

// Check image function
void makeCheckImage(void)
{
    int i, j, c;

    for (i = 0; i < checkImageWidth; i++) 
    {
        for (j = 0; j < checkImageHeight; j++) 
        {
            c = ((((i & 0x8) == 0) ^ ((j & 0x8) == 0))) * 255;
            checkImage[i][j][0] = (GLubyte)c;
            checkImage[i][j][1] = (GLubyte)c;
            checkImage[i][j][2] = (GLubyte)c;
        }
    }
}


// Reshape callback
void changeSize(int w, int h)
{
    float ratio = ((float)w) / ((float)h); // window aspect ratio
    glMatrixMode(GL_PROJECTION); // projection matrix is active
    glLoadIdentity(); // reset the projection
    gluPerspective(60.0, ratio, 0.1, 1000.0); // perspective transformation
    glMatrixMode(GL_MODELVIEW); // return to modelview mode
    glViewport(0, 0, w, h); // set viewport (drawing area) to entire window
}

// Initialization for rendering bmp file
void  bitmapInit()
{
	// Read bmp file 
    inBitmap.read("AmFBfield.bmp");
    // Check image
    makeCheckImage();

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    // Create Textures
    glGenTextures(1, texture);

    // Setup first texture
    glBindTexture(GL_TEXTURE_2D, texture[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); //scale linearly when image bigger than texture
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); //scale linearly when image smalled than texture


    glTexImage2D(GL_TEXTURE_2D, 0, 3, inBitmap.bmp_info_header.width, inBitmap.bmp_info_header.height, 0,
                 GL_BGR_EXT, GL_UNSIGNED_BYTE, &inBitmap.data[0]);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);	
}

// Function to draw the football field (including the basic one and rendering the bmp picture)
void displayFootballField()
{
	//-----------------------------------------
	// Draw the green football field
	//----------------------------------------- 
    // glColor3f(0, 1, 0);
    // glBegin(GL_POLYGON);
    // glVertex3f(-fullFieldLength / 2, fullFieldWidth / 2, -0.00001);
    // glVertex3f(fullFieldLength / 2, fullFieldWidth / 2, -0.00001);
    // glVertex3f(fullFieldLength / 2, -fullFieldWidth / 2, -0.00001);
    // glVertex3f(-fullFieldLength / 2, -fullFieldWidth / 2, -0.00001);
    // glEnd();   

	// Place texture
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture[0]);

    glBegin(GL_QUADS);
    glTexCoord2f(0, 1);
    glVertex3f(-fullFieldLength / 2 - 3.5, fullFieldWidth / 2 + 3.5, 0.0);
    glTexCoord2f(1, 1);
    glVertex3f(fullFieldLength / 2 + 3.5, fullFieldWidth / 2 + 3.5, 0.0);
    glTexCoord2f(1, 0);
    glVertex3f(fullFieldLength / 2 + 3.5, -fullFieldWidth / 2 - 3.5, 0.0);
    glTexCoord2f(0, 0);
    glVertex3f(-fullFieldLength / 2 - 3.5, -fullFieldWidth / 2 - 3.5, 0.0);
    glEnd();
    
    glDisable(GL_TEXTURE_2D); 
}

// Function to draw UAVs
void drawUAVs()
{
	colorFlag += 1;

    glColor3f(0, colorValue, colorValue);
    for (int i = 6; i < 96; i = i + 6)
    {
        glPushMatrix();
        glTranslatef(rcvbuffer[i], rcvbuffer[i+1], rcvbuffer[i+2]);
        glScalef(0.50, 0.50, 0.50);
        glutSolidOctahedron();
        // glutSolidSphere(0.5, 20, 20);
        glPopMatrix();
    }
    // Oscillate between full and half color
	if ((colorFlag % 20 == 0) && ((((colorFlag / 20) / 128) % 2) == 0))
	{
		colorValue = colorValue - 0.00392157;
	}
	if ((colorFlag % 20 == 0) && ((((colorFlag / 20) / 128) % 2) == 1))
	{
		colorValue = colorValue + 0.00392157;
	}
}

// Function to draw the sphere
void drawSphere()
{
    glColor3f(0.75, 0.75, 0.75);
    glPushMatrix();
    glTranslatef(0, 0, 50);
    glutWireSphere(10, 24, 24);
    glPopMatrix();
}

// Function to initialize the UAV positions
void initializeUAVLocations()
{
    for (int i = 0; i < 6; ++i)
    {
        rcvbuffer[i] = 0;
    }
    for (int j = 6; j < 96; j = j + 6)
    {
        rcvbuffer[j] = - playFiledLength / 2 +  playFiledLength / 4 * (((j - 6)/6)%5);
        rcvbuffer[j+1] = - fullFieldWidth / 2 + fullFieldWidth / 2 * (((j - 6)/6)/5);
        rcvbuffer[j+2] = 0.5;
        rcvbuffer[j+3] = 0;
        rcvbuffer[j+4] = 0;
        rcvbuffer[j+5] = 0;
    }
}
// Function to normalize a vector with 3 elements 
double* normalize(double delta[3])
{
    double distance = sqrt(delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2]);
    delta[0] = delta[0] / distance;
    delta[1] = delta[1] / distance;
    delta[2] = delta[2] / distance;
    return delta;
}

// Function to draw the entire scene
void renderScene()
{

    // Clear color and depth buffers
    glClearColor(0.0, 0.0, 0.36, 1.0); 
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Reset transformations
    glLoadIdentity();

    // Set camera location
    gluLookAt(0, -fullFieldWidth - 11, 101, 
              0, 0, 5,
              0.0, 0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);

    // Set the light
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };

    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);

    // Draw the sphere
	drawSphere();
	// Draw UAVs
    drawUAVs();
    // Draw the football field
    displayFootballField();
	// Make it all visible    
    glutSwapBuffers(); 
    // Receive all and store them in a buffer, send the sendBuffer
    MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
}

// timerFunction: called whenever the timer fires
// Update every 100 msec 
void timerFunction(int id)
{
    glutPostRedisplay();
    glutTimerFunc(100, timerFunction, 0);
}


// mainOpenGL  - standard GLUT initializations and callbacks
void mainOpenGL(int argc, char**argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100, 100);
    // Window size: 400 * 400
    glutInitWindowSize(400, 400);
    glutCreateWindow("GaTech Buzzy Bowl");

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_NORMALIZE);

    // Initialize bmp rendering setting
    bitmapInit();

    glutReshapeFunc(changeSize);
    glutDisplayFunc(renderScene);
    glutTimerFunc(100, timerFunction, 0);
    glutMainLoop();
}

// Function to calculate the UAV locations 
// Including the control policy and updates of the positions and velocities
void CalculateUAVsLocation(int rank)
{
    double xPos = sendBuffer[0];
    double yPos = sendBuffer[1];
    double zPos = sendBuffer[2];
    double xVel = sendBuffer[3];
    double yVel = sendBuffer[4];
    double zVel = sendBuffer[5];

    // Check the collision 
    for (int i = 1; i <= 15; ++i)
    {
    	if (i != rank)
    	{
    		if (sqrt((xPos-rcvbuffer[6*i])*(xPos-rcvbuffer[6*i]) + (yPos-rcvbuffer[6*i+1])*(yPos-rcvbuffer[6*i+1]) + (zPos-rcvbuffer[6*i+2])*(zPos-rcvbuffer[6*i+2])) <= 1.01)
    		{
    			// If there is collision, swap the velocity
    			xVel = rcvbuffer[6*i+3];
    			yVel = rcvbuffer[6*i+4];
    			zVel = rcvbuffer[6*i+5];
    			collision = true;
    			
    			break;
    		}
    	}
    }

    double delta[3] = {-xPos, -yPos, 50 - zPos};
    double* deltaNorm = normalize(delta);
    double xForce = 0;
    double yForce = 0;
    double zForce = 10;  
    // A tangent force to give initialized tangent accelerations to the UAV when surrounding the sphere
    double tanF[3] = {-deltaNorm[2]*deltaNorm[0], -deltaNorm[2]*deltaNorm[1], 1-deltaNorm[2]*deltaNorm[2]};
    // Normalize tanF
    double* tanForce = normalize(tanF);  
    double vel[3] = {xVel, yVel, zVel};
    double* velNorm = normalize(vel);
    double inProduct = deltaNorm[0] * velNorm[0] + deltaNorm[1] * velNorm[1] + deltaNorm[2] * velNorm[2];
    // A tangent force to give later tangent accelerations to the UAV when surrounding the sphere
    double tanF1[3] = {velNorm[0]-inProduct*deltaNorm[0], velNorm[1]-inProduct*deltaNorm[1], velNorm[2]-inProduct*deltaNorm[2]};
    // Normalize tanF1
    double* tanForce1 = normalize(tanF1);  

    // Policy if distance > 18
    // Before reaching the sphere, accelerate then go with constant speed
    if (sqrt(xPos*xPos + yPos*yPos + (50 - zPos)*(50 - zPos)) > 18)
    {
        if (flag1 == 1)
        {
        	// Accelerate
            if (sqrt(xVel*xVel + yVel*yVel + zVel*zVel) < 1.97)
            {
                xForce = 0.3 * deltaNorm[0];
                yForce = 0.3 * deltaNorm[1];
                zForce = 0.3 * deltaNorm[2] + 10;
            }
            // Go with constant speed
            else
            {
                xForce = 0;
                yForce = 0;
                zForce = 10;
            }            
        }
        else
        {
            xForce = 0.1 * deltaNorm[0];
            yForce = 0.1 * deltaNorm[1];
            zForce = 0.1 * deltaNorm[2] + 10;            
        }

    }

    // Policy if distance <= 18
    if (sqrt(xPos*xPos + yPos*yPos + (50 - zPos)*(50 - zPos)) <= 18)
    {
        flag1 = 0;
        // If distance > 10
        if (sqrt(xPos*xPos + yPos*yPos + (50 - zPos)*(50 - zPos)) > 10)
        {
        	// Decelerate when closer to the sphere
            if (flag2 == 1 && collision == false)
            {
                xForce = -0.24*deltaNorm[0];
                yForce = -0.24*deltaNorm[1];
                zForce = -0.24*deltaNorm[2] + 10; 

                // Give a small initialized tangent velocity
                if (rank == 8)
                {
                	if (zPos >= 38)
                	{
                		xForce = -0.24*deltaNorm[0] - 0.1;
                	}
                }                
            }
            else
            {
            	// Give initialized tangent accerations and velocities if the magnitude of the velocity is below 5
            	// Follow the Hooke’s Law in normal direction
            	if (sqrt(xVel*xVel + yVel*yVel + zVel*zVel) < 0.5)
            	{
            		// Different UAV has different initialized policy
	            	if ((rank >= 1 && rank <=5) || (rank >= 11 && rank <=15))
	            	{
	            		if (deltaNorm[1] < 0)
	            		{
			                xForce = alpha1 * deltaNorm[0] + beta * tanForce[0];
			                yForce = alpha1 * deltaNorm[1] + beta * tanForce[1];
			                zForce = alpha1 * deltaNorm[2] + 10 + beta * tanForce[2];              					
	            		}
	            		if (deltaNorm[1] > 0)
	            		{
			                xForce = alpha1 * deltaNorm[0] - beta * tanForce[0];
			                yForce = alpha1 * deltaNorm[1] - beta * tanForce[1];
			                zForce = alpha1 * deltaNorm[2] + 10 - beta * tanForce[2];              				
	            		}            		
	            	}
	            	if (rank >= 6 && rank <=10)
	            	{
	            		if (deltaNorm[0] < 0)
	            		{
			                xForce = alpha1 * deltaNorm[0] + beta * tanForce[0];
			                yForce = alpha1 * deltaNorm[1] + beta * tanForce[1];
			                zForce = alpha1 * deltaNorm[2] + 10 + beta * tanForce[2];                				
	            		}

	            		if (deltaNorm[0] > 0)
	            		{
			                xForce = alpha1 * deltaNorm[0] - beta * tanForce[0];
			                yForce = alpha1 * deltaNorm[1] - beta * tanForce[1];
			                zForce = alpha1 * deltaNorm[2] + 10 - beta * tanForce[2];             				
	            		}            		
	            	}             		
            	}

            	// If the magnitude of the velocity is above 5, follow the Hooke’s Law, and the 
            	// tangent force will always be along the porjected vector on the tangent plane
            	// Follow the Hooke’s Law in normal and tangent directions
            	if (sqrt(xVel*xVel + yVel*yVel + zVel*zVel) >= 0.5)
            	{
        			if (sqrt(xVel*xVel + yVel*yVel + zVel*zVel) < (3 + 0.02 * rank))
        			{
		                xForce = alpha1 * deltaNorm[0] + beta * tanForce1[0];
		                yForce = alpha1 * deltaNorm[1] + beta * tanForce1[1];
		                zForce = alpha1 * deltaNorm[2] + 10 + beta * tanForce1[2];              				
        			}
        			else
        			{
		                xForce = alpha1 * deltaNorm[0] - beta * tanForce1[0];
		                yForce = alpha1 * deltaNorm[1] - beta * tanForce1[1];
		                zForce = alpha1 * deltaNorm[2] + 10 - beta * tanForce1[2]; 
        			}            		
            	}
              
            }
        }

        // If distance <= 10
        if (sqrt(xPos*xPos + yPos*yPos + (50 - zPos)*(50 - zPos)) <= 10)
        {
            flag2 = 0;
            // Give initialized tangent accerations and velocities if the magnitude of the velocity is below 5
            // Follow the Hooke’s Law in normal direction
            if (sqrt(xVel*xVel + yVel*yVel + zVel*zVel) < 0.5)
            {
            	// Different UAV has different initialized policy
	        	if ((rank >= 1 && rank <=5) || (rank >= 11 && rank <=15))
	        	{
	        		if (deltaNorm[1] < 0)
	        		{
		                xForce = -alpha2 * deltaNorm[0] + beta * tanForce[0];
		                yForce = -alpha2 * deltaNorm[1] + beta * tanForce[1];
		                zForce = -alpha2 * deltaNorm[2] + 10 + beta * tanForce[2];
	        		}
	        		if (deltaNorm[1] > 0)
	        		{
		                xForce = -alpha2 * deltaNorm[0] - beta * tanForce[0];
		                yForce = -alpha2 * deltaNorm[1] - beta * tanForce[1];
		                zForce = -alpha2 * deltaNorm[2] + 10 - beta * tanForce[2];              				
	        		}            		
	        	}
	        	if (rank >= 6 && rank <=10)
	        	{
	        		if (deltaNorm[0] < 0)
	        		{
		                xForce = -alpha2 * deltaNorm[0] + beta * tanForce[0];
		                yForce = -alpha2 * deltaNorm[1] + beta * tanForce[1];
		                zForce = -alpha2 * deltaNorm[2] + 10 + beta * tanForce[2];                				
	        		}

	        		if (deltaNorm[0] > 0)
	        		{
		                xForce = -alpha2 * deltaNorm[0] - beta * tanForce[0];
		                yForce = -alpha2 * deltaNorm[1] - beta * tanForce[1];
		                zForce = -alpha2 * deltaNorm[2] + 10 - beta * tanForce[2];             				
	        		}            		
	        	}             	
            }
        	// If the magnitude of the velocity is above 5, follow the Hooke’s Law, and the 
        	// tangent force will always be along the porjected vector on the tangent plane
        	// Follow the Hooke’s Law in normal and tangent directions
            if (sqrt(xVel*xVel + yVel*yVel + zVel*zVel) >= 0.5)
            {
				if (sqrt(xVel*xVel + yVel*yVel + zVel*zVel) < (3 + 0.02 * rank))
				{
	                xForce = -alpha2 * deltaNorm[0] + beta * tanForce1[0];
	                yForce = -alpha2 * deltaNorm[1] + beta * tanForce1[1];
	                zForce = -alpha2 * deltaNorm[2] + 10 + beta * tanForce1[2];              				
				}
				else
				{
	                xForce = -alpha2 * deltaNorm[0] - beta * tanForce1[0];
	                yForce = -alpha2 * deltaNorm[1] - beta * tanForce1[1];
	                zForce = -alpha2 * deltaNorm[2] + 10 - beta * tanForce1[2]; 
				}            	
            }
       
        }
    }

    // Calculate accelerations 
    double xAccel = xForce / mass;
    double yAccel = yForce / mass;
    double zAccel = (zForce - 10) / mass;
    // Calculate positions and velocities
    xPos += xVel * 0.1 + xAccel * 0.01 / 2;
    xVel += xAccel * 0.1;
    yPos += yVel * 0.1 + yAccel * 0.01 / 2;
    yVel += yAccel * 0.1;    
    zPos += zVel * 0.1 + zAccel * 0.01 / 2;
    zVel += zAccel * 0.1;    
    // Put information in the sendBuffer
    sendBuffer[0] = xPos;
    sendBuffer[1] = yPos;
    sendBuffer[2] = zPos;
    sendBuffer[3] = xVel;
    sendBuffer[4] = yVel;
    sendBuffer[5] = zVel;
}

// Main: determines rank of the process and follows the corresponding execution
int main(int argc, char**argv)

{
    int numTasks, rank;
    // Initialize the MPI execution environment
    int rc = MPI_Init(&argc, &argv);
    // Check if initialize successfully, otherwise abort
    if (rc != MPI_SUCCESS) 
    {
        printf("Error starting MPI program. Terminating.\n");
        MPI_Abort(MPI_COMM_WORLD, rc);
    }
    // Return the total number of MPI processes in the specified communicator (MPI_COMM_WORLD)
    MPI_Comm_size(MPI_COMM_WORLD, &numTasks);
    // Return the rank of the calling MPI process within the specified communicator (MPI_COMM_WORLD)
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

    int gsize = 0;

    MPI_Comm_size(MPI_COMM_WORLD, &gsize);
    // Initialize UVA locations
    initializeUAVLocations();

    // Process of rendering 3D scene
    if (rank == 0) 
    {
        mainOpenGL(argc, argv);
    }
    // Process of controlling the UAVs
    else
    {
    	// Stay on the ground for 5 seconds
        for (int i = 0; i < 6; ++i)
        {
            sendBuffer[i] = rcvbuffer[6 * rank + i];
        }
        // Sleep for 5 seconds
        std::this_thread::sleep_for(std::chrono::seconds(5));
        // Will surrounding the sphere for about 60 seconds
        for (int ii = 0; ii < 1000 ; ii++)
        {
        	// Update UAV locations and velocities
            CalculateUAVsLocation(rank); 
            // Send buffers (does not use the receive buffer)
            MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
        }
    }
    return 0;
}