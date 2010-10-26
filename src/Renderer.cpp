/* 
 * File:   Renderer.cpp
 * Author: me
 * 
 * Created on October 24, 2010, 1:14 AM
 */

#include "Renderer.h"

#include "GLDebugFont.h"



#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"
#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/btTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btUniformScalingShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"


///
#include "BulletCollision/CollisionShapes/btShapeHull.h"

#include "LinearMath/btTransformUtil.h"


#include "LinearMath/btIDebugDraw.h"
//for debugmodes

#include <stdio.h> //printf debugging

//#define USE_DISPLAY_LISTS 1
//#ifdef USE_DISPLAY_LISTS

inline void glDrawVector(const btVector3& v) { glVertex3d(v[0], v[1], v[2]); }

class GlDrawcallback : public btTriangleCallback
{

public:

	bool	m_wireframe;

	GlDrawcallback()
		:m_wireframe(false)
	{
	}

	virtual void processTriangle(btVector3* triangle,int partId, int triangleIndex)
	{

		(void)triangleIndex;
		(void)partId;
		if (m_wireframe)
		{
			glBegin(GL_LINES);
			glColor3f(1, 0, 0);
			glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			glColor3f(0, 1, 0);
			glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
			glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			glColor3f(0, 0, 1);
			glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
			glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			glEnd();
		} else
		{
			glBegin(GL_TRIANGLES);
			//glColor3f(1, 1, 1);
			glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());

			glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
			glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			glEnd();
		}
	}
};

void Renderer::drawOpenGLx(btScalar* m, const btCollisionShape* shape, const btVector3& defaultColor,int	debugMode,const btVector3& worldBoundsMin,const btVector3& worldBoundsMax, BodyProcess* process)
{

    btVector3 color(defaultColor);
    if (process!=NULL) {
        color = process->color;
    }

    
	if (shape->getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE)
	{
		btVector3 org(m[12], m[13], m[14]);
		btVector3 dx(m[0], m[1], m[2]);
		btVector3 dy(m[4], m[5], m[6]);
//		btVector3 dz(m[8], m[9], m[10]);
		const btBoxShape* boxShape = static_cast<const btBoxShape*>(shape);
		btVector3 halfExtent = boxShape->getHalfExtentsWithMargin();
		dx *= halfExtent[0];
		dy *= halfExtent[1];
//		dz *= halfExtent[2];
		glColor3f(1,1,1);
		glDisable(GL_LIGHTING);
		glLineWidth(2);

		glBegin(GL_LINE_LOOP);
		glDrawVector(org - dx - dy);
		glDrawVector(org - dx + dy);
		glDrawVector(org + dx + dy);
		glDrawVector(org + dx - dy);
		glEnd();
		return;
	}
	else if((shape->getShapeType() == BOX_SHAPE_PROXYTYPE) && (debugMode & btIDebugDraw::DBG_FastWireframe))
	{
		btVector3 org(m[12], m[13], m[14]);
		btVector3 dx(m[0], m[1], m[2]);
		btVector3 dy(m[4], m[5], m[6]);
		btVector3 dz(m[8], m[9], m[10]);
		const btBoxShape* boxShape = static_cast<const btBoxShape*>(shape);
		btVector3 halfExtent = boxShape->getHalfExtentsWithMargin();
		dx *= halfExtent[0];
		dy *= halfExtent[1];
		dz *= halfExtent[2];
		glBegin(GL_LINE_LOOP);
		glDrawVector(org - dx - dy - dz);
		glDrawVector(org + dx - dy - dz);
		glDrawVector(org + dx + dy - dz);
		glDrawVector(org - dx + dy - dz);
		glDrawVector(org - dx + dy + dz);
		glDrawVector(org + dx + dy + dz);
		glDrawVector(org + dx - dy + dz);
		glDrawVector(org - dx - dy + dz);
		glEnd();
		glBegin(GL_LINES);
		glDrawVector(org + dx - dy - dz);
		glDrawVector(org + dx - dy + dz);
		glDrawVector(org + dx + dy - dz);
		glDrawVector(org + dx + dy + dz);
		glDrawVector(org - dx - dy - dz);
		glDrawVector(org - dx + dy - dz);
		glDrawVector(org - dx - dy + dz);
		glDrawVector(org - dx + dy + dz);
		glEnd();
		return;
	}

	glPushMatrix();
	btglMultMatrix(m);


	if (shape->getShapeType() == UNIFORM_SCALING_SHAPE_PROXYTYPE)
	{
		const btUniformScalingShape* scalingShape = static_cast<const btUniformScalingShape*>(shape);
		const btConvexShape* convexShape = scalingShape->getChildShape();
		float	scalingFactor = (float)scalingShape->getUniformScalingFactor();
		{
			btScalar tmpScaling[4][4]={{scalingFactor,0,0,0},
			{0,scalingFactor,0,0},
			{0,0,scalingFactor,0},
			{0,0,0,1}};

			drawOpenGLx( (btScalar*)tmpScaling,convexShape,color,debugMode,worldBoundsMin,worldBoundsMax, process);
		}
		glPopMatrix();
		return;
	}

	if (shape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
	{
		const btCompoundShape* compoundShape = static_cast<const btCompoundShape*>(shape);
		for (int i=compoundShape->getNumChildShapes()-1;i>=0;i--)
		{
			btTransform childTrans = compoundShape->getChildTransform(i);
			const btCollisionShape* colShape = compoundShape->getChildShape(i);
			btScalar childMat[16];
			childTrans.getOpenGLMatrix(childMat);
			drawOpenGLx(childMat,colShape,color,debugMode,worldBoundsMin,worldBoundsMax, process);
		}

	} else
	{
		if(m_textureenabled&&(!m_textureinitialized))
		{
			GLubyte*	image=new GLubyte[256*256*3];
			for(int y=0;y<256;++y)
			{
				const int	t=y>>4;
				GLubyte*	pi=image+y*256*3;
				for(int x=0;x<256;++x)
				{
					const int		s=x>>4;
					const GLubyte	b=180;
					GLubyte			c=b+((s+t&1)&1)*(255-b);
					pi[0]=pi[1]=pi[2]=c;pi+=3;
				}
			}

			glGenTextures(1,(GLuint*)&m_texturehandle);
			glBindTexture(GL_TEXTURE_2D,m_texturehandle);
			glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
			glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
			glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR_MIPMAP_LINEAR);
			glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
			glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
			gluBuild2DMipmaps(GL_TEXTURE_2D,3,256,256,GL_RGB,GL_UNSIGNED_BYTE,image);
			delete[] image;


		}

		glMatrixMode(GL_TEXTURE);
		glLoadIdentity();
		glScalef(0.025f,0.025f,0.025f);
		glMatrixMode(GL_MODELVIEW);

		static const GLfloat	planex[]={1,0,0,0};
		//	static const GLfloat	planey[]={0,1,0,0};
			static const GLfloat	planez[]={0,0,1,0};
			glTexGenfv(GL_S,GL_OBJECT_PLANE,planex);
			glTexGenfv(GL_T,GL_OBJECT_PLANE,planez);
			glTexGeni(GL_S,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
			glTexGeni(GL_T,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
			glEnable(GL_TEXTURE_GEN_S);
			glEnable(GL_TEXTURE_GEN_T);
			glEnable(GL_TEXTURE_GEN_R);
			m_textureinitialized=true;




		//drawCoordSystem();

		//glPushMatrix();
		glEnable(GL_COLOR_MATERIAL);
		if(m_textureenabled)
		{
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D,m_texturehandle);
		} else
		{
			glDisable(GL_TEXTURE_2D);
		}


		glColor3f(color.x(),color.y(), color.z());

		bool useWireframeFallback = true;

		if (!(debugMode & btIDebugDraw::DBG_DrawWireframe))
		{
			///you can comment out any of the specific cases, and use the default

			///the benefit of 'default' is that it approximates the actual collision shape including collision margin
			//int shapetype=m_textureenabled?MAX_BROADPHASE_COLLISION_TYPES:shape->getShapeType();
			int shapetype=shape->getShapeType();
			switch (shapetype)
			{

				case SPHERE_SHAPE_PROXYTYPE:
				{
					const btSphereShape* sphereShape = static_cast<const btSphereShape*>(shape);
					float radius = sphereShape->getMargin();//radius doesn't include the margin, so draw with margin
					drawSphere(radius,10,10);
					useWireframeFallback = false;
					break;
				}

				case BOX_SHAPE_PROXYTYPE:
				{
					const btBoxShape* boxShape = static_cast<const btBoxShape*>(shape);
					btVector3 halfExtent = boxShape->getHalfExtentsWithMargin();

					static int indices[36] = {
						0,1,2,
						3,2,1,
						4,0,6,
						6,0,2,
						5,1,4,
						4,1,0,
						7,3,1,
						7,1,5,
						5,4,7,
						7,4,6,
						7,2,3,
						7,6,2};

					 btVector3 vertices[8]={
						btVector3(halfExtent[0],halfExtent[1],halfExtent[2]),
						btVector3(-halfExtent[0],halfExtent[1],halfExtent[2]),
						btVector3(halfExtent[0],-halfExtent[1],halfExtent[2]),
						btVector3(-halfExtent[0],-halfExtent[1],halfExtent[2]),
						btVector3(halfExtent[0],halfExtent[1],-halfExtent[2]),
						btVector3(-halfExtent[0],halfExtent[1],-halfExtent[2]),
						btVector3(halfExtent[0],-halfExtent[1],-halfExtent[2]),
						btVector3(-halfExtent[0],-halfExtent[1],-halfExtent[2])};
#if 1
					glBegin (GL_TRIANGLES);
					int si=36;
					for (int i=0;i<si;i+=3)
					{
						const btVector3& v1 = vertices[indices[i]];;
						const btVector3& v2 = vertices[indices[i+1]];
						const btVector3& v3 = vertices[indices[i+2]];
						btVector3 normal = (v3-v1).cross(v2-v1);
						normal.normalize ();
						glNormal3f(normal.getX(),normal.getY(),normal.getZ());
						glVertex3f (v1.x(), v1.y(), v1.z());
						glVertex3f (v2.x(), v2.y(), v2.z());
						glVertex3f (v3.x(), v3.y(), v3.z());

					}
					glEnd();
#endif

					useWireframeFallback = false;
					break;
				}



#if 0

			case CONE_SHAPE_PROXYTYPE:
				{
					const btConeShape* coneShape = static_cast<const btConeShape*>(shape);
					int upIndex = coneShape->getConeUpIndex();
					float radius = coneShape->getRadius();//+coneShape->getMargin();
					float height = coneShape->getHeight();//+coneShape->getMargin();
					switch (upIndex)
					{
					case 0:
						glRotatef(90.0, 0.0, 1.0, 0.0);
						break;
					case 1:
						glRotatef(-90.0, 1.0, 0.0, 0.0);
						break;
					case 2:
						break;
					default:
						{
						}
					};

					glTranslatef(0.0, 0.0, -0.5*height);
					glutSolidCone(radius,height,10,10);
					useWireframeFallback = false;
					break;

				}
#endif

			case STATIC_PLANE_PROXYTYPE:
				{
					const btStaticPlaneShape* staticPlaneShape = static_cast<const btStaticPlaneShape*>(shape);
					btScalar planeConst = staticPlaneShape->getPlaneConstant();
					const btVector3& planeNormal = staticPlaneShape->getPlaneNormal();
					btVector3 planeOrigin = planeNormal * planeConst;
					btVector3 vec0,vec1;
					btPlaneSpace1(planeNormal,vec0,vec1);
					btScalar vecLen = 100.f;
					btVector3 pt0 = planeOrigin + vec0*vecLen;
					btVector3 pt1 = planeOrigin - vec0*vecLen;
					btVector3 pt2 = planeOrigin + vec1*vecLen;
					btVector3 pt3 = planeOrigin - vec1*vecLen;
					glBegin(GL_LINES);
					glVertex3f(pt0.getX(),pt0.getY(),pt0.getZ());
					glVertex3f(pt1.getX(),pt1.getY(),pt1.getZ());
					glVertex3f(pt2.getX(),pt2.getY(),pt2.getZ());
					glVertex3f(pt3.getX(),pt3.getY(),pt3.getZ());
					glEnd();


					break;

				}

/*
			case CYLINDER_SHAPE_PROXYTYPE:
				{
					const btCylinderShape* cylinder = static_cast<const btCylinderShape*>(shape);
					int upAxis = cylinder->getUpAxis();


					float radius = cylinder->getRadius();
					float halfHeight = cylinder->getHalfExtentsWithMargin()[upAxis];

					drawCylinder(radius,halfHeight,upAxis);

					break;
				}
*/

			case MULTI_SPHERE_SHAPE_PROXYTYPE:
			{
				const btMultiSphereShape* multiSphereShape = static_cast<const btMultiSphereShape*>(shape);

				btTransform childTransform;
				childTransform.setIdentity();


				for (int i = multiSphereShape->getSphereCount()-1; i>=0;i--)
				{
					btSphereShape sc(multiSphereShape->getSphereRadius(i));
					childTransform.setOrigin(multiSphereShape->getSpherePosition(i));
					btScalar childMat[16];
					childTransform.getOpenGLMatrix(childMat);
					drawOpenGLx(childMat,&sc,color,debugMode,worldBoundsMin,worldBoundsMax, process);
				}

				break;
			}

			default:
				{


					if (shape->isConvex())
					{
						ShapeCache*	sc=cache((btConvexShape*)shape);
#if 0
						btConvexShape* convexShape = (btConvexShape*)shape;
						if (!shape->getUserPointer())
						{
							//create a hull approximation
							void* mem = btAlignedAlloc(sizeof(btShapeHull),16);
							btShapeHull* hull = new(mem) btShapeHull(convexShape);

							///cleanup memory
							m_shapeHulls.push_back(hull);

							btScalar margin = shape->getMargin();
							hull->buildHull(margin);
							convexShape->setUserPointer(hull);


							//	printf("numTriangles = %d\n", hull->numTriangles ());
							//	printf("numIndices = %d\n", hull->numIndices ());
							//	printf("numVertices = %d\n", hull->numVertices ());


						}
#endif



						//if (shape->getUserPointer())
						{
							//glutSolidCube(1.0);
							btShapeHull* hull = &sc->m_shapehull/*(btShapeHull*)shape->getUserPointer()*/;


							if (hull->numTriangles () > 0)
							{
								int index = 0;
								const unsigned int* idx = hull->getIndexPointer();
								const btVector3* vtx = hull->getVertexPointer();

								glBegin (GL_TRIANGLES);

								for (int i = 0; i < hull->numTriangles (); i++)
								{
									int i1 = index++;
									int i2 = index++;
									int i3 = index++;
									btAssert(i1 < hull->numIndices () &&
										i2 < hull->numIndices () &&
										i3 < hull->numIndices ());

									int index1 = idx[i1];
									int index2 = idx[i2];
									int index3 = idx[i3];
									btAssert(index1 < hull->numVertices () &&
										index2 < hull->numVertices () &&
										index3 < hull->numVertices ());

									btVector3 v1 = vtx[index1];
									btVector3 v2 = vtx[index2];
									btVector3 v3 = vtx[index3];
									btVector3 normal = (v3-v1).cross(v2-v1);
									normal.normalize ();
									glNormal3f(normal.getX(),normal.getY(),normal.getZ());
									glVertex3f (v1.x(), v1.y(), v1.z());
									glVertex3f (v2.x(), v2.y(), v2.z());
									glVertex3f (v3.x(), v3.y(), v3.z());

								}
								glEnd ();

							}
						}
					}
				}
			}

		}


		glNormal3f(0,1,0);


		/// for polyhedral shapes
		if (debugMode==btIDebugDraw::DBG_DrawFeaturesText && (shape->isPolyhedral()))
		{
			btPolyhedralConvexShape* polyshape = (btPolyhedralConvexShape*) shape;

			{

				glColor3f(1.f, 1.f, 1.f);
				int i;
				for (i=0;i<polyshape->getNumVertices();i++)
				{
					btVector3 vtx;
					polyshape->getVertex(i,vtx);
					char buf[12];
					sprintf(buf," %d",i);
					//btDrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				}

				for (i=0;i<polyshape->getNumPlanes();i++)
				{
					btVector3 normal;
					btVector3 vtx;
					polyshape->getPlane(normal,vtx,i);
					//btScalar d = vtx.dot(normal);

					//char buf[12];
					//sprintf(buf," plane %d",i);
					//btDrawString(BMF_GetFont(BMF_kHelvetica10),buf);

				}
			}

		}


#ifdef USE_DISPLAY_LISTS

		if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE||shape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
		{
			GLuint dlist =   OGL_get_displaylist_for_shape((btCollisionShape * )shape);
			if (dlist)
			{
				glCallList(dlist);
			}
			else
			{
#else
		if (shape->isConcave() && !shape->isInfinite())
		{
			btConcaveShape* concaveMesh = (btConcaveShape*) shape;

			GlDrawcallback drawCallback;
			drawCallback.m_wireframe = (debugMode & btIDebugDraw::DBG_DrawWireframe)!=0;

			concaveMesh->processAllTriangles(&drawCallback,worldBoundsMin,worldBoundsMax);

		}
#endif

#ifdef USE_DISPLAY_LISTS
	}
}
#endif



        process->draw();


	}
	glPopMatrix();

}

