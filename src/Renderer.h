/* 
 * File:   Renderer.h
 * Author: me
 *
 * Created on October 24, 2010, 1:14 AM
 */

#ifndef RENDERER_H
#define	RENDERER_H

#include "GL_ShapeDrawer.h"

class Renderer : public GL_ShapeDrawer {
public:
    void drawOpenGLx(btScalar* m, const btCollisionShape* shape, const btVector3& color, int debugMode,const btVector3& worldBoundsMin,const btVector3& worldBoundsMax, int* unused);
    
};


#endif	/* RENDERER_H */

