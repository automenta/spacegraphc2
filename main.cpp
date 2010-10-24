/* 
 * File:   main.cpp
 * Author: me
 *
 * Created on October 23, 2010, 11:07 PM
 */

#include <cstdlib>

using namespace std;

#include "Spacegraph.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

#include "Humanoid.h"
#include "Bench.h"

GLDebugDrawer	gDebugDrawer;

int main(int argc,char* argv[])
{
        Spacegraph s;

        s.initPhysics();

        s.addCell(new Humanoid(&s, btVector3(1, 0.5, 0)));
        s.addCell(new Humanoid(&s, btVector3(-1, 0.5, 0)));
        s.addCell(new Bench(&s, btVector3(1.7, 1.9, 1.6)));


        //s.getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

        return glutmain(argc, argv, 1920, 1080,"SpaceGraphC",&s);
}
