#include "biorbd.h"
#include "LuaTest/luamodel.h"
#include "LuaTest/luatables.h"
#include <rbdl/rbdl.h>
#include <string>
#include <vector>
#include <cstring>
using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Addons;
using namespace BIORBD_NAMESPACE;

int main()

{
    BiorbdEigen3::Model biorbd_model("sagar_ior.bioMod");
    BiorbdEigen3::Model arm_model("arm26.bioMod");

    RigidBodyDynamics::Model rbdl_model;
    bool modelLoaded = LuaModelReadFromFile("sagar_ior.lua", &rbdl_model, false);

    BiorbdEigen3::Model biorbd_from_lua;
    modelLoaded = LuaModelReadFromFile("sagar_ior.lua", &biorbd_from_lua, true);

    cout << "Hello World!" << endl;
    return 0;
}
    
