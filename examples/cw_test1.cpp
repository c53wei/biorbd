#include "biorbd.h"

#include "LuaTest/luamodel.h"
#include "LuaTest/luatables.h"
#include "LuaTest/luatypes.h"
extern "C"
{
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
}

#include <rbdl/rbdl.h>
#include "rbdl/rbdl_errors.h"

#include "Utils/Path.h"
#include "Utils/String.h"
#include "Utils/Vector3d.h"

#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <cstring>
#include <iomanip>
#include <map>
#include <filesystem>


using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Addons;
using namespace RigidBodyDynamics::Math;
using namespace BIORBD_NAMESPACE;


int main()

{
    BiorbdEigen3::Model model("sagar_ior.lua");

    cout << model.nbSegment() << endl; // 16
    cout << model.nbMarkers() << endl; // 49
    
    Writer::writeModel(model, "test_ior.bioMod");
    
    BiorbdEigen3::Model model2("test_ior.bioMod");

    cout << model2.nbSegment() << endl; // 16
    cout << model2.nbMarkers() << endl; // 49
    
    
    return 0;
}

