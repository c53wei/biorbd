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

//namespace fs = __fs::filesystem;

int main()

{
    BiorbdEigen3::Model model("sagar_ior.lua");
    // TODO: Add Markers
    
    LuaTable model_table = LuaTable::fromFile("sagar_ior.lua");
    int frame_count = model_table["frames"].length();
    int marker_count;
    
    for(int i=1; i <= frame_count; ++i)
    {
        utils::String body_name(model_table["frames"][i]["name"].getDefault<std::string>(""));
        std::cout << "Segement: "<< body_name << std::endl;
        
        utils::String parent_name( model_table["frames"][i]["parent"].get<std::string>());
        std::cout << "Parent: " << parent_name << std::endl;
        
  
        std::vector<LuaKey> marker_keys = model_table["frames"][i]["markers"].keys();
        
        // Insantiate parameters for AddMarker()
        utils::String marker_name;
        int parent_int = 0;
        utils::Vector3d pos(0,0,0);
        bool technical = true;
        bool anatomical = false;
        utils::String axesToRemove;
        for(int j = 0; j < marker_keys.size(); ++j)
        {
            marker_name = marker_keys[j].string_value;
            parent_int = model.GetBodyId(body_name.c_str());
            // If parent_int still equals zero, no name has concurred
            utils::Error::check(model.IsBodyId(parent_int),
                                        "Wrong name in a segment");
            pos = model_table["frames"][i]["markers"][marker_name.c_str()].getDefault<RigidBodyDynamics::Math::Vector3d>(RigidBodyDynamics::Math::Vector3d::Zero());
            cout << marker_name + ": " << pos << endl;
            model.addMarker(pos, marker_name, parent_name, technical, anatomical, axesToRemove,
                             parent_int);
        }
        
        cout << model.nbMarkers() << endl;

    }
    return 0;
}

