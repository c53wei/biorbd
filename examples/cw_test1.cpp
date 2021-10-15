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
using namespace BIORBD_NAMESPACE;

namespace fs = __fs::filesystem;

int main()

{
    // Ultimate goal: Get the following line working:
//    BiorbdEigen3::Model force_lua("sagar_ior.lua");
    BiorbdEigen3::Model model("sagar_ior.lua");
    
    
//    cout << "Current path is " << fs::current_path() << '\n'; // (1)
//    fs::current_path(fs::temp_directory_path()); // (3)
//    cout << "Current path is " << fs::current_path() << '\n';
    return 0;
}
