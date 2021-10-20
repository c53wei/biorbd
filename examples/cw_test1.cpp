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
    // Ultimate goal: Get the following line working:
//    BiorbdEigen3::Model force_lua("sagar_ior.lua");
    // Set up parameters to mimick Reader::readModelFile() and LuaModelReadFromTable
    const char* path = "sagar_ior.lua";
    BiorbdEigen3::Model model(path);
    LuaTable model_table = LuaTable::fromFile(path);
    bool verbose = true;
    
    int frame_count = model_table["frames"].length();
    // Define parent to child segment mapping dictionary
    typedef map<string, unsigned int> StringIntMap;
    StringIntMap body_table_id_map;
    
    body_table_id_map["ROOT"] = 0;
    
    
    
    for(int i=1; i<frame_count; ++i)
    {
        if (!model_table["frames"][i]["parent"].exists()) {
          throw Errors::RBDLError("Parent not defined for frame ");
        }

        utils::String body_name(model_table["frames"][i]["name"].getDefault<string>(""));
        cout << "Segement: "<< body_name << endl;
        
        utils::String parent_name( model_table["frames"][i]["parent"].get<string>());
        cout << "Parent: " << parent_name << endl;
        
        unsigned int parent_id = body_table_id_map[parent_name];
        
        // TODO: Make joint map into rotation and translation sequence
        RigidBodyDynamics::Math::MatrixNd placeholder(6, 6);
        RigidBodyDynamics::Math::MatrixNd joint_matrix (model_table["frames"][i]["joint"].getDefault<RigidBodyDynamics::Math::MatrixNd>(placeholder)
                                                                                                            );
        cout << "Initial Matrix:\n" << joint_matrix << "\n\n";
        
        char xyz[4] = "xyz";
        string trans, rot;
        
        if(joint_matrix.size())
        {
            // Rotation
            RigidBodyDynamics::Math::MatrixNd rot_mat = joint_matrix(Eigen::all, Eigen::seq(0, Eigen::last/2));
            cout << "Rotation Matrix:\n" << rot_mat << "\n";
            Eigen::Matrix<ptrdiff_t, 3, 1> res = (rot_mat.array() == 1).colwise().count();
            for(int j = 0; j < 3; ++j)
            {
                if(res[j]) {
                    rot.append(1, xyz[j]);
                }
            }
            cout << rot << "\n\n";
            // Translation matrix
            RigidBodyDynamics::Math::MatrixNd trans_mat = joint_matrix(Eigen::all, Eigen::seq(Eigen::last/2 + 1, Eigen::last));
            cout << "Translation Matrix:\n" << trans_mat << "\n";
            res = (trans_mat.array() == 1).colwise().count();
            for(int j = 0; j < 3; ++j)
            {
                if(res[j]) {
                    trans.append(1, xyz[j]);
                }
            }
            cout << trans << "\n\n";
        }
    }
    
    
    if (model_table["gravity"].exists()) {
        Vector3d gravity(model_table["gravity"].get<Vector3d>());
        model.gravity = utils::Vector3d(gravity);
        
      if (verbose) {
        cout << "gravity = " << model.gravity.transpose() << endl;
      }
    }
//    cout << "Current path is " << fs::current_path() << '\n'; // (1)
//    fs::current_path(fs::temp_directory_path()); // (3)
//    cout << "Current path is " << fs::current_path() << '\n';
    return 0;
}

