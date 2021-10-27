#include "biorbd.h"
#include <iostream>

using namespace std;
using namespace BIORBD_NAMESPACE;


int main()

{
    Model model("sagar_ior.lua");

    cout << model.nbSegment() << endl; // 16
    cout << model.nbMarkers() << endl; // 49
    
    Writer::writeModel(model, "test_ior.bioMod");
    
    Model model2("test_ior.bioMod");

    cout << model2.nbSegment() << endl; // 16
    cout << model2.nbMarkers() << endl; // 49
    
    
    return 0;
}

