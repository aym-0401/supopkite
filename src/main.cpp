//
// Created by Aymeric HALÉ, Charlotte MILLET & François OLLITRAULT.
//
// We give our explicit authorization to use, distribute and modify this code.
// Some portions of the code are authorized modifications of other works indicated
// by their copyrights.
//

#include <iostream>

#include "tracking.hpp"
#include "orientation_4pts.hpp"
#include "generate_database.hpp"

using namespace std;

int main(int argc, char *argv[]) {
    
    string usage = "\nUsage:\n\nTracking mode: ./supopkite video.mp4 -tracking\nOrientation mode: ./supopkite video.mp4 -orientation\nGenerate dabate: ./supopkite model3D.stl -newdata\n";
    
    char* const filepath = argv[1];
    char* const mode = argv[2];
    
    if (argc == 1) {
        cout << usage << endl;
    } else if (strcmp(mode, "-tracking") == 0) {
        tracking(filepath);
    } else if (strcmp(mode, "-orientation") == 0) {
        orientation_4pts(filepath);
    } else if (strcmp(mode, "-newdata") == 0) {
        generate_database(filepath);
    } else {
        cout << usage << endl;
    }
    
    return 0;
}

// ------------------
// Visual Studio Code
// ------------------
//
// Exécuter le programme : Ctrl+F5 ou menu Déboguer > Exécuter sans débogage
// Déboguer le programme : F5 ou menu Déboguer > Démarrer le débogage

// Astuces pour bien démarrer :
//   1. Utilisez la fenêtre Explorateur de solutions pour ajouter des fichiers et les gérer.
//   2. Utilisez la fenêtre Team Explorer pour vous connecter au contrôle de code source.
//   3. Utilisez la fenêtre Sortie pour voir la sortie de la génération et d'autres messages.
//   4. Utilisez la fenêtre Liste d'erreurs pour voir les erreurs.
//   5. Accédez à Projet > Ajouter un nouvel élément pour créer des fichiers de code, ou à Projet > Ajouter un élément existant pour ajouter des fichiers de code existants au projet.
//   6. Pour rouvrir ce projet plus tard, accédez à Fichier > Ouvrir > Projet et sélectionnez le fichier .sln.
