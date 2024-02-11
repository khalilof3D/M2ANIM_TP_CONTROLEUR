#include "FSM.h"
#include <iostream>
using namespace std;

// Interpolation lin�aire :
// Doit retourner la valeur dans l'intervalle [x1,x2] qui correspond au placement de y1 dans l'intervalle [0,y2]
float linear_interpolate (float x1, float x2, float y1, float y2) {
    if (y1<=0) return x1;
    if (y1>=y2) return x2;
    float p = y1 / y2;
    return (1-p) * x1 + p * x2;
}



//angle départ
//angle arrivée
//vitesse départ
//vitesse arrivée
//t
float hermiteCubicInterpolate(float p0, float p1, float v0, float v1, float t) {

    float t3 = t * t * t;
    float t2 = t * t;
    float h00 = 2 * t3 - 3 * t2 + 1; // Coefficient pour p0
    float h10 = t3 - 2 * t2 + t;     // Coefficient pour v0
    float h01 = -2 * t3 + 3 * t2;    // Coefficient pour p1
    float h11 = t3 - t2;             // Coefficient pour v1
    return h00 * p0 + h10 * v0 + h01 * p1 + h11 * v1;
}



FSM::FSM() { }

FSM::~FSM() {}

void FSM::update(double Dt, std::vector<float> currentAnglesLocal, std::vector<float> currentAnglesGlobal) {
    // Mise � jour du temps �coul� dans l'�tat
    m_timeInState += Dt;
    // L'�tat courant
    State s = m_states[m_currentState];
    // Si la machine a un seul �tat on reste dedans
    if (m_nbStates==1) return;
    // Transition si dur�e �coul�e
    if (m_timeInState > s.transitionTime) {
        // Remise � z�ro du temps �coul�
        m_timeInState = 0.0;
        //m_anglespeed.clear();


        // R�cup�ration des angles courants en tant que point de d�part de l'�tat (animation fluide)
        for (unsigned int i=0;i<s.targetAngles.size();i++) {

            if (m_states[s.nextState].targetLocal[i]) {

                    m_anglespeed[i] = currentAnglesLocal[i] - m_anglesAtTransition[i];

                    m_anglesAtTransition[i] = currentAnglesLocal[i];


                      }

                else {

                    m_anglespeed[i]= currentAnglesGlobal[i] - m_anglesAtTransition[i];

                    m_anglesAtTransition[i] = currentAnglesGlobal[i];

                     }
        }
        // Passage � l'�tat suivant
        m_currentState = s.nextState;
    }
}
//bool h_interpolation
std::vector<float> FSM::getCurrentTargetAngles() const {
    // Poses cl�s non interpol�es si un seul �tat ou transition directe
    if (m_nbStates==1 || m_states[m_currentState].transitionTime==0.0) return m_states[m_currentState].targetAngles;

    // Poses cl�s interpol�es sinon
    std::vector<float> targetAnglesInterpolated;
    float y1 = m_timeInState; // dur�e �coul�e depuis le d�but de l'�tat courant
    float y2 = m_states[m_currentState].transitionTime;  // dur�e max avant transition



    if (m_interpolation){




        for (unsigned int i=0; i<m_states[m_currentState].targetAngles.size();i++) {
            float x1 = m_anglesAtTransition[i]; // l'angle au d�but de l'�tat
            float x2 = m_states[m_currentState].targetAngles[i]; // la cible courante
            if (x1==x2) targetAnglesInterpolated.push_back(x1); // pas d'interpolation si identiques
            else targetAnglesInterpolated.push_back(linear_interpolate(x1,x2,y1,y2)); // interpolation lin�aire sinon


        }
        }

           else {
                for (unsigned int i=0; i<m_states[m_currentState].targetAngles.size();i++) {

                    float p0 = m_anglesAtTransition[i];
                    float p1 = m_states[m_currentState].targetAngles[i];
                    float v0 = m_anglespeed[i];

                    //Pour mouvement à vitesse stable

                    float v1_target = manualspeed ? v1 : v0;
                    //cout<<v1_target<<endl;

                    //Pour mouvement à vitesse non stable
                    //float v1 = y2 != 0 ? (p1 - p0) / y2 : v1;




                    float t = y1/y2;
                    if (p0==p1) targetAnglesInterpolated.push_back(p0); // pas d'interpolation si identiques
                    else targetAnglesInterpolated.push_back(hermiteCubicInterpolate(p0,p1,v0,v1_target,t));
                }




            }





    //cout << getName() << endl;
    return targetAnglesInterpolated;
}





FSM_Stand::~FSM_Stand() {};
string FSM_Stand::getName() const {
    return "Stand"; }
FSM_Stand::FSM_Stand() {

    // La machine � �tats finis pour un mouvement stable debout
    // un seul �tat, toutes les articulations � z�ro dans le monde
    m_nbStates = 1;
    m_currentState = 0;
    State s;
    // ETAT 0 //
    s.ID = 0;
    s.nextState = 0;
    s.transitionTime = 0.0;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);

}


FSM_Sad::~FSM_Sad() {};
string FSM_Sad::getName() const {
    return "Sad"; }
FSM_Sad::FSM_Sad() {
    // La machine � �tats finis pour un mouvement stable debout
    // un seul �tat, toutes les articulations � z�ro dans le monde
    m_nbStates = 5;
    m_currentState = 0;
    State s;
    s.ID = 0;
    s.nextState = 1;
    s.transitionTime = 1.0;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);







    // ETAT 0 //
    s.ID = 1;
    s.nextState = 2;
    s.transitionTime = 0.8;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(-0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(-0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.4); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(-0.4); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.8); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);


    s.ID = 2;
    s.nextState = 3;
    s.transitionTime = 0.4;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(-0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(-0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.8); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);

    s.ID = 3;
    s.nextState = 4;
    s.transitionTime = 0.8;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(-0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(-0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(-0.4); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.4); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.8); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);

    s.ID = 4;
    s.nextState = 1;
    s.transitionTime = 0.4;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(-0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(-0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.8); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);


        // copie des premi�res valeurs dans m_anglesAtTransition
    m_anglespeed.resize(m_states[m_currentState].targetAngles.size(), 0.0f);


    for (unsigned int i=0;i<s.targetAngles.size();i++) {
        m_anglesAtTransition.push_back(m_states[m_currentState].targetAngles[i]);
        m_anglespeed[i] = 0.0f;
    }

}






FSM_MoonWalker::~FSM_MoonWalker() {};
string FSM_MoonWalker::getName() const {
    return "MoonWalker"; }
FSM_MoonWalker::FSM_MoonWalker() {
    // La machine � �tats finis pour un mouvement stable debout
    // un seul �tat, toutes les articulations � z�ro dans le monde
    m_nbStates = 10;
    m_currentState = 0;
    State s;
    // ETAT 0 //
    s.ID = 0;
    s.nextState = 2;
    s.transitionTime = 1;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);


        s.ID = 1;
    s.nextState = 2;
    s.transitionTime = 1;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(1); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);



    s.ID = 2;
    s.nextState = 3;
    s.transitionTime = 1;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(-0.8); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(-0.5); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.5); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);

    //tirement de l'autre pied
    s.ID = 3;
    s.nextState = 4;
    s.transitionTime = 1;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(-0.8); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.7); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(-0.4); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);


    //////////////////////////////////////////////////////////// symetrie
        s.ID = 4;
    s.nextState = 6;
    s.transitionTime = 1;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);


        s.ID = 5;
    s.nextState = 6;
    s.transitionTime = 1;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(1); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);



    s.ID = 6;
    s.nextState = 7;
    s.transitionTime = 1;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(-0.8); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.5); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(-0.5); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);

    //tirement de l'autre pied
    s.ID = 7;
    s.nextState = 0;
    s.transitionTime = 1;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(-0.8); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(-0.4); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.7); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);


    // Etat déclenché
    s.ID = 8;
    s.nextState = 9;
    s.transitionTime = 3;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(-0.2); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(-0.2); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(-1.0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(-1.0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.6); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);



    s.ID = 9;
    s.nextState = 0;
    s.transitionTime = 15;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(-0.3); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(-0.3); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(-1.3); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(-1.3); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.9); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);


     m_anglespeed.resize(m_states[m_currentState].targetAngles.size(), 0.0f);



    // copie des premi�res valeurs dans m_anglesAtTransition
    for (unsigned int i=0;i<s.targetAngles.size();i++) {
        m_anglesAtTransition.push_back(m_states[m_currentState].targetAngles[i]);
        m_anglespeed[i] = 0;

    }
}

FSM_Walk::~FSM_Walk() {};
string FSM_Walk::getName() const {
    return "Walk"; }
FSM_Walk::FSM_Walk() {
    m_nbStates = 6;
    m_currentState = 0;
    State s;
    // ETAT 0 //
    s.ID = 0;
    s.nextState = 1;
    s.transitionTime = 0.08;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(-1.53); s.targetLocal.push_back(true);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.48); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.06); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 1 //
    s.ID = 1;
    s.nextState = 2;
    s.transitionTime = 0.12;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(1.12); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.06); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 2 //
    s.ID = 2;
    s.nextState = 3;
    s.transitionTime = 0.12;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false); //CHEVILLE_GAUCHE
    s.targetAngles.push_back(-0.56); s.targetLocal.push_back(true); //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false); //GENOU_GAUCHE
    s.targetAngles.push_back(-0.13); s.targetLocal.push_back(true); //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false); //HANCHE_GAUCHE
    s.targetAngles.push_back(-0.23); s.targetLocal.push_back(true); //HANCHE_DROIT
    s.targetAngles.push_back(-0.06); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 3 //
    s.ID = 3;
    s.nextState = 4;
    s.transitionTime = 0.08;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(-1.53); s.targetLocal.push_back(true);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.48); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.06); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 4 //
    s.ID = 4;
    s.nextState = 5;
    s.transitionTime = 0.12;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(1.12); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.06); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 5 //
    s.ID = 5;
    s.nextState = 0;
    s.transitionTime = 0.12;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(-0.56); s.targetLocal.push_back(true); //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false); //CHEVILLE_DROIT
    s.targetAngles.push_back(-0.13); s.targetLocal.push_back(true); //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false); //GENOU_DROIT
    s.targetAngles.push_back(-0.23); s.targetLocal.push_back(true); //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false); //HANCHE_DROIT
    s.targetAngles.push_back(-0.06); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);

     m_anglespeed.resize(m_states[m_currentState].targetAngles.size(), 0.0f);
    // copie des premi�res valeurs dans m_anglesAtTransition
    for (unsigned int i=0;i<s.targetAngles.size();i++) {
        m_anglesAtTransition.push_back(m_states[m_currentState].targetAngles[i]);
        m_anglespeed[i] = 0;

    }

}





// si je joue sur y un trigger d'�tat ?
 // je veux déclarer la classe fsm jump au fichier fsm.h

FSM_Jump::~FSM_Jump() {};
string FSM_Jump::getName() const {
    return "Jump"; }
FSM_Jump::FSM_Jump() {
        // Create a new state for the jump
        m_nbStates = 2;
        m_currentState = 0;
        State s;

        s.ID = 1;
        s.nextState = 0;
        s.transitionTime = 0.1; // No transition time for the jump state
        s.targetAngles.clear();
        s.targetLocal.clear();

        s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
        s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
        s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
        s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
        s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
        s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_DROIT
        s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC

        // Add the jump state to the FSM
        m_states.push_back(s);

        s.ID = 0;
        s.nextState = 1;
        s.transitionTime = 0.12;
        s.targetAngles.clear(); s.targetLocal.clear();
        s.targetAngles.push_back(0.0); s.targetLocal.push_back(false); //CHEVILLE_GAUCHE
        s.targetAngles.push_back(0.0); s.targetLocal.push_back(false); //CHEVILLE_DROIT
        s.targetAngles.push_back(-0.7); s.targetLocal.push_back(false); //GENOU_GAUCHE
        s.targetAngles.push_back(-0.7); s.targetLocal.push_back(false); //GENOU_DROIT
        s.targetAngles.push_back(0.0); s.targetLocal.push_back(false); //HANCHE_GAUCHE
        s.targetAngles.push_back(0.0); s.targetLocal.push_back(false); //HANCHE_DROIT
        s.targetAngles.push_back(1.0); s.targetLocal.push_back(false);  //TRONC
        m_states.push_back(s);


         m_anglespeed.resize(m_states[m_currentState].targetAngles.size(), 0.0f);
        // Update the angles at transition to the jump state
        for (unsigned int i=0;i<s.targetAngles.size();i++) {
        m_anglesAtTransition.push_back(m_states[m_currentState].targetAngles[i]);
        m_anglespeed[i] = 0;
    }

    }




    FSM_Honte::~FSM_Honte() {};
    string FSM_Honte::getName() const {
    return "Honte"; }
    FSM_Honte::FSM_Honte() {
    m_nbStates = 3;
    m_currentState = 0;
    State s;
    // ETAT 0 //
    s.ID = 0;
    s.nextState = 1;
    s.transitionTime = 0.3;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 1 //
    s.ID = 1;
    s.nextState = 2;
    s.transitionTime = 0.7;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(-0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(-0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(-2); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(-2); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.4); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
        // ETAT 1 //
    s.ID = 2;
    s.nextState = 0;
    s.transitionTime = 0.4;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(-0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(-0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(true);  //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(true);  //GENOU_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.2); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);


     m_anglespeed.resize(m_states[m_currentState].targetAngles.size(), 0.0f);
    // copie des premi�res valeurs dans m_anglesAtTransition
    for (unsigned int i=0;i<s.targetAngles.size();i++) {
        m_anglesAtTransition.push_back(m_states[m_currentState].targetAngles[i]);
        m_anglespeed[i] = 0;
    }
}


FSM_Dance::~FSM_Dance() {};
string FSM_Dance::getName() const {
    return "Dance"; }
FSM_Dance::FSM_Dance() {
    m_nbStates = 6;
    m_currentState = 0;
    State s;
    // ETAT 0 //
    s.ID = 0;
    s.nextState = 1;
    s.transitionTime = 0.08;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(true);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 1 //
    s.ID = 1;
    s.nextState = 2;
    s.transitionTime = 0.12;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(2.0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 2 //
    s.ID = 2;
    s.nextState = 3;
    s.transitionTime = 0.12;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false); //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(true); //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false); //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(true); //GENOU_DROIT
    s.targetAngles.push_back(1.0); s.targetLocal.push_back(false); //HANCHE_GAUCHE
    s.targetAngles.push_back(-1); s.targetLocal.push_back(true); //HANCHE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 3 //
    s.ID = 3;
    s.nextState = 4;
    s.transitionTime = 0.08;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(true);  //GENOU_DROIT
    s.targetAngles.push_back(2.0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(-2); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 4 //
    s.ID = 4;
    s.nextState = 0;
    s.transitionTime = 0.12;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(1.0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(-1); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 5 //
    s.ID = 5;
    s.nextState = 0;
    s.transitionTime = 0.12;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0); s.targetLocal.push_back(true); //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false); //CHEVILLE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(true); //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false); //GENOU_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(true); //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false); //HANCHE_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);


     m_anglespeed.resize(m_states[m_currentState].targetAngles.size(), 0.0f);
    // copie des premi�res valeurs dans m_anglesAtTransition
    for (unsigned int i=0;i<s.targetAngles.size();i++) {
        m_anglesAtTransition.push_back(m_states[m_currentState].targetAngles[i]);
        m_anglespeed[i] = 0;
    }

}






 // créer une fonction fsm jump qui prend en paramètre la hauteur du saut et qui fait un saut de cette hauteur



     FSM_Jumper::~FSM_Jumper() {};
     string FSM_Jumper::getName() const {
    return "Jumper"; }
     FSM_Jumper::FSM_Jumper() {
    m_nbStates = 3;
    m_currentState = 0;
    State s;
    // ETAT 0 //
    s.ID = 0;
    s.nextState = 1;
    s.transitionTime = 0.3;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
    // ETAT 1 //
    s.ID = 1;
    s.nextState = 2;
    s.transitionTime = 0.1;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(-0.1); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(-0.1); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(-2.3); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(-2.3); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(2); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(2); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.4); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
        // ETAT 1 //
    s.ID = 2;
    s.nextState = 0;
    s.transitionTime = 0.4;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.1); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.1); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(-0.5); s.targetLocal.push_back(true);  //GENOU_GAUCHE
    s.targetAngles.push_back(-0.5); s.targetLocal.push_back(true);  //GENOU_DROIT
    s.targetAngles.push_back(0); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);


    // copie des premi�res valeurs dans m_anglesAtTransition
    m_anglespeed.resize(m_states[m_currentState].targetAngles.size(), 0.0f);

    for (unsigned int i=0;i<s.targetAngles.size();i++) {
        m_anglesAtTransition.push_back(m_states[m_currentState].targetAngles[i]);
        m_anglespeed[i] = 0;
    }

}


