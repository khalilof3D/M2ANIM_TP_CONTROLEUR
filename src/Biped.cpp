#include "Biped.h"
#include <iostream>
#include <algorithm>
using namespace std;

Biped::Biped (b2World* world) : m_world (world), m_hasFallen(false) { // Constructeur

		// Creation des corps rigides
		// ==========================

		// Proprietes communes
		b2BodyDef bodyDef;
		bodyDef.fixedRotation = false;
		bodyDef.allowSleep = false;
		bodyDef.awake = true;
		bodyDef.type = b2_dynamicBody;
		bodyDef.linearDamping = 0.01f;
		bodyDef.angularDamping = 0.01f;
        b2PolygonShape shape;
		b2FixtureDef fixture;
		fixture.shape = &shape;
		fixture.filter.groupIndex = -1; // same group and don't collide

		// PIED GAUCHE
		bodyDef.position.Set(0.05f,0.05f); // 5cm au dessus du sol
		m_bodies[PIED_GAUCHE] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.1f, 0.05f); // boite de 20cm x 10cm
        fixture.density = 5.0f;
        fixture.friction = 0.999;
        fixture.userData = (void*)PIED_GAUCHE;
		m_bodies[PIED_GAUCHE]->CreateFixture(&fixture);

		// PIED DROIT
		bodyDef.position.Set(0.05f,0.05f); // 5cm au dessus du sol
		m_bodies[PIED_DROIT] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.1f, 0.05f); // boite de 20cm x 10cm
        fixture.density = 5.0f;
        fixture.userData = (void*)PIED_DROIT;
		m_bodies[PIED_DROIT]->CreateFixture(&fixture);

		// JAMBE GAUCHE
		bodyDef.position.Set(0.0f,0.25f); // 15 cm au dessus de cheville
		m_bodies[JAMBE_GAUCHE] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.05f, 0.15f); // boite de 10cm x 30cm
        fixture.density = 5.0f;
        fixture.userData = (void*)JAMBE_GAUCHE;
		m_bodies[JAMBE_GAUCHE]->CreateFixture(&fixture);

		// JAMBE DROIT
		bodyDef.position.Set(0.0f,0.25f); // 15 cm au dessus de cheville
		m_bodies[JAMBE_DROIT] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.05f, 0.15f); // boite de 10cm x 30cm
        fixture.density = 5.0f;
        fixture.userData = (void*)JAMBE_DROIT;
		m_bodies[JAMBE_DROIT]->CreateFixture(&fixture);

		// CUISSE GAUCHE
		bodyDef.position.Set(0.0f,0.55f); // 15 cm au dessus de genou
		m_bodies[CUISSE_GAUCHE] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.05f, 0.15f); // boite de 10cm x 30cm
        fixture.density = 5.0f;
        fixture.userData = (void*)CUISSE_GAUCHE;
		m_bodies[CUISSE_GAUCHE]->CreateFixture(&fixture);

		// CUISSE DROIT
		bodyDef.position.Set(0.0f,0.55f); // 15 cm au dessus de genou
		m_bodies[CUISSE_DROIT] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.05f, 0.15f); // boite de 10cm x 30cm
        fixture.density = 5.0f;
        fixture.userData = (void*)CUISSE_DROIT;
		m_bodies[CUISSE_DROIT]->CreateFixture(&fixture);

		// TRONC
		bodyDef.position.Set(0.0f,1.0f); // 30 cm au dessus de hanche
		m_bodies[TRONC] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.1f, 0.3f); // boite de 20cm x 60cm
        fixture.density = 4.0f;
        fixture.userData = (void*)TRONC;
		m_bodies[TRONC]->CreateFixture(&fixture);

		// Creation des articulations
		// ==========================

		// Proprietes communes
		b2RevoluteJointDef jointDef;
		jointDef.lowerAngle = -0.5f * b2_pi;
		jointDef.upperAngle = 0.5f * b2_pi;
		jointDef.enableLimit = true;

		// CHEVILLE GAUCHE
		jointDef.Initialize(m_bodies[PIED_GAUCHE],m_bodies[JAMBE_GAUCHE],m_bodies[JAMBE_GAUCHE]->GetWorldCenter()+b2Vec2(0.0f,-0.15f));
		m_joints[CHEVILLE_GAUCHE] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
		m_currentAnglesLocal.push_back(0.0);
		m_currentAnglesGlobal.push_back(0.0);

		// CHEVILLE DROIT
		jointDef.Initialize(m_bodies[PIED_DROIT],m_bodies[JAMBE_DROIT],m_bodies[JAMBE_DROIT]->GetWorldCenter()+b2Vec2(0.0f,-0.15f));
		m_joints[CHEVILLE_DROIT] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
		m_currentAnglesLocal.push_back(0.0);
		m_currentAnglesGlobal.push_back(0.0);

		// GENOU GAUCHE
		jointDef.Initialize(m_bodies[JAMBE_GAUCHE],m_bodies[CUISSE_GAUCHE],m_bodies[JAMBE_GAUCHE]->GetWorldCenter()+b2Vec2(0.0f,0.15f));
		jointDef.lowerAngle = 0.0;
		m_joints[GENOU_GAUCHE] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
		m_currentAnglesLocal.push_back(0.0);
		m_currentAnglesGlobal.push_back(0.0);

		// GENOU DROIT
		jointDef.Initialize(m_bodies[JAMBE_DROIT],m_bodies[CUISSE_DROIT],m_bodies[JAMBE_DROIT]->GetWorldCenter()+b2Vec2(0.0f,0.15f));
		m_joints[GENOU_DROIT] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
		m_currentAnglesLocal.push_back(0.0);
		m_currentAnglesGlobal.push_back(0.0);
		jointDef.lowerAngle = -0.5f * b2_pi;

		// HANCHE GAUCHE
		jointDef.Initialize(m_bodies[CUISSE_GAUCHE],m_bodies[TRONC],m_bodies[CUISSE_GAUCHE]->GetWorldCenter()+b2Vec2(0.0f,0.15f));
		m_joints[HANCHE_GAUCHE] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
		m_currentAnglesLocal.push_back(0.0);
		m_currentAnglesGlobal.push_back(0.0);

		// HANCHE DROIT
		jointDef.Initialize(m_bodies[CUISSE_DROIT],m_bodies[TRONC],m_bodies[CUISSE_DROIT]->GetWorldCenter()+b2Vec2(0.0f,0.15f));
		m_joints[HANCHE_DROIT] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
		m_currentAnglesLocal.push_back(0.0);
		m_currentAnglesGlobal.push_back(0.0);

		m_currentAnglesLocal.push_back(0.0);
		m_currentAnglesGlobal.push_back(0.0); // pour l'angle du tronc

		// PD Controleurs
		// ==============

	    /*KpKvRatio = 0.83;
		float KpCheville = 2.32;
		float KpGenou = 4.43;
		float KpHanche = 9.95;
		float KpTronc = 11.89;*/



		//SumTroque currentcost
        /*KpKvRatio = 0.873399;
		float KpCheville = 2.62908;
		float KpGenou = 4.54618;
		float KpHanche = 11.7343;
		float KpTronc = 12.9415;*/


		// SumVelocity currentcost
		/*KpKvRatio =  0.92138;
		float KpCheville = 2.64174;
		float KpGenou = 4.52391;
		float KpHanche = 11.0567;
		float KpTronc = 12.4632;*/




		// Stability
		/*KpKvRatio =  4.89702  ;
		float KpCheville = 6.58204 ;
		float KpGenou = 18.4583 ;
		float KpHanche = 23.5742  ;
		float KpTronc = 1.11965   ;*/




		// All inclusive
		KpKvRatio =  0.898984 ;
		float KpCheville = 2.72343;
		float KpGenou = 5.53794 ;
		float KpHanche = 10.4363 ;
		float KpTronc = 15.3312 ;







        m_PDControllers[CHEVILLE_GAUCHE] = new PDController(KpCheville,KpKvRatio*sqrt(KpCheville));
        m_PDControllers[CHEVILLE_DROIT] = new PDController(KpCheville,KpKvRatio*sqrt(KpCheville));
        m_PDControllers[GENOU_GAUCHE] = new PDController(KpGenou,KpKvRatio*sqrt(KpGenou));
        m_PDControllers[GENOU_DROIT] = new PDController(KpGenou,KpKvRatio*sqrt(KpGenou));
        m_PDControllers[HANCHE_GAUCHE] = new PDController(KpHanche,KpKvRatio*sqrt(KpHanche));
        m_PDControllers[HANCHE_DROIT] = new PDController(KpHanche,KpKvRatio*sqrt(KpHanche));
        m_PDControllers[NB_ARTICULATIONS] = new PDController(KpTronc,KpKvRatio*sqrt(KpTronc)); // TRONC

        // Finite State Machine
        // ====================
        //m_stateMachine = new FSM_Stand();
        //m_stateMachine = new FSM_MoonWalker();
        //m_stateMachine = new FSM_Sad();


        //Angle limite
        //float seuil = 0.1;
        //m_stateMachine = new FSM_Walk();
        m_stateMachine = new FSM_Honte();

}

Biped::~Biped() { // Destructor
    for (int i = 0; i <= NB_ARTICULATIONS; ++i) {
        if (m_PDControllers[i]!=NULL) {delete m_PDControllers[i]; m_PDControllers[i] = NULL;}
    }
    delete m_stateMachine;
}

void Biped::update(double Dt) {
    // Remise � zero des moments articulaires
    for (int j = 0; j <= NB_ARTICULATIONS; ++j) m_motorTarget[j] = 0.0;

    // Mise � jour de la position et de la vitesse du COM
    computeCenterOfMass();

    if (!hasFallen()) { // Teste si le bip�de est tomb�
        // Mise � jour de l'�tat dans la machine si condition remplie
        m_stateMachine->update(Dt,m_currentAnglesLocal,m_currentAnglesGlobal);


        // Calcul des moments n�cessaires au suivi des poses cl�s
        KeyPoseTracking();

        // Application des moments
        for (int j = 0; j <= NB_ARTICULATIONS; ++j) m_bodies[j]->ApplyTorque(m_motorTarget[j],true);

        isblocked();
        saveme();
        Sador45();
        //changeState(4);
    }
}

bool Biped::hasFallen() {
    // vrai si d�ja � terre (impossible de se remettre debout)
	if (m_hasFallen) return m_hasFallen;

	// d�tection que le bip�de est tomb� : le CdM du tronc est au niveau des genoux
	m_hasFallen = m_bodies[TRONC]->GetWorldCenter().y < 0.4;

	return m_hasFallen;
}



void Biped::applyImpulse(const float x, const float y) {


    b2Vec2 jump(x, y);
    b2Vec2 point = getCOM();
    for (int i = 0; i < NB_CORPS; ++i) {
        m_bodies[i]->ApplyLinearImpulse(jump, point, true);
    }
}

// aplliquer force virtuelle sur m bodies
void Biped::applyVirtualForce(b2Vec2 force) {


    b2Vec2 point = getCOM();

    for (int i = 0; i < NB_CORPS; ++i) {


		m_bodies[i]->ApplyForce(force, point, true);
	}
}


void Biped::saveme() {

    float s = m_bodies[TRONC]->GetWorldCenter().y;

if ( s < 0.5)
{
    cout << "Saving impulse ! Distance = "<< s << endl;
    //delete m_stateMachine;

    //m_stateMachine = new FSM_Dance();

    applyImpulse(0.0f,0.2f);


    //changeFSM(new FSM_Moon());



}
}



// Activer FSM Dance si le bipède est bloqué
bool Biped::isblocked() {

    for (int i = 0; i < NB_CORPS; ++i) {
        b2ContactEdge* contact = m_bodies[i]->GetContactList();
        while (contact != NULL) {
            b2Body* otherBody = contact->other;
            if (reinterpret_cast<intptr_t>(otherBody->GetFixtureList()->GetUserData()) != 55 && reinterpret_cast<intptr_t>(otherBody->GetFixtureList()->GetUserData()) != 99) {

                if (m_velocityCOM.x < 0) {

                    //delete m_stateMachine;

                    //m_stateMachine = new FSM_Dance();
                    changeFSM(new FSM_Dance());



                     return true;


                }
            }
            contact = contact->next;
    }
}

return false;

}

string Biped::getFSMname() {

 return m_stateMachine->getName();

}








//changer la FSM
//void Biped::changeFSM(FSM* fsm) {
//	delete m_stateMachine;
//	m_stateMachine = fsm;
//}





void Biped::changeFSM(FSM* nextFSM) {


    if (!switcher) {
            //cout << switcher << endl;
        delete m_stateMachine;
        m_stateMachine = nextFSM;
    }

   else {

    //cout << switcher << endl;

        unsigned int bestState = 0;
        float minScore = std::numeric_limits<float>::max();

        // Parcourir tous les états possibles dans la nouvelle FSM
        for (const auto& state : nextFSM->m_states) {
            float score = 0.0f;

            // Calculer l'écart pour les angles
            for (size_t i = 0; i <= NB_ARTICULATIONS; ++i) {
                float currentAngle;
                // Utiliser l'angle local si c'est spécifié, sinon utiliser l'angle global
                if ((state.targetLocal[i]) && (i != NB_ARTICULATIONS)) {
                    currentAngle = -m_joints[i]->GetJointAngle();
                }
                else {
                    currentAngle = m_bodies[i]->GetTransform().q.GetAngle();
                }
                // Calculer la différence quadratique pour les angles
                score += pow(state.targetAngles[i] - currentAngle, 2)*(0.3*state.transitionTime);
                // / (0.5*state.transitionTime + 1);
            }

            // Ajouter le temps de transition comme facteur dans le score
            // Supposons que state.transitionTime représente le temps nécessaire pour passer à cet état
            //float totalScore = angleDifferenceScore + state.transitionTime;

            // Mettre à jour le meilleur état si le score total est inférieur
            if (score < minScore) {
                minScore = score;
                bestState = state.ID;
            }
        }

        // Changer à l'état avec le score minimal

        delete m_stateMachine;
        m_stateMachine = nextFSM;
        changeState(bestState);
    }
}























//changer toute fsm vers walk sauf walk changer en moonwalker
bool Biped::FSMofWalks(bool statut) {


    statut = (m_stateMachine->getName() == "Walk") ;
    if (statut) { changeFSM(new FSM_MoonWalker());
	}
    else {
		changeFSM(new FSM_Walk()) ;
	}
return statut;
}





// Changer l'état d'une FSM
void Biped::changeState(unsigned int id) {
    // Vérifie si le nouvel état est différent de l'état actuel
    if (m_stateMachine->m_currentState != id) {
        // Trouve l'indice de l'état dans le vecteur des états de la FSM
        auto it = find_if(m_stateMachine->m_states.begin(), m_stateMachine->m_states.end(),
                               [id](const State& state) { return state.ID == id; });
        if (it != m_stateMachine->m_states.end()) {
            // Si l'état est trouvé, change l'état courant de la FSM
            m_stateMachine->m_currentState = distance(m_stateMachine->m_states.begin(), it);
            // Réinitialise le temps écoulé dans l'état à 0
            m_stateMachine->m_timeInState = 0.0;
            // Optionnellement, réinitialiser d'autres variables si nécessaire
            // Exemple : m_stateMachine->resetStateVariables();
        }
    }
}







// Détecter les collisions avec la balle
bool Biped::ballbulled() {
    for (int i = 0; i < NB_CORPS; ++i) {
        b2ContactEdge* contact = m_bodies[i]->GetContactList();
        while (contact) {
            b2Body* otherBody = contact->other;
            int userData = reinterpret_cast<intptr_t>(otherBody->GetFixtureList()->GetUserData());

            if (userData == 55) {
                // Collision détectée avec une balle
                return true;
            }
            contact = contact->next;
        }
    }
    return false; // Aucune collision avec une balle
}


// Passer à l'état 8 de la FSM MoonWalker si une collision avec la balle a eu lieu
void Biped::Sador45() {
    if (ballbulled()){
        FSM_MoonWalker* mw = dynamic_cast<FSM_MoonWalker*>(m_stateMachine);
        if (mw && (m_stateMachine->getID() != 8)) {
            // La FSM active est FSM_Dance et une collision avec l'ID 55 a eu lieu
            changeState(8);
        }
        else if (!mw) {

            changeFSM(new FSM_Sad());
        }

    }

}


float Biped::sumTorque() const {
    // Retourne la somme des carr�s des moments articulaires appliqu�s (pour l'optimisation)
    float sum = 0;
    for (int j = 0; j <= NB_ARTICULATIONS; ++j) sum += m_motorTarget[j]*m_motorTarget[j];
    return sum;
}

float Biped::sumAngleVelocity() const {
    // Retourne la somme des vitesses angulaires absolues (pour l'optimisation)
    float sum = 0;
    for (int j = 0; j < NB_ARTICULATIONS; ++j) sum += fabs(m_joints[j]->GetJointSpeed());
    return sum;
}


// fonction d'évaluation de stabilité, évaluer la droite qui relis le centre de masse et le centre de masse du tronc à la normale du sol
float Biped::stabilityCost() const {

    b2Vec2 ynormal(0.0f, 1.0f); // Axe Y normal pour la comparaison

    // Calculer le vecteur du tronc au centre de masse (COM)
    b2Vec2 stability = m_positionCOM - m_bodies[TRONC]->GetWorldCenter();
    stability.Normalize(); // Normaliser pour obtenir uniquement la direction

    // Calculer le produit scalaire et ajuster sa plage de -1 à 1 à 0 à 1
    float cosinus = b2Dot(stability, ynormal);

    return (1-cosinus)*15;
}






//====================== PRIVATE ============================//

void Biped::computeCenterOfMass() {
    // Calcul de la position et de la vitesse du CdM du bipede dans le repere du monde
    float32 total_mass = 0.0f;
    m_velocityCOM = m_positionCOM;
    m_positionCOM.SetZero();
    for (int i = 0; i < NB_CORPS; ++i) {
        float32 massBody = m_bodies[i]->GetMass();
        b2Vec2 comBody = m_bodies[i]->GetWorldCenter();
        m_positionCOM += massBody * comBody;
        total_mass += massBody;
    }
    m_positionCOM = (1.0f / total_mass) * m_positionCOM;
    m_velocityCOM = m_positionCOM - m_velocityCOM;
}




//Effectuer une correction si l'angle est proche de la limite par moment
void Biped::angleLimit(int idx, float currentAngle) {
	float lowerLimit = -m_joints[idx]->GetUpperLimit();
	float upperLimit = -m_joints[idx]->GetLowerLimit();

	float Kp = m_PDControllers[idx]->getKpGain() ; // Gain proportionnel

	// Appliquer un moment correctif si l'angle est proche de la limite inférieure
	if (currentAngle < lowerLimit + seuil) {
		float correction = Kp * (lowerLimit + seuil - currentAngle);
		m_motorTarget[idx] += correction;
	}
	// Appliquer un moment correctif si l'angle est proche de la limite supérieure
	else if (currentAngle > upperLimit - seuil) {
		float correction = Kp * (currentAngle - (upperLimit - seuil));
		m_motorTarget[idx] -= correction;
	}
}


//Ajuste les moments pour nulliser la somme; principe n*éléments - n*moyenne = 0
void Biped::NoEnergy() {
    float momentos = 0.0f;
    // Calculer la somme des moments
    for (int j = 0; j <= NB_ARTICULATIONS; ++j) {
        momentos += m_motorTarget[j];
    }
    // Calculer la moyenne des moments
    float momento = momentos / (NB_ARTICULATIONS + 1);
    // Ajuster chaque moment pour que la somme soit nulle
    for (int j = 0; j <= NB_ARTICULATIONS; ++j) {
        m_motorTarget[j] -= momento;
    }
}





void Biped::KeyPoseTracking () {
    // R�cup�ration des cibles et de l'information local/global
    vector<float> targetAngles = m_stateMachine->getCurrentTargetAngles();
    vector<bool> targetLocal = m_stateMachine->getCurrentTargetLocal();
    float currentAngle, moment;

    // Pour toutes les articulations
    for (int j = 0; j <= NB_ARTICULATIONS; ++j) {
        // Lit la cible pour l'articulation j dans targetAngles
        float targetAngle = targetAngles[j];
        // Affecte la cible au r�gulateur PD par setTarget
        m_PDControllers[j]->setTarget(targetAngle);
        // Mise � jour de m_currentAnglesLocal par b2RevoluteJoint::GetJointAngle() (attention au signe et attention pour j==NB_ARTICULATIONS pas d'angle local)
        if (j!=NB_ARTICULATIONS) m_currentAnglesLocal[j] = -m_joints[j]->GetJointAngle();
        else m_currentAnglesLocal[j] = 0.0;
        // Mise � jour de m_currentAnglesGlobal par b2Body::GetTransform().q.GetAngle() avec l'�quivalence d'indice d'articulation et de corps rigide (cf. �num�rations)
        m_currentAnglesGlobal[j] = m_bodies[j]->GetTransform().q.GetAngle();
        //m_currentspeed[j] = m_joints[j]->GetJointSpeed();




        // Calcul du moment � ajouter dans m_motorTarget gr�ce au r�gulateur PD et en fonction de si la cible est locale ou globale
        if (targetLocal[j])
        {
            m_motorTarget[j] += m_PDControllers[j]->compute(m_currentAnglesLocal[j]);


            //Angle limite
            angleLimit(j, m_currentAnglesLocal[j]);
        }

        else m_motorTarget[j] += m_PDControllers[j]->compute(m_currentAnglesGlobal[j]);

    }

    // Activer le controlleur qui n'accepte pas l'énergie externe
    if (energy0) {
    NoEnergy();
    }
}


