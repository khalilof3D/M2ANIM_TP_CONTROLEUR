#ifndef BIPED_H
#define BIPED_H

#include "Box2D/Box2D.h"
#include "PDController.h"
#include "FSM.h"

enum {PIED_GAUCHE,PIED_DROIT,JAMBE_GAUCHE,JAMBE_DROIT,CUISSE_GAUCHE,CUISSE_DROIT,TRONC,NB_CORPS};               // Les corps rigides
enum {CHEVILLE_GAUCHE,CHEVILLE_DROIT,GENOU_GAUCHE,GENOU_DROIT,HANCHE_GAUCHE,HANCHE_DROIT,NB_ARTICULATIONS};     // Les articulations

class Biped {

public:

	Biped (b2World* world);                     // Constructeur
	virtual	~Biped();                           // Destructeur

	void update(double Dt);                     // Mise à jour du bipede (controleur)

	bool hasFallen();                                               // Vrai si le bipede est tombé


	bool isblocked(); //Le bipede est bloqué dans une collision
	bool energy0=false;

	void applyImpulse(const float x, const float y);
	void applyVirtualForce(b2Vec2 force);
	void changeState(unsigned int newState);
	void changeFSM(FSM* fsm);
	bool FSMofWalks(bool statut);
	std::string getFSMname();

	float seuil = 0.1;

	// Proportionalité entre Kp et Kv
	float KpKvRatio = 0.83;

	bool switcher = false;




	b2Vec2 getCOM() const {return m_positionCOM;}                   // Retourne la position du CdM
	FSM* getStateMachine(){return m_stateMachine;}                  // Retourne la machine à états finis des poses clés
	PDController ** getPDControllers (){return m_PDControllers;}    // Retourne le tableau des régulateurs PD



	float sumTorque() const;                                        // Retourne la somme des carrés des moments articulaires appliqués (pour l'optimisation)
	float sumAngleVelocity() const;                                 // Retourne la somme des vitesses angulaires absolues (pour l'optimisation)
	float stabilityCost() const;

protected:

    b2World             *	m_world;                            // Le monde physique
	bool	                m_hasFallen;                        // Vrai si le bipede est tombe
	b2Vec2                  m_positionCOM;                      // La position du CdM du bipede
	b2Vec2                  m_velocityCOM;                      // La vitesse du CdM du bipede
	b2Body			    *	m_bodies[NB_CORPS];                 // Le tableau de corps rigides
	b2RevoluteJoint     *	m_joints[NB_ARTICULATIONS];		    // Le tableau d'articulations
	std::vector<float>      m_currentAnglesLocal;               // Les angles courants des articulations (locaux)
	std::vector<float>      m_currentAnglesGlobal;              // Les orientations courantes des corps rigides (globaux)
	float                   m_motorTarget[NB_ARTICULATIONS+1];  // Le tableau de moments articulaires à appliquer (+1 pour tronc)
	PDController		*	m_PDControllers[NB_ARTICULATIONS+1];// Le tableau des regulateurs PD (un par articulation +1 pour tronc)
	FSM                 *   m_stateMachine;                     // La machine à états finis des poses clés

    void computeCenterOfMass();                         // Calcul de la position et de la vitesse du CdM du bipede dans le repere du monde
    void KeyPoseTracking ();                            // Calcul des moments nécessaires au suivi des poses clés



    void saveme();  //Si le centre du tronc est sûr le point d'atteindre 0.4 cette fonction génére une petite impulsation verticale
    bool ballbulled(); // Si le bipede est touché par une balle
    void Sador45(); // change d'état uniquement pour la FSM moon walker

    void angleLimit(int idx, float currentAngle);
    void NoEnergy();


};

#endif
