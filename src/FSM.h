#ifndef FSM_H
#define FSM_H

#include <vector>
#include <string>

struct State {
    unsigned int ID;                    // Identificateur de l'�tat
    std::vector<float> targetAngles;    // Angle cl� pour chaque articulation
    std::vector<bool> targetLocal;      // Vrai si l'angle cl� est local au parent, faux sinon (global)
    unsigned int nextState;             // Identificateur de l'�tat suivant
    float transitionTime;               // La dur�e de transition (si bas� dur�e)

};

class FSM {

public:

	// Mise � jour de l'�tat si condition remplie
	void update(double Dt, std::vector<float> currentAnglesLocal, std::vector<float> currentAnglesGlobal);

	// Retourne les cibles et information local/global h_interpolation
	std::vector<float> getCurrentTargetAngles() const;
	std::vector<bool> getCurrentTargetLocal() const {return m_states[m_currentState].targetLocal;}

	// Retourne l'ID de l'�tat
	unsigned int getID() const {return m_states[m_currentState].ID;}
	virtual ~FSM();
	virtual std::string getName() const = 0;
	//void setv1(float v) {v1 = v;}


protected:

	FSM();      // Construit la machine avec ses �tats


public:
    unsigned int            m_nbStates;             // Le nombre d'�tats
    unsigned int            m_currentState;         // L'indice de l'�tat courant
    std::vector<State>      m_states;               // Les �tats
    double                  m_timeInState;          // Temps �coul� dans l'�tat
    std::vector<float>      m_anglesAtTransition;   // Les angles au moment de la derni�re transition


    std::vector<float>      m_anglespeed;           // Vitesse angulaire
    bool                    m_interpolation;          // faux pour linéaire, vrai pour hermitienne
                            //vitesse cible lors de l'interpolation hermetienne
    bool                    manualspeed=false;
    float v1;









};

class FSM_Stand : public FSM { // La machine � �tats finis pour un mouvement stable debout
public:
	FSM_Stand();
	virtual ~FSM_Stand();
	std::string getName() const override;
};

class FSM_Walk : public FSM { // La machine � �tats finis pour un mouvement de marche
public:
	FSM_Walk();
	virtual ~FSM_Walk();
	std::string getName() const override;
};







class FSM_Sad : public FSM { // La machine � �tats finis pour un mouvement stable debout
public:
	FSM_Sad();
	virtual ~FSM_Sad();
	std::string getName() const override;

};

class FSM_Honte : public FSM { // La machine � �tats finis pour un mouvement de marche
public:
	FSM_Honte();
	virtual ~FSM_Honte();
	std::string getName() const override;
};


class FSM_Jumper : public FSM { // La machine � �tats finis pour un mouvement de marche
public:
	FSM_Jumper();
	virtual ~FSM_Jumper();
	std::string getName() const override;
};



class FSM_Dance : public FSM { // La machine � �tats finis pour un mouvement de marche
public:
	FSM_Dance();
	virtual ~FSM_Dance();
	std::string getName() const override;
};


class FSM_Jump : public FSM {
public:
    FSM_Jump();
    virtual ~FSM_Jump();
    std::string getName() const override;

    void jump(float jumpHeight);
};




class
FSM_MoonWalker : public FSM {
public:

    FSM_MoonWalker();
    virtual ~FSM_MoonWalker();
    std::string getName() const override;

};



#endif
