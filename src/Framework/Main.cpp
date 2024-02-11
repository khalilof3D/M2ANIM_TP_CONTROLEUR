
#include <glew/glew.h>
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw_gl3.h"
#include "DebugDraw.h"

#include "Application.h"

#include "glfw/glfw3.h"
#include <stdio.h>
#include <fstream>
#include <iostream>

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

#ifdef _WIN32
#ifdef _MSC_VER
#include <crtdbg.h>
#else
#define _ASSERT(expr) ((void)0)
#define _ASSERTE(expr) ((void)0)
#endif
#endif
using namespace std;

struct UIState {bool showMenu;};

namespace {
	GLFWwindow* mainWindow = NULL;
	UIState ui;
	Application* app;
	Settings settings;
	bool rightMouseDown;
	b2Vec2 lastp;
}

static void sCreateUI(GLFWwindow* window) {
	ui.showMenu = true;
	// Init UI
	const char* fontPath = "src/Data/DroidSans.ttf";
	ImGui::GetIO().Fonts->AddFontFromFileTTF(fontPath, 20.f);
	if (ImGui_ImplGlfwGL3_Init(window, false) == false)	{fprintf(stderr, "Could not init GUI renderer.\n");assert(false);return;}

	ImGuiStyle& style = ImGui::GetStyle();
	style.FrameRounding = style.GrabRounding = style.ScrollbarRounding = 2.0f;
	style.FramePadding = ImVec2(4, 2);
	style.DisplayWindowPadding = ImVec2(0, 0);
	style.DisplaySafeAreaPadding = ImVec2(0, 0);
}

static void sResizeWindow(GLFWwindow*, int width, int height) {
	g_camera.m_width = width;
	g_camera.m_height = height;
}

static void sRestart() {
	delete app;
	app = new Application;
}

static void sKeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	ImGui_ImplGlfwGL3_KeyCallback(window, key, scancode, action, mods);
	bool keys_for_ui = ImGui::GetIO().WantCaptureKeyboard;
	if (keys_for_ui) return;

	if (action == GLFW_PRESS) {
		switch (key) {
		case GLFW_KEY_ESCAPE: // Quit
			glfwSetWindowShouldClose(mainWindow, GL_TRUE);
			break;

		case GLFW_KEY_LEFT: // Pan left
			if (mods == GLFW_MOD_CONTROL) {
				b2Vec2 newOrigin(2.0f, 0.0f);
				app->ShiftOrigin(newOrigin);
			}
			else {g_camera.m_center.x -= 0.5f;}
			break;

		case GLFW_KEY_RIGHT: // Pan right
			if (mods == GLFW_MOD_CONTROL) {
				b2Vec2 newOrigin(-2.0f, 0.0f);
				app->ShiftOrigin(newOrigin);
			}
			else {g_camera.m_center.x += 0.5f;}
			break;

		case GLFW_KEY_DOWN: // Pan down
			if (mods == GLFW_MOD_CONTROL) {
				b2Vec2 newOrigin(0.0f, 2.0f);
				app->ShiftOrigin(newOrigin);
			}
			else {g_camera.m_center.y -= 0.5f;}
			break;

		case GLFW_KEY_UP: // Pan up
			if (mods == GLFW_MOD_CONTROL) {
				b2Vec2 newOrigin(0.0f, -2.0f);
				app->ShiftOrigin(newOrigin);
			}
			else {g_camera.m_center.y += 0.5f;}
			break;

		case GLFW_KEY_HOME: // Reset view
			g_camera.m_zoom = 1.0f;
			g_camera.m_center.Set(0.0f, 20.0f);
			break;

		case GLFW_KEY_PAGE_DOWN: // Zoom out
			g_camera.m_zoom = b2Min(1.1f * g_camera.m_zoom, 20.0f);
			break;

		case GLFW_KEY_PAGE_UP: // Zoom in
			g_camera.m_zoom = b2Max(0.9f * g_camera.m_zoom, 0.02f);
			break;

		case GLFW_KEY_R: // Reset app
			sRestart();
			break;

		case GLFW_KEY_SPACE: // Launch a ball
			if (app) {app->LaunchBall();}
			break;

        /*
		case GLFW_KEY_P: // Pause
			settings.pause = !settings.pause;
			break;
        */

		case GLFW_KEY_TAB: // Show menu
			ui.showMenu = !ui.showMenu;

		default:
			if (app) {app->Keyboard(key);}
		}
	}
	else if (action == GLFW_RELEASE) {app->KeyboardUp(key);}
}

static void sCharCallback(GLFWwindow* window, unsigned int c) {
	ImGui_ImplGlfwGL3_CharCallback(window, c);
}

static void sMouseButton(GLFWwindow* window, int32 button, int32 action, int32 mods) {
	ImGui_ImplGlfwGL3_MouseButtonCallback(window, button, action, mods);

	double xd, yd;
	glfwGetCursorPos(mainWindow, &xd, &yd);
	b2Vec2 ps((float32)xd, (float32)yd);

	// Use the mouse to move things around.
	if (button == GLFW_MOUSE_BUTTON_1) {
		b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);
		if (action == GLFW_PRESS) {
			app->MouseDown(pw);
		}
		if (action == GLFW_RELEASE) {app->MouseUp(pw);}
	}
	else if (button == GLFW_MOUSE_BUTTON_2)	{
		if (action == GLFW_PRESS) {
			lastp = g_camera.ConvertScreenToWorld(ps);
			rightMouseDown = true;
		}
		if (action == GLFW_RELEASE) {rightMouseDown = false;}
	}
}

static void sMouseMotion(GLFWwindow*, double xd, double yd) {
	b2Vec2 ps((float)xd, (float)yd);
	b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);
	app->MouseMove(pw);
	if (rightMouseDown)	{
		b2Vec2 diff = pw - lastp;
		g_camera.m_center.x -= diff.x;
		g_camera.m_center.y -= diff.y;
		lastp = g_camera.ConvertScreenToWorld(ps);
	}
}

static void sScrollCallback(GLFWwindow* window, double dx, double dy) {
	ImGui_ImplGlfwGL3_ScrollCallback(window, dx, dy);
	bool mouse_for_ui = ImGui::GetIO().WantCaptureMouse;
	if (!mouse_for_ui) {
		if (dy > 0) {g_camera.m_zoom /= 1.1f;}
		else {g_camera.m_zoom *= 1.1f;}
	}
}

static void sSimulate() {
	glEnable(GL_DEPTH_TEST);
	app->Step(&settings);
	app->DrawTitle("Controles : Esc=quitter, haut/bas=zoom, R=reset, space=balle");
	glDisable(GL_DEPTH_TEST);

}

static void sInterface() {
	int menuWidth = 300;
	if (ui.showMenu) {
		ImGui::SetNextWindowPos(ImVec2((float)g_camera.m_width - menuWidth - 10, 10));
		ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)g_camera.m_height - 20));
		ImGui::Begin("Proprietes de l'application", &ui.showMenu, ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoCollapse);
		ImGui::PushAllowKeyboardFocus(false); // Disable TAB

		ImGui::Text("Scene");
		ImGui::Separator();
		ImGui::Checkbox("Balles", &settings.launchBalls);
		ImGui::Checkbox("Marches", &settings.steps);

		ImGui::Text("Visualisation");
		ImGui::Separator();
		ImGui::Checkbox("Centre des masses", &settings.showCOM);
		ImGui::Checkbox("Articulations", &settings.drawJoints);
		ImGui::Checkbox("Points de contact", &settings.drawContactPoints);
		ImGui::Checkbox("Normales de contact", &settings.drawContactNormals);


		ImGui::Checkbox("Force virtuelle", &settings.virtualforce);


		if (settings.virtualforce) {
    ImGui::SliderFloat("Force X", &settings.forceval.x, -1.0f, 1.0f, "%.3f");
    ImGui::SliderFloat("Force Y", &settings.forceval.y, -1.0f, 1.0f, "%.3f");
    app->Virtualforce(settings.forceval);
}

if (ImGui::Button(settings.h_interpolation ? "Mode d'interpolation : Lineaire" : "Mode d'interpolation : Hermitien")) {
    // Bascule l'état à chaque clic
    settings.h_interpolation = !settings.h_interpolation;

    // Si nous passons à l'interpolation linéaire, réinitialiser
    if (!settings.h_interpolation) {
        settings.v1speed = false; // Réinitialiser le checkbox
    }

    // Mise à jour de l'état d'interpolation dans votre application
    app->interpolation(settings.h_interpolation);
}


// Si l'interpolation Hermite est sélectionnée, afficher les options pour la vitesse angulaire manuelle
if (!settings.h_interpolation) {
    ImGui::Checkbox("Vitesse angulaire manuelle", &settings.v1speed);

    if (settings.v1speed) {
        ImGui::SliderFloat("Vitesse angulaire cible", &settings.newv1, -2.0f, 2.0f, "%.3f");
        if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Definir la vitesse angulaire cible à chaque pas de temps");
        app->setv1(settings.newv1);}
        else {

        settings.newv1 = 0.0f; // Réinitialisation du slider
        }

}
// Appeler checkv1 dans tous les cas pour s'assurer que la FSM est correctement mise à jour
app->checkv1(settings.v1speed);




    ImGui::Checkbox("Interdiction d'energie externe", &settings.zenergy);
    if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Conception d un contrôleur interdisant l ajout d énergie externe (somme des moments articulaires qui doit être nulle)");
    app->zeroenergy(settings.zenergy);


//if (settings.walk ? "Switch to MoonWalk" : "Switch to Walk") {
  //  settings.walk = app->toWalk(bool w)
    //


ImGui::Text("Changer de FSM");
if (ImGui::Button(app->FSMname() == "Walk" ? "Switch MoonWalk" : "Switch to Walk")) {
    app->Walk();
}



ImGui::SliderFloat("Angle limite :", &settings.seuil, 0.0f, 180.0f, "%.3f");
if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Definir le seuil d'angle limite pour la contrainte articulaire.");
app->AngleLimite(settings.seuil);



ImGui::Checkbox("Changement de FSM optimisé", &settings.fsmchange);
app->fsmchange(settings.fsmchange);




//ImGui::Checkbox("Normales de contact", &settings.drawContactNormals);
//optimization
ImGui::Checkbox("Mode optimisation", &settings.optimization);


		ImVec2 button_sz = ImVec2(-1, 0);
		/*if (ImGui::Button("Pause (P)", button_sz)) settings.pause = !settings.pause;*/
		if (ImGui::Button("Remise a zero (R)", button_sz)) sRestart();
		if (ImGui::Button("Quitter (Esc)", button_sz)) glfwSetWindowShouldClose(mainWindow, GL_TRUE);

		ImGui::PopAllowKeyboardFocus();
		ImGui::End();
	}
}

void glfwErrorCallback(int error, const char *description) {
	fprintf(stderr, "GLFW error occured. Code: %d. Description: %s\n", error, description);
}

struct OptimizationData {
    int nbIter = 0;
    float bestCost = 100000;
    float currentCost = 0;

    static const int nbParam = 5;

    float currentParam [nbParam];
    float bestParam [nbParam];
    std::string filenameBestParam = "optimization.txt";
};

int main(int, char**) {

    // Init GLFW
	glfwSetErrorCallback(glfwErrorCallback);
	if (glfwInit() == 0) {fprintf(stderr, "Failed to initialize GLFW\n");return -1;}

    // Shader version on Linux
#ifndef _WIN32
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif // _WIN32

	// Window
	char title[64];
	sprintf(title, "M2ID3D-INF2321M TP CONTROLE DE BIPEDE");
	mainWindow = glfwCreateWindow(g_camera.m_width, g_camera.m_height, title, NULL, NULL);
	if (mainWindow == NULL) {fprintf(stderr, "Failed to open GLFW mainWindow.\n");glfwTerminate();return -1;}

	// Callbacks
	glfwMakeContextCurrent(mainWindow);
	glfwSetScrollCallback(mainWindow, sScrollCallback);
	glfwSetWindowSizeCallback(mainWindow, sResizeWindow);
	glfwSetKeyCallback(mainWindow, sKeyCallback);
	glfwSetCharCallback(mainWindow, sCharCallback);
	glfwSetMouseButtonCallback(mainWindow, sMouseButton);
	glfwSetCursorPosCallback(mainWindow, sMouseMotion);
	glfwSetScrollCallback(mainWindow, sScrollCallback);

	// Init GLEW
	GLenum err = glewInit();
	if (GLEW_OK != err)	{fprintf(stderr, "Error: %s\n", glewGetErrorString(err));exit(EXIT_FAILURE);}
	g_debugDraw.Create();
	sCreateUI(mainWindow);

	// Create application
	app = new Application;

	// Optimization data
    OptimizationData m_optiData;
    if (settings.optimization) {
        std::ifstream loadFile (m_optiData.filenameBestParam);
        if (loadFile.is_open()) {
                loadFile >> m_optiData.nbIter >> m_optiData.bestCost;
                for (int i=0; i<m_optiData.nbParam; i++) {
                        loadFile >> m_optiData.bestParam[i];
                        m_optiData.currentParam[i] = m_optiData.bestParam[i];
                }
                loadFile.close();
                app->setOptimizationData(m_optiData.bestParam);
        }
        else {
            app->getOptimizationData(m_optiData.currentParam);
            for (int i=0; i<m_optiData.nbParam; i++) {
                    m_optiData.bestParam[i] = m_optiData.currentParam[i];
            }
        }
    }

	// Control the frame rate. One draw per monitor refresh.
	glfwSwapInterval(1);
	glClearColor(0.0f, 0.0f, 0.0f, 1.f);

	double startTime = glfwGetTime();

	while (!glfwWindowShouldClose(mainWindow)) {
        // window
		glfwGetWindowSize(mainWindow, &g_camera.m_width, &g_camera.m_height);
        int bufferWidth, bufferHeight;
        glfwGetFramebufferSize(mainWindow, &bufferWidth, &bufferHeight);
        glViewport(0, 0, bufferWidth, bufferHeight);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // GUI
		ImGui_ImplGlfwGL3_NewFrame();
		ImGui::SetNextWindowPos(ImVec2(0,0));
		ImGui::SetNextWindowSize(ImVec2((float)g_camera.m_width, (float)g_camera.m_height));
		ImGui::Begin("Overlay", NULL, ImVec2(0,0), 0.0f, ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoInputs|ImGuiWindowFlags_AlwaysAutoResize|ImGuiWindowFlags_NoScrollbar);
		ImGui::SetCursorPos(ImVec2(5, (float)g_camera.m_height - 20));
		ImGui::End();

        // Run time step
		sSimulate();
		sInterface();

		// Render and poll events
		ImGui::Render();
		glfwSwapBuffers(mainWindow);
		glfwPollEvents();

		// Optimisation
		if (settings.optimization) {
                // Récupération du temps écoulé
                double currentTime = glfwGetTime();
                // Si on a dépassé le temps imparti pour une simulation
                if (currentTime - startTime >= settings.optiDuration) {
                        // Incrément du nombre d'itération
                        m_optiData.nbIter++;
                        // Arrêt de l'optimisation si limite d'itérations atteinte
                        if (m_optiData.nbIter==10000) break;
                        // Calcul du cout de la simulation
                        // Erreur due a la distance parcourue
                        float errorDistance = 10.0 - app->BipedPosition();
                        // Erreur cumulée de la simulation (ex. moments ou vitesses articulaires)
                        float errorCumulated = m_optiData.currentCost / 3000.0;
                        // Somme des erreurs pondérées
                        float totalError = errorDistance + errorCumulated;
                        // Si le bipède n'est pas tombé et que le coût est meilleur
                        if (!app->BipedHasFallen() && totalError < m_optiData.bestCost) {
                            // Le nouveau meilleur cout est le courant
                            m_optiData.bestCost = totalError;
                            // Recopie des paramètres courants dans les meilleurs paramètres
                            for (int i=0; i<m_optiData.nbParam; i++) m_optiData.bestParam[i] = m_optiData.currentParam[i];
                        }
                        // Remise à zéro du cout
                        m_optiData.currentCost = 0;
                        // Sauvegarde de la meilleure solution dans le fichier
                        std::ofstream saveFile (m_optiData.filenameBestParam);
                        saveFile << m_optiData.nbIter << " " << m_optiData.bestCost << " ";
                        for (int i=0; i<m_optiData.nbParam; i++) saveFile << m_optiData.bestParam[i] << " ";
                        saveFile << std::endl;
                        saveFile.close();

                        // Reinitialisation de la simulation
                        sRestart();

                        // Modification des meilleurs paramètres
                        for (int i=0; i<m_optiData.nbParam; i++) {
                                // Signe aléatoire (ie. incrément ou décrément)
                                int signe = (rand() % 2 == 0) ? -1 : 1;
                                // Valeur max de l'incrément (diminue avec les itérations)
                                float maxrand = exp(-m_optiData.nbIter*0.001);
                                // Incrément aléatoire
                                //float delta = RandomFloat(0.0,fabs(maxrand*m_optiData.bestParam[i]*0.25));
                                float delta = signe * RandomFloat(0.0f, exp(-static_cast<float>(m_optiData.nbIter) * 0.001f) * 0.25f * m_optiData.bestParam[i]);
                                // Application de l'incrément
                                m_optiData.currentParam[i] = m_optiData.bestParam[i] + delta*signe;
                        }
                        // Application des modifications sur les paramètres de l'optimisation
                        app->setOptimizationData(m_optiData.currentParam);
                        // Remize à zéro du temps
                        startTime = currentTime;
                }
                else {
                    // Si on a pas dépassé le temps imparti, on met à jour le cout cumulé
                    m_optiData.currentCost += app->getCurrentCost();
                }
		}

	}

    // Delete
	if (app) {
		delete app;
		app = NULL;
	}
	g_debugDraw.Destroy();
	ImGui_ImplGlfwGL3_Shutdown();
	glfwTerminate();

	return 0;
}
