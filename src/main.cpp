#include "givio.h"
#include "givr.h"

#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp> // lerp

#include <panel.h>
#include <picking_controls.h>
#include <turntable_controls.h>

#include "models.hpp"
#include "imgui_panel.hpp"

using namespace giv;
using namespace giv::io;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;

// program entry point
int main(void) {
	// initialize OpenGL and window
	GLFWContext glContext;
	glContext.glMajorVesion(3)
		.glMinorVesion(3)
		.glForwardComaptability(true)
		.glCoreProfile()
		.glAntiAliasingSamples(2)
		.matchPrimaryMonitorVideoMode();
	std::cout << glfwVersionString() << '\n';

	// setup window (OpenGL context)
	ImGuiWindow window = glContext.makeImGuiWindow(Properties()
		.size(dimensions{ 1000, 1000 })
		.title("Mass Spring Systems")
		.glslVersionString("#version 330 core"));
	// set our imgui update function
	panel::update_lambda_function = imgui_panel::draw;

	ViewContext view = View(TurnTable(), Perspective());
	TurnTableControls controls(window, view.camera);

	//  Custom Bind keys
	auto toggle_panel_routine = [&](auto event) {
		if (event.action == GLFW_PRESS)
			imgui_panel::showPanel = !imgui_panel::showPanel;
		};
	auto reset_view_routine = [&](auto event) {view.camera.reset(); };
	auto close_window_routine = [&](auto) { window.shouldClose(); };
	window.keyboardCommands()
		| Key(GLFW_KEY_P, toggle_panel_routine)
		| Key(GLFW_KEY_V, reset_view_routine)
		| Key(GLFW_KEY_ESCAPE, close_window_routine);

	// Models
	imgui_panel::ModelType model_type = imgui_panel::ModelType::MassOnSpring;
	std::unique_ptr<simulation::models::GenericModel> model
		= std::make_unique<simulation::models::MassOnSpringModel>();

	// main loop
	mainloop(std::move(window), [&](float /*dt - Time since last frame. You should start by using imgui_panel::dt and only use this under the "Free the Physics" time step scheme */) {
		// updates from panel
		if (imgui_panel::reset_view) {
			view.camera.reset();
		}

		// Change simulation model
		if (model_type != imgui_panel::selected_model_type) {
			model_type = imgui_panel::selected_model_type;
			imgui_panel::play_simulation = false; //For safety reasons, stop simulation
			switch (model_type) {
			case imgui_panel::ModelType::MassOnSpring: {
				model = std::make_unique<simulation::models::MassOnSpringModel>();
				imgui_panel::dt_simulation = 0.001f; //Good idea to hard-code a good dt for each simulation
			}break;
			case imgui_panel::ModelType::ChainPendulum: {
				model = std::make_unique<simulation::models::ChainPendulumModel>();
				imgui_panel::dt_simulation = 0.001f; //Good idea to hard-code a good dt for each simulation
			}break;
			case imgui_panel::ModelType::CubeOfJelly: {
				//TO-DO: Fill
                model = std::make_unique<simulation::models::CubeOfJellyModel>();
                imgui_panel::dt_simulation = 0.001f;
			}break;
			case imgui_panel::ModelType::HangingCloth: {
				//TO-DO: Fill
			}break;
			}
		}

		//Simulation updates
		if (imgui_panel::reset_simulation) {
			model->reset();
		}

		if (imgui_panel::step_simulation) {
			model->step(imgui_panel::dt_simulation);
		}

		if (imgui_panel::play_simulation) {
			for (size_t i = 0; i < imgui_panel::number_of_iterations_per_frame; i++) {
				model->step(imgui_panel::dt_simulation);
			}
		}

		// render
		auto color = imgui_panel::clear_color;
		glClearColor(color.x, color.y, color.z, color.z);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		view.projection.updateAspectRatio(window.width(), window.height());

		model->render(view);
		});

	return EXIT_SUCCESS;
}