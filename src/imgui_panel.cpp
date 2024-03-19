#include "imgui_panel.hpp"

namespace imgui_panel {
	// default values
	bool showPanel = true;
	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
	bool reset_view = false;
	int number_of_iterations_per_frame = 1;

    float gravity = 9.81f;

	//Simulation settings
	ModelType selected_model_type = ModelType::MassOnSpring;
	std::map<ModelType, const char*> type_to_name_map = {
		  {ModelType::MassOnSpring,  "Mass On Spring"}
		, {ModelType::ChainPendulum, "Chain Pendulum"}
		, {ModelType::CubeOfJelly,   "Cube Of Jelly"} // UNCOMMENT THIS WHEN IMPLEMENTED
		, {ModelType::HangingCloth,  "Hanging Cloth"} // UNCOMMENT THIS WHEN IMPLEMENTED
	};


	bool play_simulation = false;
	bool reset_simulation = false;
	bool step_simulation = false;
	float dt_simulation = 0.001f;

	std::function<void(void)> draw = [](void) {
		if (showPanel && ImGui::Begin("Panel", &showPanel, ImGuiWindowFlags_MenuBar)) {
			ImGui::Spacing();
			ImGui::Separator();

			ImGui::ColorEdit3("Clear color", (float*)&clear_color);
			reset_view = ImGui::Button("Reset View");
			ImGui::SliderInt("Iterations Per Frame", &number_of_iterations_per_frame, 1, 100);
            ImGui::SliderFloat("Gravity Acceleration", &gravity, 0.0f, 20.0f);

			ImGui::Spacing();
			ImGui::Separator();

			if (ImGui::BeginCombo("Model", type_to_name_map[selected_model_type])){
				for (const std::pair<ModelType, const char*> entry_pair : type_to_name_map){
					bool is_selected = (entry_pair.first == selected_model_type);
					if (ImGui::Selectable(entry_pair.second, is_selected))
						selected_model_type = entry_pair.first;
					if (is_selected)
						ImGui::SetItemDefaultFocus();
				}
				ImGui::EndCombo();
			}

			ImGui::Checkbox("Play Simulation", &play_simulation);
			reset_simulation = ImGui::Button("Reset Simulation");
			if (!play_simulation) {
				step_simulation = ImGui::Button("Step Simulation");
			}
			ImGui::DragFloat("Simulation dt", &dt_simulation, 1.e-5f, 1.e-5f, 1.f, "%.6e");

			ImGui::Spacing();
			ImGui::Separator();

			// Any simulation specific functions/IO
			switch (selected_model_type) {
			case ModelType::MassOnSpring: {
				// Maybe mass or spring constents (or gravity is funky)
			} break;
			case ModelType::ChainPendulum: {
				// ...
			} break;
			case ModelType::CubeOfJelly: {
				// ...
			} break;
			case ModelType::HangingCloth: {
				// ...
			} break;
			}

			ImGui::Spacing();
			ImGui::Separator();

			float frame_rate = ImGui::GetIO().Framerate;
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
				1000.0f / frame_rate, frame_rate);

			ImGui::Spacing();
			ImGui::Separator();

			ImGui::End();
		}
	};
} // namespace panel