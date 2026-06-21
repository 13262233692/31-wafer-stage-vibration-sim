extends Node3D

var sim: ReticleStageSim
var stage_visual: MeshInstance3D
var target_marker: MeshInstance3D
var coil_visuals: Array[MeshInstance3D] = []
var camera: Camera3D
var light: DirectionalLight3D
var env: WorldEnvironment

var oscilloscope: Control
var status_panel: Panel
var control_panel: Panel

var phase_label: Label
var time_label: Label
var error_label: Label
var position_label: Label
var velocity_label: Label
var settling_label: Label
var lorentz_label: Label

var start_btn: Button
var reset_btn: Button

var target_pos_spin: SpinBox
var accel_g_spin: SpinBox
var active_stiffness_spin: SpinBox
var active_damping_spin: SpinBox
var threshold_spin: SpinBox
var time_scale_spin: SpinBox

var stage_base_z: float = 0.0
var visual_scale: float = 100.0

func _ready() -> void:
	_setup_environment()
	_setup_lighting()
	_setup_stage_visual()
	_setup_coil_visuals()
	_setup_target_marker()
	_setup_camera()
	_setup_simulation()
	_setup_ui()
	
	sim.start_simulation()

func _setup_environment() -> void:
	env = WorldEnvironment.new()
	var sky = Sky.new()
	sky.set_radiance_size(Sky.RADIANCE_SIZE_256)
	var env_res = Environment.new()
	env_res.set_background_mode(Environment.BG_CLEAR_COLOR)
	env_res.set_ambient_light_source(Environment.AMBIENT_SOURCE_SKY)
	env_res.set_ambient_light_color(Color(0.02, 0.02, 0.05))
	env_res.set_ambient_light_energy(0.3)
	env_res.set_fog_enabled(true)
	env_res.set_fog_light_color(Color(0.0, 0.0, 0.02))
	env_res.set_fog_depth_begin(50.0)
	env_res.set_fog_depth_end(200.0)
	env.set_environment(env_res)
	add_child(env)

func _setup_lighting() -> void:
	light = DirectionalLight3D.new()
	light.set_rotation_degrees(Vector3(-45, 30, 0))
	light.set_light_color(Color(0.9, 0.95, 1.0))
	light.set_light_energy(0.8)
	light.set_shadow(true)
	light.set_shadow_opacity(0.3)
	add_child(light)
	
	var fill_light = OmniLight3D.new()
	fill_light.set_position(Vector3(5, 10, 5))
	fill_light.set_light_color(Color(0.3, 0.4, 0.8))
	fill_light.set_light_energy(0.5)
	fill_light.set_omni_range(50.0)
	add_child(fill_light)

func _setup_stage_visual() -> void:
	stage_visual = MeshInstance3D.new()
	var stage_mesh = BoxMesh.new()
	stage_mesh.set_size(Vector3(4.0, 0.15, 3.0))
	stage_visual.set_mesh(stage_mesh)
	
	var mat = StandardMaterial3D.new()
	mat.set_albedo(Color(0.15, 0.15, 0.2))
	mat.set_metallic(0.95)
	mat.set_roughness(0.1)
	mat.set_emission(Color(0.05, 0.08, 0.15))
	mat.set_emission_energy(0.3)
	stage_visual.set_surface_override_material(0, mat)
	stage_visual.set_position(Vector3(0, 0, 0))
	add_child(stage_visual)
	
	var edge_mesh = BoxMesh.new()
	edge_mesh.set_size(Vector3(4.05, 0.02, 3.05))
	var edge = MeshInstance3D.new()
	edge.set_mesh(edge_mesh)
	var edge_mat = StandardMaterial3D.new()
	edge_mat.set_albedo(Color(0.1, 0.2, 0.4))
	edge_mat.set_emission(Color(0.1, 0.3, 0.8))
	edge_mat.set_emission_energy(2.0)
	edge_mat.set_transparency(StandardMaterial3D.TRANSPARENCY_ALPHA)
	edge_mat.set_albedo_color(Color(0.1, 0.2, 0.4, 0.5))
	edge.set_surface_override_material(0, edge_mat)
	edge.set_position(Vector3(0, 0.085, 0))
	stage_visual.add_child(edge)
	
	var grid_mat = StandardMaterial3D.new()
	grid_mat.set_albedo(Color(0.2, 0.3, 0.5))
	grid_mat.set_emission(Color(0.15, 0.25, 0.6))
	grid_mat.set_emission_energy(1.0)
	
	for i in range(-1, 2):
		var line_h = BoxMesh.new()
		line_h.set_size(Vector3(3.8, 0.005, 0.005))
		var line_h_inst = MeshInstance3D.new()
		line_h_inst.set_mesh(line_h)
		line_h_inst.set_surface_override_material(0, grid_mat)
		line_h_inst.set_position(Vector3(0, 0.076, i * 1.0))
		stage_visual.add_child(line_h_inst)
		
		var line_v = BoxMesh.new()
		line_v.set_size(Vector3(0.005, 0.005, 2.8))
		var line_v_inst = MeshInstance3D.new()
		line_v_inst.set_mesh(line_v)
		line_v_inst.set_surface_override_material(0, grid_mat)
		line_v_inst.set_position(Vector3(i * 1.0, 0.076, 0))
		stage_visual.add_child(line_v_inst)

func _setup_coil_visuals() -> void:
	var coil_mat = StandardMaterial3D.new()
	coil_mat.set_albedo(Color(0.0, 0.4, 0.8))
	coil_mat.set_emission(Color(0.0, 0.5, 1.0))
	coil_mat.set_emission_energy(3.0)
	
	var positions = [
		Vector3(-1.5, -0.2, -1.0),
		Vector3(-1.5, -0.2, 0.0),
		Vector3(-1.5, -0.2, 1.0),
		Vector3(1.5, -0.2, -1.0),
		Vector3(1.5, -0.2, 0.0),
		Vector3(1.5, -0.2, 1.0),
		Vector3(0.0, -0.2, -1.0),
		Vector3(0.0, -0.2, 1.0),
	]
	
	for pos in positions:
		var coil = MeshInstance3D.new()
		var coil_mesh = CylinderMesh.new()
		coil_mesh.set_top_radius(0.15)
		coil_mesh.set_bottom_radius(0.15)
		coil_mesh.set_height(0.05)
		coil.set_mesh(coil_mesh)
		coil.set_surface_override_material(0, coil_mat)
		coil.set_position(pos)
		stage_visual.add_child(coil)
		coil_visuals.append(coil)

func _setup_target_marker() -> void:
	target_marker = MeshInstance3D.new()
	var marker_mesh = BoxMesh.new()
	marker_mesh.set_size(Vector3(0.01, 0.5, 3.5))
	target_marker.set_mesh(marker_mesh)
	var marker_mat = StandardMaterial3D.new()
	marker_mat.set_albedo(Color(1.0, 0.1, 0.1))
	marker_mat.set_emission(Color(1.0, 0.1, 0.1))
	marker_mat.set_emission_energy(2.0)
	marker_mat.set_transparency(StandardMaterial3D.TRANSPARENCY_ALPHA)
	marker_mat.set_albedo_color(Color(1.0, 0.1, 0.1, 0.4))
	target_marker.set_surface_override_material(0, marker_mat)
	target_marker.set_position(Vector3(0, 0.3, 0))
	add_child(target_marker)

func _setup_camera() -> void:
	camera = Camera3D.new()
	camera.set_position(Vector3(0, 8, 12))
	camera.set_rotation_degrees(Vector3(-30, 0, 0))
	camera.set_fov(50.0)
	camera.set_near(0.01)
	camera.set_far(500.0)
	add_child(camera)
	camera.make_current()

func _setup_simulation() -> void:
	sim = ReticleStageSim.new()
	sim.set_name("ReticleStageSim")
	sim.set_mass(50.0)
	sim.set_lorentz_force_constant(50.0)
	sim.set_spring_stiffness(0.0)
	sim.set_passive_damping(2000.0)
	sim.set_active_stiffness(1.5e7)
	sim.set_active_damping(60000.0)
	sim.set_active_integral_gain(5.0e6)
	sim.set_target_position(0.01)
	sim.set_accel_g(2.0)
	sim.set_simulation_frequency(100000.0)
	sim.set_settling_threshold_nm(0.1)
	sim.set_time_scale(1.0)
	sim.set_max_sim_time(0.15)
	add_child(sim)

func _setup_ui() -> void:
	var canvas = CanvasLayer.new()
	canvas.set_layer(10)
	add_child(canvas)
	
	var root_panel = PanelContainer.new()
	root_panel.set_anchors_preset(Control.PRESET_FULL_RECT)
	canvas.add_child(root_panel)
	
	var h_split = HSplitContainer.new()
	h_split.set_anchors_preset(Control.PRESET_FULL_RECT)
	root_panel.add_child(h_split)
	
	var left_spacer = ColorRect.new()
	left_spacer.set_custom_minimum_size(Vector2(0, 0))
	left_spacer.set_color(Color(0, 0, 0, 0))
	h_split.add_child(left_spacer)
	
	var right_panel = VBoxContainer.new()
	right_panel.set_custom_minimum_size(Vector2(480, 0))
	h_split.add_child(right_panel)
	
	_setup_oscilloscope(right_panel)
	_setup_status_panel(right_panel)
	_setup_control_panel(right_panel)

func _setup_oscilloscope(parent: VBoxContainer) -> void:
	var scope_container = PanelContainer.new()
	scope_container.set_custom_minimum_size(Vector2(460, 280))
	var scope_style = StyleBoxFlat.new()
	scope_style.set_bg_color(Color(0.02, 0.02, 0.05, 0.92))
	scope_style.set_border_color(Color(0.1, 0.3, 0.6))
	scope_style.set_border_width_all(2)
	scope_container.add_theme_stylebox_override("panel", scope_style)
	parent.add_child(scope_container)
	
	oscilloscope = Control.new()
	oscilloscope.set_name("Oscilloscope")
	oscilloscope.set_custom_minimum_size(Vector2(460, 280))
	oscilloscope.set_script(load("res://scripts/oscilloscope_panel.gd"))
	scope_container.add_child(oscilloscope)

func _setup_status_panel(parent: VBoxContainer) -> void:
	var status_container = PanelContainer.new()
	var status_style = StyleBoxFlat.new()
	status_style.set_bg_color(Color(0.02, 0.02, 0.08, 0.92))
	status_style.set_border_color(Color(0.1, 0.2, 0.4))
	status_style.set_border_width_all(1)
	status_container.add_theme_stylebox_override("panel", status_style)
	parent.add_child(status_container)
	
	var vbox = VBoxContainer.new()
	vbox.add_theme_constant_override("separation", 2)
	status_container.add_child(vbox)
	
	var header = Label.new()
	header.set_text("─── RETICLE STAGE STATUS ───")
	header.add_theme_color_override("font_color", Color(0.3, 0.7, 1.0))
	header.set_horizontal_alignment(HORIZONTAL_ALIGNMENT_CENTER)
	vbox.add_child(header)
	
	phase_label = _add_status_row(vbox, "PHASE", "IDLE", Color(0.8, 0.8, 0.8))
	time_label = _add_status_row(vbox, "SIM TIME", "0.000 ms", Color(0.8, 0.8, 0.8))
	position_label = _add_status_row(vbox, "POSITION", "0.000000 mm", Color(0.5, 1.0, 0.5))
	velocity_label = _add_status_row(vbox, "VELOCITY", "0.000000 m/s", Color(0.5, 1.0, 0.5))
	error_label = _add_status_row(vbox, "ERROR", "0.000 nm", Color(1.0, 0.5, 0.5))
	settling_label = _add_status_row(vbox, "SETTLING", "---", Color(1.0, 0.8, 0.2))
	lorentz_label = _add_status_row(vbox, "LORENTZ F", "0.000 N", Color(0.3, 0.8, 1.0))

func _add_status_row(parent: VBoxContainer, title: String, default: String, color: Color) -> Label:
	var hbox = HBoxContainer.new()
	parent.add_child(hbox)
	
	var title_lbl = Label.new()
	title_lbl.set_text(title + ": ")
	title_lbl.add_theme_color_override("font_color", Color(0.5, 0.5, 0.6))
	title_lbl.set_custom_minimum_size(Vector2(120, 0))
	hbox.add_child(title_lbl)
	
	var value_lbl = Label.new()
	value_lbl.set_text(default)
	value_lbl.add_theme_color_override("font_color", color)
	hbox.add_child(value_lbl)
	
	return value_lbl

func _setup_control_panel(parent: VBoxContainer) -> void:
	var ctrl_container = PanelContainer.new()
	var ctrl_style = StyleBoxFlat.new()
	ctrl_style.set_bg_color(Color(0.02, 0.02, 0.08, 0.92))
	ctrl_style.set_border_color(Color(0.1, 0.2, 0.4))
	ctrl_style.set_border_width_all(1)
	ctrl_container.add_theme_stylebox_override("panel", ctrl_style)
	parent.add_child(ctrl_container)
	
	var vbox = VBoxContainer.new()
	vbox.add_theme_constant_override("separation", 4)
	ctrl_container.add_child(vbox)
	
	var header = Label.new()
	header.set_text("─── CONTROLS ───")
	header.add_theme_color_override("font_color", Color(0.3, 0.7, 1.0))
	header.set_horizontal_alignment(HORIZONTAL_ALIGNMENT_CENTER)
	vbox.add_child(header)
	
	target_pos_spin = _add_spin_row(vbox, "Target Pos (mm):", 1.0, 0.1, 100.0, 0.1, 10.0)
	accel_g_spin = _add_spin_row(vbox, "Accel (G):", 2.0, 0.1, 10.0, 0.1, 2.0)
	active_stiffness_spin = _add_spin_row(vbox, "Active K (MN/m):", 8.0, 0.1, 100.0, 0.1, 15.0)
	active_damping_spin = _add_spin_row(vbox, "Active C (kN·s/m):", 40.0, 1.0, 200.0, 1.0, 60.0)
	threshold_spin = _add_spin_row(vbox, "Settle < (nm):", 0.1, 0.01, 10.0, 0.01, 0.1)
	time_scale_spin = _add_spin_row(vbox, "Time Scale:", 1.0, 0.01, 10.0, 0.01, 1.0)
	
	var btn_hbox = HBoxContainer.new()
	btn_hbox.add_theme_constant_override("separation", 10)
	vbox.add_child(btn_hbox)
	
	start_btn = Button.new()
	start_btn.set_text("▶ START")
	start_btn.set_custom_minimum_size(Vector2(100, 35))
	var btn_style_start = StyleBoxFlat.new()
	btn_style_start.set_bg_color(Color(0.0, 0.3, 0.1))
	btn_style_start.set_border_color(Color(0.0, 0.5, 0.2))
	btn_style_start.set_border_width_all(1)
	start_btn.add_theme_stylebox_override("normal", btn_style_start)
	start_btn.add_theme_color_override("font_color", Color(0.5, 1.0, 0.5))
	start_btn.pressed.connect(_on_start_pressed)
	btn_hbox.add_child(start_btn)
	
	reset_btn = Button.new()
	reset_btn.set_text("↺ RESET")
	reset_btn.set_custom_minimum_size(Vector2(100, 35))
	var btn_style_reset = StyleBoxFlat.new()
	btn_style_reset.set_bg_color(Color(0.3, 0.1, 0.0))
	btn_style_reset.set_border_color(Color(0.5, 0.2, 0.0))
	btn_style_reset.set_border_width_all(1)
	reset_btn.add_theme_stylebox_override("normal", btn_style_reset)
	reset_btn.add_theme_color_override("font_color", Color(1.0, 0.7, 0.3))
	reset_btn.pressed.connect(_on_reset_pressed)
	btn_hbox.add_child(reset_btn)

func _add_spin_row(parent: VBoxContainer, label: String, value: float, min_v: float, max_v: float, step: float, default: float) -> SpinBox:
	var hbox = HBoxContainer.new()
	parent.add_child(hbox)
	
	var lbl = Label.new()
	lbl.set_text(label)
	lbl.add_theme_color_override("font_color", Color(0.6, 0.6, 0.7))
	lbl.set_custom_minimum_size(Vector2(160, 0))
	hbox.add_child(lbl)
	
	var spin = SpinBox.new()
	spin.set_min(min_v)
	spin.set_max(max_v)
	spin.set_step(step)
	spin.set_value(default)
	spin.set_custom_minimum_size(Vector2(120, 0))
	hbox.add_child(spin)
	
	return spin

func _on_start_pressed() -> void:
	_apply_params()
	sim.start_simulation()

func _on_reset_pressed() -> void:
	sim.reset_simulation()

func _apply_params() -> void:
	sim.set_target_position(target_pos_spin.value * 1e-3)
	sim.set_accel_g(accel_g_spin.value)
	sim.set_active_stiffness(active_stiffness_spin.value * 1e6)
	sim.set_active_damping(active_damping_spin.value * 1e3)
	sim.set_settling_threshold_nm(threshold_spin.value)
	sim.set_time_scale(time_scale_spin.value)

func _process(delta: float) -> void:
	super._process(delta)
	_update_visuals()
	_update_status()

func _update_visuals() -> void:
	if sim == null or stage_visual == null:
		return
	
	var pos_m = sim.get_position()
	var pos_visual = pos_m * visual_scale
	stage_visual.set_position(Vector3(0, 0, -pos_visual))
	
	var target_visual = sim.get_target_position() * visual_scale
	target_marker.set_position(Vector3(0, 0.3, -target_visual))
	
	var phase = sim.get_phase()
	var coil_intensity = 0.0
	match phase:
		1:
			coil_intensity = 1.0
		2:
			coil_intensity = 0.8
		3:
			coil_intensity = 0.3
		4:
			coil_intensity = 0.1
		_:
			coil_intensity = 0.05
	
	for coil in coil_visuals:
		var mat = coil.get_surface_override_material(0) as StandardMaterial3D
		if mat:
			mat.set_emission_energy(coil_intensity * 5.0)
			mat.set_emission(Color(0.0, coil_intensity * 0.5, coil_intensity * 1.0))

func _update_status() -> void:
	if sim == null:
		return
	
	var phase = sim.get_phase()
	var phase_names = ["IDLE", "ACCELERATE", "DECELERATE", "SETTLE", "SETTLED"]
	var phase_colors = [
		Color(0.5, 0.5, 0.5),
		Color(0.2, 0.8, 1.0),
		Color(1.0, 0.8, 0.2),
		Color(1.0, 0.5, 0.1),
		Color(0.2, 1.0, 0.3)
	]
	
	if phase < phase_names.size():
		phase_label.set_text(phase_names[phase])
		phase_label.add_theme_color_override("font_color", phase_colors[phase])
	
	time_label.set_text("%.3f ms" % (sim.get_sim_time() * 1000.0))
	position_label.set_text("%.6f mm" % (sim.get_position() * 1000.0))
	velocity_label.set_text("%.6f m/s" % sim.get_velocity())
	
	var error_nm = sim.get_position_error_nm()
	if abs(error_nm) < 1.0:
		error_label.set_text("%.4f nm" % error_nm)
		error_label.add_theme_color_override("font_color", Color(0.2, 1.0, 0.3))
	else:
		error_label.set_text("%.2f nm" % error_nm)
		if abs(error_nm) < 100.0:
			error_label.add_theme_color_override("font_color", Color(1.0, 0.8, 0.2))
		else:
			error_label.add_theme_color_override("font_color", Color(1.0, 0.3, 0.3))
	
	if sim.is_settled():
		settling_label.set_text("ACHIEVED (%.3f ms)" % (sim.get_settling_time() * 1000.0))
		settling_label.add_theme_color_override("font_color", Color(0.2, 1.0, 0.3))
	elif phase == 3:
		settling_label.set_text("SETTLING...")
		settling_label.add_theme_color_override("font_color", Color(1.0, 0.8, 0.2))
	else:
		settling_label.set_text("---")
		settling_label.add_theme_color_override("font_color", Color(0.5, 0.5, 0.5))
