extends Control

var error_data: PackedFloat64Array = PackedFloat64Array()
var time_data: PackedFloat64Array = PackedFloat64Array()
var position_data: PackedFloat64Array = PackedFloat64Array()
var lorentz_data: PackedFloat64Array = PackedFloat64Array()

var display_mode: int = 0

var time_window_ms: float = 50.0
var amplitude_nm: float = 500.0
var settle_threshold_nm: float = 0.1

var grid_color: Color = Color(0.1, 0.2, 0.3, 0.6)
var axis_color: Color = Color(0.2, 0.4, 0.6)
var error_color: Color = Color(0.0, 1.0, 0.5)
var threshold_color: Color = Color(1.0, 0.1, 0.1)
var bg_color: Color = Color(0.01, 0.01, 0.03, 0.95)
var cursor_color: Color = Color(1.0, 1.0, 0.3, 0.5)

var auto_scale: bool = true

func _ready() -> void:
	set_process(true)

func _process(_delta: float) -> void:
	var sim_node = get_node_or_null("/root/Main/ReticleStageSim")
	if sim_node == null:
		sim_node = get_tree().get_root().find_child("ReticleStageSim", true, false)
	
	if sim_node and sim_node.get_sample_count() > 0:
		error_data = sim_node.get_error_time_series()
		time_data = sim_node.get_time_series()
		position_data = sim_node.get_position_time_series()
		lorentz_data = sim_node.get_lorentz_force_series()
		settle_threshold_nm = sim_node.get_settling_threshold_nm()
	
	queue_redraw()

func _draw() -> void:
	var rect = get_rect()
	var margin_left: float = 55.0
	var margin_right: float = 15.0
	var margin_top: float = 25.0
	var margin_bottom: float = 35.0
	
	var plot_rect = Rect2(
		rect.position.x + margin_left,
		rect.position.y + margin_top,
		rect.size.x - margin_left - margin_right,
		rect.size.y - margin_top - margin_bottom
	)
	
	draw_rect(plot_rect, bg_color)
	
	_draw_grid(plot_rect)
	_draw_axis_labels(plot_rect)
	
	if time_data.size() > 1:
		var current_time_ms = time_data[time_data.size() - 1]
		if auto_scale:
			_auto_adjust_scale(current_time_ms)
		
		match display_mode:
			0:
				_draw_error_waveform(plot_rect)
			1:
				_draw_position_waveform(plot_rect)
			2:
				_draw_lorentz_waveform(plot_rect)
		
		_draw_threshold_lines(plot_rect)
		_draw_cursor_info(plot_rect)

func _draw_grid(plot_rect: Rect2) -> void:
	var h_lines: int = 8
	var v_lines: int = 10
	
	for i in range(h_lines + 1):
		var y: float = plot_rect.position.y + (plot_rect.size.y * i / h_lines)
		draw_line(
			Vector2(plot_rect.position.x, y),
			Vector2(plot_rect.position.x + plot_rect.size.x, y),
			grid_color, 1.0
		)
	
	for i in range(v_lines + 1):
		var x: float = plot_rect.position.x + (plot_rect.size.x * i / v_lines)
		draw_line(
			Vector2(x, plot_rect.position.y),
			Vector2(x, plot_rect.position.y + plot_rect.size.y),
			grid_color, 1.0
		)
	
	draw_rect(plot_rect, axis_color, false, 2.0)

func _draw_axis_labels(plot_rect: Rect2) -> void:
	var font = ThemeDB.fallback_font
	var font_size: int = 10
	
	match display_mode:
		0:
			draw_string(font, Vector2(plot_rect.position.x - 50, plot_rect.position.y + 12), "nm", HORIZONTAL_ALIGNMENT_RIGHT, 45, font_size, axis_color)
		1:
			draw_string(font, Vector2(plot_rect.position.x - 50, plot_rect.position.y + 12), "mm", HORIZONTAL_ALIGNMENT_RIGHT, 45, font_size, axis_color)
		2:
			draw_string(font, Vector2(plot_rect.position.x - 50, plot_rect.position.y + 12), "N", HORIZONTAL_ALIGNMENT_RIGHT, 45, font_size, axis_color)
	
	draw_string(font, Vector2(plot_rect.position.x + plot_rect.size.x - 30, plot_rect.position.y + plot_rect.size.y + 15), "ms", HORIZONTAL_ALIGNMENT_LEFT, 30, font_size, axis_color)
	
	var time_labels: int = 5
	for i in range(time_labels + 1):
		var x: float = plot_rect.position.x + (plot_rect.size.x * i / time_labels)
		var t_ms: float = time_window_ms * i / time_labels
		draw_string(font, Vector2(x - 15, plot_rect.position.y + plot_rect.size.y + 15), "%.1f" % t_ms, HORIZONTAL_ALIGNMENT_LEFT, 30, font_size, Color(0.4, 0.5, 0.6))

func _draw_error_waveform(plot_rect: Rect2) -> void:
	if time_data.size() < 2:
		return
	
	var points: PackedVector2Array = PackedVector2Array()
	var start_idx = _find_start_index()
	
	for i in range(start_idx, time_data.size()):
		var t_ms: float = time_data[i]
		var e_nm: float = error_data[i]
		var x: float = plot_rect.position.x + (t_ms / time_window_ms) * plot_rect.size.x
		var y: float = plot_rect.position.y + plot_rect.size.y * 0.5 - (e_nm / amplitude_nm) * plot_rect.size.y * 0.5
		y = clampf(y, plot_rect.position.y, plot_rect.position.y + plot_rect.size.y)
		points.append(Vector2(x, y))
	
	if points.size() > 1:
		var glow_color = Color(error_color.r, error_color.g, error_color.b, 0.3)
		draw_polyline(points, glow_color, 4.0, true)
		draw_polyline(points, error_color, 2.0, true)

func _draw_position_waveform(plot_rect: Rect2) -> void:
	if time_data.size() < 2:
		return
	
	var points: PackedVector2Array = PackedVector2Array()
	var start_idx = _find_start_index()
	var pos_max: float = 0.015
	
	for i in range(start_idx, time_data.size()):
		var t_ms: float = time_data[i]
		var pos_mm: float = position_data[i] * 1000.0
		var x: float = plot_rect.position.x + (t_ms / time_window_ms) * plot_rect.size.x
		var y: float = plot_rect.position.y + plot_rect.size.y - (pos_mm / pos_max) * plot_rect.size.y
		y = clampf(y, plot_rect.position.y, plot_rect.position.y + plot_rect.size.y)
		points.append(Vector2(x, y))
	
	if points.size() > 1:
		draw_polyline(points, Color(0.3, 0.8, 1.0), 2.0, true)

func _draw_lorentz_waveform(plot_rect: Rect2) -> void:
	if time_data.size() < 2:
		return
	
	var points: PackedVector2Array = PackedVector2Array()
	var start_idx = _find_start_index()
	var force_max: float = 2000.0
	
	for i in range(start_idx, time_data.size()):
		var t_ms: float = time_data[i]
		var f: float = lorentz_data[i]
		var x: float = plot_rect.position.x + (t_ms / time_window_ms) * plot_rect.size.x
		var y: float = plot_rect.position.y + plot_rect.size.y * 0.5 - (f / force_max) * plot_rect.size.y * 0.5
		y = clampf(y, plot_rect.position.y, plot_rect.position.y + plot_rect.size.y)
		points.append(Vector2(x, y))
	
	if points.size() > 1:
		draw_polyline(points, Color(1.0, 0.6, 0.2), 2.0, true)

func _draw_threshold_lines(plot_rect: Rect2) -> void:
	if display_mode != 0:
		return
	
	var thresh_y_pos: float = plot_rect.position.y + plot_rect.size.y * 0.5 - (settle_threshold_nm / amplitude_nm) * plot_rect.size.y * 0.5
	var thresh_y_neg: float = plot_rect.position.y + plot_rect.size.y * 0.5 + (settle_threshold_nm / amplitude_nm) * plot_rect.size.y * 0.5
	
	var dash_len: float = 8.0
	var gap_len: float = 4.0
	
	var x: float = plot_rect.position.x
	while x < plot_rect.position.x + plot_rect.size.x:
		var end_x: float = minf(x + dash_len, plot_rect.position.x + plot_rect.size.x)
		draw_line(Vector2(x, thresh_y_pos), Vector2(end_x, thresh_y_pos), threshold_color, 1.5)
		draw_line(Vector2(x, thresh_y_neg), Vector2(end_x, thresh_y_neg), threshold_color, 1.5)
		x += dash_len + gap_len
	
	var font = ThemeDB.fallback_font
	draw_string(font, Vector2(plot_rect.position.x + plot_rect.size.x - 60, thresh_y_pos - 3), "%.1fnm" % settle_threshold_nm, HORIZONTAL_ALIGNMENT_LEFT, 60, 10, threshold_color)

func _draw_cursor_info(plot_rect: Rect2) -> void:
	if time_data.size() == 0:
		return
	
	var font = ThemeDB.fallback_font
	var font_size: int = 11
	
	var current_error: float = 0.0
	var current_time: float = 0.0
	if error_data.size() > 0:
		current_error = error_data[error_data.size() - 1]
	if time_data.size() > 0:
		current_time = time_data[time_data.size() - 1]
	
	var info_x: float = plot_rect.position.x + 5
	var info_y: float = plot_rect.position.y + 15
	
	var mode_names = ["ERROR (nm)", "POSITION (mm)", "LORENTZ (N)"]
	draw_string(font, Vector2(info_x, info_y), mode_names[display_mode], HORIZONTAL_ALIGNMENT_LEFT, 150, font_size, Color(0.8, 0.9, 1.0))
	
	draw_string(font, Vector2(plot_rect.position.x + plot_rect.size.x - 140, info_y), "t=%.2fms" % current_time, HORIZONTAL_ALIGNMENT_LEFT, 140, font_size, Color(0.6, 0.7, 0.8))

func _find_start_index() -> int:
	if time_data.size() == 0:
		return 0
	
	var time_start: float = 0.0
	if time_data[time_data.size() - 1] > time_window_ms:
		time_start = time_data[time_data.size() - 1] - time_window_ms
	
	for i in range(time_data.size()):
		if time_data[i] >= time_start:
			return i
	return 0

func _auto_adjust_scale(current_time_ms: float) -> void:
	if current_time_ms > time_window_ms:
		time_window_ms = current_time_ms * 1.1
	
	if display_mode == 0 and error_data.size() > 0:
		var max_err: float = 0.0
		var start_idx = _find_start_index()
		for i in range(start_idx, error_data.size()):
			var abs_err = absf(error_data[i])
			if abs_err > max_err:
				max_err = abs_err
		
		if max_err > amplitude_nm * 0.8:
			amplitude_nm = max_err * 1.5
		elif max_err < amplitude_nm * 0.1 and max_err > settle_threshold_nm * 2:
			amplitude_nm = max_err * 3.0
		if amplitude_nm < settle_threshold_nm * 5:
			amplitude_nm = settle_threshold_nm * 5.0
